"""GELLO teleoperator interface using Dynamixel motors.

Implements the LeRobot Teleoperator interface for the GELLO 7-DOF leader device.
Reads joint positions from Dynamixel servos and applies calibration offsets.
"""

import logging
import json
import time
import math
from threading import Thread, Event, Lock
from dataclasses import dataclass
from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.dynamixel import (
    DynamixelMotorsBus,
    OperatingMode,
)
from lerobot.teleoperators import Teleoperator
from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError
from .config_gello import GelloConfig
from pathlib import Path
logger = logging.getLogger(__name__)

@dataclass
class GelloCalibration:
    joint_offsets: dict[str, int] # map from motor name to offset in counts
    gripper_open_position: int # motor counts for the open position
    gripper_closed_position: int # motor counts for the closed position

class Gello(Teleoperator):
    """
    This is the GELLO teleoperator for the ur5 robot.
    The hardware is from Phillip Wu: https://wuphilipp.github.io/gello_site/
    """

    config_class = GelloConfig
    name = "gello"
    RAD_PER_COUNT = 2 * math.pi / 4096
    JOINT_NAMES = ["joint_0", "joint_1", "joint_2", "joint_3", "joint_4", "joint_5"]

    def __init__(self, config: GelloConfig):
        self.config = config
        super().__init__(config)
        self._validate_config()
        self.bus = DynamixelMotorsBus(
            port=self.config.port,
            motors={
                **{name: Motor(motor_id, "xl330-m288", MotorNormMode.RANGE_M100_100)
                   for name, motor_id in zip(self.JOINT_NAMES, config.joint_ids)},
                **({"gripper": Motor(config.gripper_id, "xl330-m077", MotorNormMode.RANGE_0_100)}
                   if config.use_gripper else {}),
            }
        )

        self.thread: Thread | None = None
        self.stop_event: Event | None = None
        self.lock: Lock = Lock()
        self.latest_action: dict[str, float] | None = None
        self.last_read_time = 0.0
        self.read_error = None

    @property
    def action_features(self) -> dict[str, type]:
        return {motor: float for motor in self.bus.motors}

    @property
    def feedback_features(self) -> dict[str, type]:
        return {}

    @property
    def is_connected(self) -> bool:
        return self.bus.is_connected

    def _validate_config(self):
        c = self.config
        if (len(c.joint_ids) != 6 or len(set(c.joint_ids)) != 6
                or any(type(v) is not int or not 0 <= v <= 252 for v in c.joint_ids)):
            raise ValueError("joint_ids must contain six distinct Dynamixel IDs in [0, 252]")
        if c.use_gripper:
            if type(c.gripper_id) is not int or not 0 <= c.gripper_id <= 252:
                raise ValueError("gripper_id must be a Dynamixel ID in [0, 252]")
            if c.gripper_id in c.joint_ids:
                raise ValueError("Joint IDs must not overlap the gripper ID")
        if c.joint_offsets_rad is not None:
            if len(c.joint_offsets_rad) != 6 or not all(math.isfinite(v) for v in c.joint_offsets_rad):
                raise ValueError("joint_offsets_rad must contain six finite radian offsets")
            if c.use_gripper:
                endpoints = (c.gripper_open_deg, c.gripper_closed_deg)
                if any(v is None or not math.isfinite(v) for v in endpoints):
                    raise ValueError("Fixed-offset gripper mode requires finite gripper_open_deg and gripper_closed_deg")
                if c.gripper_open_deg == c.gripper_closed_deg:
                    raise ValueError("Gripper open and closed endpoints must differ")
        if len(c.joint_signs) != 6 or any(v not in (-1, 1) for v in c.joint_signs):
            raise ValueError("joint_signs must contain six +1/-1 values")
        if len(c.calibration_position) != 6 or not all(math.isfinite(v) for v in c.calibration_position):
            raise ValueError("calibration_position must contain six finite radian values")
        if not 0 < c.smoothing <= 1:
            raise ValueError("smoothing must be in (0, 1]")
        for name in ('stale_timeout_s', 'read_period_s', 'thread_join_timeout_s'):
            value = getattr(c, name)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and positive")

    def connect(self, calibrate: bool = True) -> None:
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")
        self._stop_read_thread()
        try:
            self.bus.connect(handshake=False)
            self.bus.set_baudrate(self.config.baudrate)
            # Public ping API verifies IDs and models at the configured baud rate.
            for motor in self.bus.motors.values():
                expected = self.bus.model_number_table[motor.model]
                if self.bus.ping(motor.id) != expected:
                    raise ConnectionError(f"Missing or incorrect GELLO motor ID {motor.id}")
            # Configure before measuring offsets, since operating modes affect counts.
            self.configure()
            self._load_calibration()
            if not self.is_calibrated:
                if not calibrate:
                    raise RuntimeError("No GELLO calibration. Connect with calibrate=True first.")
                self.calibrate()
            self.latest_action = self._process_action(
                self.bus.sync_read("Present_Position", normalize=False))
            self.last_read_time = time.monotonic()
            self.read_error = None
            if self.config.use_async:
                self._start_read_thread()
        except BaseException:
            try:
                self.disconnect()
            except Exception:
                logger.exception("GELLO cleanup after connection failure failed")
            raise

    @property
    def is_calibrated(self) -> bool:
        return self.config.joint_offsets_rad is not None or isinstance(self.calibration, GelloCalibration)

    def calibrate(self) -> None:
        if self.config.joint_offsets_rad is not None:
            # Never replace the supplied mapping with a measured zero pose.
            return
        if not self.is_connected:
            raise DeviceNotConnectedError("Connect GELLO before calibrating")
        if self.thread is not None and self.thread.is_alive():
            raise RuntimeError("Disconnect and reconnect before recalibrating; reader is running")
        self.bus.disable_torque()
        input(f"Place GELLO at the physical pose corresponding to UR joint angles "
              f"{self.config.calibration_position} radians"
              + (", OPEN its gripper" if self.config.use_gripper else "") + ", then press ENTER: ")
        opened = self.bus.sync_read("Present_Position", normalize=False)
        closed = {}
        if self.config.use_gripper:
            input("Keep the arm still, CLOSE the GELLO gripper, then press ENTER: ")
            closed = self.bus.sync_read("Present_Position", normalize=False)
        calibration = GelloCalibration(
            joint_offsets={motor: opened[motor] for motor in self.JOINT_NAMES},
            gripper_open_position=opened.get('gripper', 0),
            gripper_closed_position=closed.get('gripper', 1),
        )
        self._validate_calibration(calibration)
        self.calibration_fpath.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.calibration_fpath.with_suffix('.json.tmp')
        temporary.write_text(json.dumps(calibration.__dict__, indent=2))
        temporary.replace(self.calibration_fpath)
        self.calibration = calibration
        self.latest_action = None
        logger.info("Calibration saved to %s", self.calibration_fpath)

    def configure(self) -> None:
        """Keep every motor passive, including the gripper trigger."""
        self.bus.disable_torque()
        self.bus.configure_motors()
        # Original GELLO only disables torque. Preserve its existing encoder mode
        # in fixed-offset mode rather than changing the position reference.
        if self.config.joint_offsets_rad is None:
            for motor in self.bus.motors:
                self.bus.write("Operating_Mode", motor, OperatingMode.EXTENDED_POSITION.value)
        self.bus.disable_torque()

    def setup_motors(self) -> None:
        for motor in reversed(self.bus.motors):
            input(f"Connect the controller board to the '{motor}' motor only and press enter.")
            self.bus.setup_motor(motor)
            print(f"'{motor}' motor id set to {self.bus.motors[motor].id}")

    def _process_action(self, raw_action: dict[str, int]) -> dict[str, float]:
        # Preserve multi-turn radians; gripper is 0=open, 1=closed.
        if not self.is_calibrated:
            raise RuntimeError("GELLO has no calibration")
        if set(raw_action) != set(self.action_features) or not all(math.isfinite(v) for v in raw_action.values()):
            raise ValueError("Expected one finite encoder reading for each configured motor")
        result = {}
        for idx, motor in enumerate(self.JOINT_NAMES):
            sign = self.config.joint_signs[idx]
            if self.config.joint_offsets_rad is not None:
                # Original GELLO: (raw angle in radians - offset in radians) * sign.
                angle_rad = (raw_action[motor] * self.RAD_PER_COUNT
                             - self.config.joint_offsets_rad[idx]) * sign
            else:
                offset = self.calibration.joint_offsets[motor]
                angle_rad = sign * (raw_action[motor] - offset) * self.RAD_PER_COUNT + self.config.calibration_position[idx]
            result[motor] = angle_rad
        if self.config.use_gripper:
            if self.config.joint_offsets_rad is not None:
                angle_deg = raw_action["gripper"] * 360.0 / 4096
                result["gripper"] = (angle_deg - self.config.gripper_open_deg) / (self.config.gripper_closed_deg - self.config.gripper_open_deg)
            else:
                result["gripper"] = (raw_action["gripper"] - self.calibration.gripper_open_position) / (self.calibration.gripper_closed_position - self.calibration.gripper_open_position)
            result["gripper"] = max(0.0, min(1.0, result["gripper"]))
        return result

    def _read_loop(self, stop_event: Event) -> None:
        while not stop_event.is_set():
            try:
                action = self._process_action(self.bus.sync_read("Present_Position", normalize=False))
                with self.lock:
                    if self.latest_action is None or self.read_error is not None:
                        self.latest_action = action
                    else:
                        alpha = self.config.smoothing
                        self.latest_action = {k: alpha*v + (1-alpha)*self.latest_action[k]
                                              for k, v in action.items()}
                    self.last_read_time = time.monotonic()
                    self.read_error = None
            except Exception as exc:
                with self.lock:
                    first_failure = self.read_error is None
                    self.read_error = exc
                if first_failure:
                    logger.exception("GELLO encoder read failed")
            stop_event.wait(self.config.read_period_s)

    def _start_read_thread(self) -> None:
        self._stop_read_thread()
        self.stop_event = Event()
        self.thread = Thread(target=self._read_loop, args=(self.stop_event,),
                             name="gello-reader", daemon=True)
        self.thread.start()

    def _stop_read_thread(self) -> None:
        if self.stop_event is not None:
            self.stop_event.set()
        if self.thread is not None and self.thread.is_alive():
            self.thread.join(timeout=self.config.thread_join_timeout_s)
            if self.thread.is_alive():
                # Retain references and do not close a bus still used by the reader.
                raise RuntimeError("GELLO reader did not stop; retry disconnect after the bus read returns")
        self.thread = None
        self.stop_event = None

    def get_action(self) -> dict[str, float]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")
        if self.config.use_async:
            with self.lock:
                if self.read_error is not None:
                    raise RuntimeError("GELLO encoder reader failed") from self.read_error
                if (self.latest_action is None or self.thread is None or not self.thread.is_alive()
                        or time.monotonic() - self.last_read_time > self.config.stale_timeout_s):
                    raise RuntimeError("GELLO action is stale or the reader has stopped")
                return self.latest_action.copy()
        return self._process_action(self.bus.sync_read("Present_Position", normalize=False))

    def send_feedback(self, feedback: dict[str, float]) -> None:
        if feedback:
            raise NotImplementedError("GELLO force feedback is not implemented")

    def disconnect(self) -> None:
        self._stop_read_thread()
        try:
            if self.bus.is_connected:
                self.bus.disconnect()
        finally:
            with self.lock:
                self.latest_action = None
                self.last_read_time = 0.0
                self.read_error = None

    @classmethod
    def _validate_calibration(cls, calibration):
        if set(calibration.joint_offsets) != set(cls.JOINT_NAMES):
            raise ValueError("Calibration must contain exactly six joint offsets")
        values = [*calibration.joint_offsets.values(), calibration.gripper_open_position,
                  calibration.gripper_closed_position]
        if not all(math.isfinite(v) for v in values):
            raise ValueError("Calibration counts must be finite")
        if calibration.gripper_open_position == calibration.gripper_closed_position:
            raise ValueError("Gripper open and closed positions must differ")

    def _load_calibration(self, fpath: Path | None = None) -> None:
        if self.config.joint_offsets_rad is not None:
            self.calibration = None
            return
        if fpath is None:
            fpath = self.calibration_fpath
        if fpath.is_file():
            with open(fpath, "r") as f:
                self.calibration = GelloCalibration(**json.load(f))
            self._validate_calibration(self.calibration)
            logger.info(f"Calibration loaded from {fpath}")
        else:
            logger.info(f"No calibration file found at {fpath}")
            self.calibration = None