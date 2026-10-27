"""UR5e follower with OnRobot RG6 gripper.

The caller owns loop pacing; servoj_t must match its intended control period.
Camera and Modbus I/O are synchronous and can reduce the achieved update rate.
"""
import logging
import math
import time
from typing import Any

from lerobot.cameras import make_cameras_from_configs
from lerobot.robots import Robot
from lerobot.utils.errors import DeviceNotConnectedError
import rtde_control
import rtde_receive

from .onrobot_gripper import OnRobotGripper
from .config_ur5e import UR5EConfig

logger = logging.getLogger(__name__)


class UR5E(Robot):
    config_class = UR5EConfig
    name = "gello_ur5e"

    def __init__(self, config: UR5EConfig):
        super().__init__(config)
        self.config = config
        self._validate_config()
        self.cameras = make_cameras_from_configs(config.cameras)
        self.rtde_ctrl = None
        self.rtde_rec = None
        self.gripper = OnRobotGripper(
            ip=config.gripper_ip, port=config.gripper_port,
            device_id=config.gripper_device_id, timeout=config.gripper_timeout,
        ) if config.use_gripper else None
        self._startup_pending = True
        self._last_q = None
        self._last_gripper_width = None
        self._last_gripper_time = 0.0

    def _validate_config(self):
        c = self.config
        for name in ('speed', 'acc', 'servoj_t', 'start_move_speed', 'start_move_acc',
                     'max_start_error_rad', 'max_joint_step_rad', 'gripper_timeout', 'start_move_timeout_s'):
            value = getattr(c, name)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be finite and positive')
        if not 0.03 <= c.servoj_lookahead <= 0.2 or not 100 <= c.servoj_gain <= 2000:
            raise ValueError('Invalid servo lookahead or gain')
        if not 0 <= c.gripper_closed_width_mm < c.gripper_open_width_mm <= 160:
            raise ValueError('Gripper widths must satisfy 0 <= closed < open <= 160 mm')
        if not 25 <= c.gripper_force_n <= 120:
            raise ValueError('RG6 force must be between 25 and 120 N')
        for name in ('gripper_min_delta_mm', 'gripper_min_period_s'):
            value = getattr(c, name)
            if not math.isfinite(value) or value < 0:
                raise ValueError(f'{name} must be finite and nonnegative')

    @staticmethod
    def _joint_vector(values):
        values = [float(v) for v in values]
        if len(values) != 6 or not all(math.isfinite(v) for v in values):
            raise ValueError('Expected six finite joint angles in radians')
        return values

    @property
    def action_features(self):
        return {**{f'joint_{i}': float for i in range(6)},
                **({'gripper': float} if self.config.use_gripper else {})}

    @property
    def observation_features(self):
        return {**self.action_features, **{
            name: (cam.height, cam.width, 3) for name, cam in self.cameras.items()
        }}

    @property
    def is_connected(self):
        return bool(self.rtde_ctrl is not None and self.rtde_rec is not None
                    and self.rtde_ctrl.isConnected() and self.rtde_rec.isConnected()
                    and (self.gripper is None or self.gripper.is_connected)
                    and all(cam.is_connected for cam in self.cameras.values()))

    def connect(self, calibrate: bool = True):
        if self.is_connected:
            return
        if self.rtde_ctrl is not None or self.rtde_rec is not None:
            self.disconnect()
        try:
            self.rtde_ctrl = rtde_control.RTDEControlInterface(self.config.ip)
            self.rtde_rec = rtde_receive.RTDEReceiveInterface(self.config.ip)
            if self.gripper is not None:
                self.gripper.connect()
            for cam in self.cameras.values():
                cam.connect()
            if not self.is_connected:
                raise ConnectionError('Not all UR5e devices connected')
            self.configure()
        except Exception:
            try:
                self.disconnect()
            except Exception:
                logger.exception('Cleanup after connection failure failed')
            raise

    def configure(self):
        pass

    @property
    def is_calibrated(self):
        # UR joint encoders are calibrated by the controller; GELLO calibrates separately.
        return True

    def calibrate(self):
        pass

    def disconnect(self):
        errors = []
        def attempt(fn):
            try:
                fn()
            except Exception as exc:
                errors.append(exc)
        if self.rtde_ctrl is not None:
            attempt(self.rtde_ctrl.servoStop)
            attempt(self.rtde_ctrl.stopScript)
            attempt(self.rtde_ctrl.disconnect)
        if self.rtde_rec is not None:
            attempt(self.rtde_rec.disconnect)
        if self.gripper is not None:
            attempt(self.gripper.disconnect)
        for cam in self.cameras.values():
            if cam.is_connected:
                attempt(cam.disconnect)
        self.rtde_ctrl = self.rtde_rec = None
        self._startup_pending = True
        self._last_q = self._last_gripper_width = None
        self._last_gripper_time = 0.0
        if errors:
            raise RuntimeError('One or more devices failed to disconnect cleanly') from errors[0]

    def _require_connection(self):
        if not self.is_connected:
            raise DeviceNotConnectedError(f'{self} is not connected.')

    def _normalized_width(self, width):
        c = self.config
        return max(0.0, min(1.0, (c.gripper_open_width_mm - width) /
                            (c.gripper_open_width_mm - c.gripper_closed_width_mm)))

    def get_observation(self) -> dict[str, Any]:
        self._require_connection()
        q = self._joint_vector(self.rtde_rec.getActualQ())
        obs = {f'joint_{i}': v for i, v in enumerate(q)}
        if self.gripper is not None:
            obs['gripper'] = self._normalized_width(self.gripper.get_width())
        for name, cam in self.cameras.items():
            obs[name] = cam.async_read()
        return obs

    def send_action(self, action: dict[str, float]) -> dict[str, float]:
        self._require_connection()
        try:
            return self._send_action(action)
        except Exception:
            # A rejected action must not leave the previous servo command active.
            try:
                self.rtde_ctrl.servoStop()
            except Exception:
                logger.exception('Failed to stop servo after action failure')
            self._last_q = None
            raise

    def _align_to_first_action(self, target):
        """Move once to the captured first action; no servo/gripper commands yet."""
        c = self.config
        if not self.rtde_ctrl.isJointsWithinSafetyLimits(target):
            raise ValueError('Initial GELLO target is outside controller safety limits')
        logger.info('Moving to initial GELLO angles %s at %.3f rad/s; hold GELLO still',
                    target, c.start_move_speed)
        previous = self.rtde_ctrl.getAsyncOperationProgress()
        try:
            if not self.rtde_ctrl.moveJ(target, c.start_move_speed, c.start_move_acc, True):
                raise RuntimeError('Startup moveJ rejected')
            deadline = time.monotonic() + c.start_move_timeout_s
            started = False
            while True:
                self._require_connection()
                if self.rtde_rec.isProtectiveStopped() or self.rtde_rec.isEmergencyStopped():
                    raise RuntimeError('Robot stopped during startup move')
                progress = self.rtde_ctrl.getAsyncOperationProgress()
                started = started or progress >= 0
                if progress < 0 and (started or progress != previous):
                    break
                if time.monotonic() >= deadline:
                    raise TimeoutError('Startup move timed out')
                time.sleep(0.05)
            actual = self._joint_vector(self.rtde_rec.getActualQ())
            if max(abs(a-b) for a, b in zip(target, actual)) > 0.01:
                raise RuntimeError('Startup move finished without reaching the GELLO target')
            logger.info('Initial GELLO pose reached; starting servo teleoperation')
        except BaseException:
            try:
                self.rtde_ctrl.stopJ(0.5)
            except Exception:
                logger.exception('Failed to stop startup move')
            raise

    def _send_action(self, action):
        if set(action) != set(self.action_features):
            raise ValueError(f'Expected action keys {list(self.action_features)}')
        requested = self._joint_vector(action[f'joint_{i}'] for i in range(6))
        if self.gripper is not None:
            grip = float(action['gripper'])
            if not math.isfinite(grip):
                raise ValueError('Gripper action must be finite')
            grip = max(0.0, min(1.0, grip))
        c = self.config
        if self._startup_pending:
            # A failure cannot trigger another automatic move on the next action.
            self._startup_pending = False
            if c.align_to_first_action:
                self._align_to_first_action(requested)
        actual = self._joint_vector(self.rtde_rec.getActualQ())
        if self._last_q is None and max(abs(a-b) for a, b in zip(requested, actual)) > c.max_start_error_rad:
            raise ValueError('Align GELLO with the UR5e before sending the first action')
        # Limit each target relative to actual position and the previous command.
        base = actual if self._last_q is None else self._last_q
        q = []
        for target, previous, measured in zip(requested, base, actual):
            lo = max(previous, measured) - c.max_joint_step_rad
            hi = min(previous, measured) + c.max_joint_step_rad
            if lo > hi:
                raise RuntimeError('UR5e tracking error exceeded the target-step allowance')
            q.append(max(lo, min(hi, target)))
        if self.gripper is not None:
            width = round(c.gripper_open_width_mm - grip *
                          (c.gripper_open_width_mm - c.gripper_closed_width_mm), 1)
            now = time.monotonic()
            if self._last_gripper_width is None or (
                abs(width - self._last_gripper_width) >= c.gripper_min_delta_mm
                and now - self._last_gripper_time >= c.gripper_min_period_s
            ):
                self.gripper.move_rg6(width, c.gripper_force_n)
                self._last_gripper_width = width
                self._last_gripper_time = time.monotonic()
        if not self.rtde_ctrl.servoJ(q, c.speed, c.acc, c.servoj_t,
                                    c.servoj_lookahead, c.servoj_gain):
            raise RuntimeError('UR5e rejected servoJ command')
        self._last_q = q
        result = {f'joint_{i}': v for i, v in enumerate(q)}
        if self.gripper is not None:
            result['gripper'] = self._normalized_width(self._last_gripper_width)
        return result
