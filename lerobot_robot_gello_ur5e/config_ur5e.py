from dataclasses import dataclass, field

from lerobot.cameras import CameraConfig
# Register the OpenCV config type for YAML decoding.
from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig  # noqa: F401
from lerobot.cameras.configs import ColorMode
from lerobot.cameras.realsense.configuration_realsense import RealSenseCameraConfig
from lerobot.robots import RobotConfig


@RobotConfig.register_subclass("gello_ur5e")
@dataclass
class UR5EConfig(RobotConfig):
    ip: str = "195.148.65.50"
    
    cameras: dict[str, CameraConfig] = field(
        default_factory=lambda: {
            "wrist": RealSenseCameraConfig(
                serial_number_or_name="02032305076",
                color_mode=ColorMode.RGB,
                use_depth=False,
                fps=5,
                width=424,
                height=240,
                warmup_s=5,
            ),

            # Add second camera here when ready.
            # Replace serial number with real camera serial.
            # "front": RealSenseCameraConfig(
            #     serial_number_or_name="PUT_SECOND_CAMERA_SERIAL_HERE",
            #     color_mode=ColorMode.RGB,
            #     use_depth=False,
            #     fps=5,
            #     width=424,
            #     height=240,
            #     warmup_s=5,
            # ),
        }
    )

    speed: float = 0.3
    acc: float = 0.3
    servoj_t: float = 1.0 / 100.0
    servoj_lookahead: float = 0.2
    servoj_gain: int = 100

    start_move_speed: float = 0.05
    start_move_acc: float = 0.05

    use_gripper: bool = True
    gripper_ip: str = "195.148.65.33"
    gripper_port: int = 502
    gripper_device_id: int = 65
    gripper_timeout: float = 0.2
    gripper_open_width_mm: float = 160.0
    gripper_closed_width_mm: float = 0.0
    gripper_force_n: float = 40.0
    gripper_min_delta_mm: float = 1.0
    gripper_min_period_s: float = 0.05

    # Connection alone never moves the arm. Enable only after checking the path.
    align_to_first_action: bool = False
    start_move_timeout_s: float = 180.0
    max_start_error_rad: float = 0.1
    max_joint_step_rad: float = 0.05


# Compatibility for scripts using the original name.
Ur5eRobotConfig = UR5EConfig
