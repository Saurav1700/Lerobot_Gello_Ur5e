
'''
This is the code for the configuration of gello leader in this case its Gello'''

from dataclasses import dataclass, field
from lerobot.teleoperators.config import TeleoperatorConfig

@TeleoperatorConfig.register_subclass("gello_ur5e")
@dataclass
class GelloConfig(TeleoperatorConfig):
    # Port to connect to the arm
    port: str = "/dev/ttyUSB0"
    baudrate: int = 57_600
    joint_ids: list[int] = field(default_factory=lambda: [1, 2, 3, 4, 5, 6])
    # Original GELLO offsets are radians, not the measured encoder-count offsets.
    # None selects the existing interactive calibration workflow.
    joint_offsets_rad: list[float] | None = None
    use_gripper: bool = True
    gripper_id: int = 7
    # Absolute encoder angles, in degrees, from the original GELLO gripper_config.
    gripper_open_deg: float | None = None
    gripper_closed_deg: float | None = None
    calibration_position: list[float] = field(default_factory=lambda: [0, -1.57, 1.57, -1.57, -1.57, -1.57])
    joint_signs: list[int] = field(default_factory=lambda: [1, 1, -1, 1, 1, 1])
    
    gripper_travel_counts: int = 575

    # Smoothing factor for Exponential Moving Average (EMA).
    # Range (0, 1]. 1 means no smoothing (instant update).
    # Lower values smooth out jitter but add latency.
    smoothing: float = 0.85
    # Whether to run device reading in a background thread.
    # This helps when USB communication is slow (e.g. long cables).
    use_async: bool = True
    stale_timeout_s: float = 0.5
    read_period_s: float = 0.01
    thread_join_timeout_s: float = 2.0
