<<<<<<< HEAD
# GELLO → UR5e LeRobot plugins

Two independently installable plugins, tested with **LeRobot 0.4.4**:

| Project | Python package | CLI type |
| --- | --- | --- |
| `lerobot_robot_gello_ur5e` | `lerobot_robot_gello_ur5e` | `--robot.type=gello_ur5e` |
| `lerobot_teleoperator_gello` | `lerobot_teleoperator_gello` | `--teleop.type=gello_ur5e` |

Each folder contains the driver modules and its own `pyproject.toml`. There are
no wrapper packages or symlinks. Use the new package names in Python imports.

## Installation

Use your existing LeRobot Conda environment (Python 3.10+). From this repository root:

```bash
conda activate lerobot
python -m pip install -e ./lerobot_robot_gello_ur5e -e ./lerobot_teleoperator_gello
```

LeRobot must already be installed; neither plugin installs, upgrades or downgrades
it. The follower declares `ur-rtde` and `pymodbus`; the leader declares
`dynamixel-sdk` and `pyserial`, which are not guaranteed by a base LeRobot install.
Pip reuses installed versions that satisfy these requirements.

If the RealSense SDK is missing, install the optional camera dependency with
`python -m pip install -e './lerobot_robot_gello_ur5e[realsense]'`.
It is unnecessary when using `--robot.cameras='{}'`.
LeRobot 0.4.4 was tested; run the integration checks below with your installed
version before connecting hardware.

```python
from lerobot_robot_gello_ur5e import UR5E, UR5EConfig, OnRobotGripper
from lerobot_teleoperator_gello import Gello, GelloConfig
```

## CLI

Check discovery without connecting hardware:

```bash
lerobot-teleoperate --robot.type=gello_ur5e --teleop.type=gello_ur5e --help
```

After calibration and physical alignment, a camera-free starting command is:

```bash
lerobot-teleoperate \
  --robot.type=gello_ur5e --robot.id=ur5e \
  --robot.ip=195.148.65.50 --robot.gripper_ip=195.148.65.33 \
  --robot.cameras='{}' --robot.servoj_t=0.01 \
  --teleop.type=gello_ur5e --teleop.id=gello \
  --teleop.port=/dev/ttyUSB0 --fps=100
```

Replace addresses and serial port with your hardware. The robot defaults to a
specific wrist-camera serial number when cameras are not overridden.
A requested 100 Hz rate is not a measured guarantee: serial, camera and Modbus
latency affect the loop. With the default 5 fps camera, do not assume 100 Hz
observation updates. There is no independent follower watchdog.

The legacy preset captures the first GELLO action and moves slowly to those
joint angles before servo following starts. Hold GELLO still during positioning.
Gripper commands start after the arm reaches the target. All leader motors
remain passive. The preset uses synchronous reads to avoid stale startup data.

## Verification

Run each simulated test separately (they install fake dependencies in-process):

```bash
python tests/test_gello_simulated.py
python tests/test_ur5e_simulated.py
```

With the real plugins and dependencies installed:

```bash
python tests/test_plugin_integration.py
```

The integration checks exercise installed distribution discovery, real LeRobot
config parsing and device factories, camera config construction, feature
compatibility. They do not connect to hardware or validate
motion, register-map correctness, camera stream profiles or achievable timing.

## Original six-joint GELLO mapping

Use the supplied preset to retain the original YAML mapping:

```bash
lerobot-teleoperate --config_path=configs/gello_ur5e_legacy.yaml
```

The preset uses your FTDI serial path, motor IDs 1–6, offsets
`[4.712, 3.142, 3.142, 3.142, 3.142, 3.142]` **in radians**, and signs
`[1, 1, -1, 1, 1, 1]`. The conversion is
`(encoder_counts * 2*pi/4096 - joint_offsets_rad[i]) * joint_signs[i]`.
Fixed offsets bypass saved calibration and interactive zeroing; the
`calibration_position` field is not applied. No angle wrapping or automatic
start-pose adjustment is performed. The existing encoder operating mode is
preserved in this mode, matching the original GELLO torque-disabled setup.

The preset now enables both grippers. GELLO motor 7 uses the earlier endpoints
184.9855° open and 143.1855° closed. It sends 0=open and 1=closed to the OnRobot
RG6 at 195.148.65.33, mapped to 160 mm open and 0 mm closed at 40 N. These endpoint
values came from the old configuration; verify the leader's reported gripper
value before teleoperation. The leader motor remains passive.

Run `python scripts/check_gello_mapping.py` to check arm angles and the normalized
gripper value without connecting to the UR5e or OnRobot. If the startup reader
stale error returns, use `--teleop.use_async=false --fps=30` for the previously
suggested synchronous check. Set **both** `robot.use_gripper` and
`teleop.use_gripper` to false to return to six-joint operation.

The preset requests 100 Hz and uses smoothing 0.99, matching the upstream GELLO
coefficient. Filtering runs on the asynchronous reader, so dynamic timing is
not guaranteed identical to the old application. Joint mapping is identical
for identical encoder readings. Physical alignment and the follower's target
limits still apply; this preset commands a startup move to the first GELLO action. Confirm the
reported joint angles against the UR pendant before driving the follower.


## CLI startup: follow the current GELLO pose

```bash
lerobot-teleoperate --config_path=configs/gello_ur5e_legacy.yaml --fps=30 --robot.servoj_t=0.0333333333
```

The preset enables `robot.align_to_first_action`. The first action received
from GELLO is captured as the startup target. The UR5e moves there once using
`moveJ` at `start_move_speed` / `start_move_acc` (both default 0.05), then
continues with servoJ. There is no Enter prompt and no fixed `safe_start_q`.
Clear the entire path and hold GELLO still during the move. Controller target
limits are checked, but this does not guarantee a collision-free path.
Ctrl+C during startup requests stopJ; errors stop the move and abort the run.

`--robot.align_to_first_action=false` disables automatic alignment and requires
manual alignment as before. The optional `scripts/teleoperate_gello.py` launcher
still provides the Enter gate if desired; the stock CLI starts following directly.
=======
# Lerobot_Gello_Ur5e
Lerobot integration of gello
>>>>>>> 633d058c6a8802a6535575d1b24af4ffa6ae5027
