# GELLO → UR5e with LeRobot

Teleoperate a Universal Robots UR5e using a passive GELLO leader arm through
LeRobot. The leader reads Dynamixel encoders; the follower sends joint targets
through `ur-rtde`. An optional OnRobot RG6 gripper follows the GELLO gripper,
and optional cameras provide image observations.

This repository contains two independently installable plugins:

| Directory / Python package | Purpose | LeRobot CLI option |
| --- | --- | --- |
| `lerobot_robot_gello_ur5e` | UR5e follower, RG6 and cameras | `--robot.type=gello_ur5e` |
| `lerobot_teleoperator_gello` | GELLO leader | `--teleop.type=gello_ur5e` |

Both plugins use the same CLI type name, in separate robot and teleoperator
registries. The documented compatibility baseline is **LeRobot 0.4.4 with
Python 3.10**. Newer LeRobot versions are not validated here.

## 1. Hardware and software prerequisites

The commands below target an Ubuntu/Debian Linux PC with USB and Ethernet access.
You need Git and Conda (Miniforge or Miniconda), plus:

- A UR5e with its controller and teach pendant, reachable from the PC over Ethernet.
- A powered GELLO leader and USB serial adapter. The driver configures arm motors
  as Dynamixel `xl330-m288` and the gripper motor as `xl330-m077`; different models
  may require code changes. Default arm IDs are 1–6, gripper ID 7, at 57,600 baud.
- Optionally, an OnRobot RG6 with a reachable Modbus TCP interface compatible
  with this driver's register map. Defaults are port 502 and device ID 65.
- Optionally, an Intel RealSense camera and/or an OpenCV-compatible USB camera.

Configure the PC's Ethernet interface for the robot network using addresses
assigned to your installation. The IPs, USB paths, camera serial numbers and
calibration values in this repository belong to the original setup; replace them.
Enable the controller's remote-control access as required by your UR setup,
release brakes and check the pendant before attempting motion.

**Motion behavior:** the supplied legacy preset enables automatic positioning.
Its first GELLO action causes a slow `moveJ` to that captured pose, followed by
live `servoJ` commands. There is no confirmation prompt in the stock CLI.
Check the full motion path, keep the workspace clear, keep the emergency stop
accessible and hold GELLO still during positioning. Controller joint-limit
checks do not establish that a path is collision-free. There is no independent
follower watchdog in this plugin.

## 2. Install on a new PC

Install Git if needed:

```bash
sudo apt update
sudo apt install git
```

Install Conda first if `conda` is unavailable. See the
[LeRobot 0.4.4 installation guide](https://huggingface.co/docs/lerobot/v0.4.4/en/installation)
for Miniforge setup and platform-specific dependency troubleshooting.

Create and activate the environment:

```bash
conda create -y -n lerobot python=3.10
conda activate lerobot
conda install -y -c conda-forge ffmpeg=7.1.1
python -m pip install --upgrade pip
python -m pip install 'lerobot==0.4.4'
```

Clone this repository and install both plugins from its root:

```bash
git clone https://github.com/Saurav1700/Lerobot_Gello_Ur5e.git
cd Lerobot_Gello_Ur5e
python -m pip install -e ./lerobot_robot_gello_ur5e -e ./lerobot_teleoperator_gello
python -m pip check
```

The follower installs `ur-rtde>=1.6,<2` and `pymodbus>=3.10,<4`; the leader
installs `dynamixel-sdk>=3.7,<4` and `pyserial>=3.5,<4`. Neither plugin installs
LeRobot itself. Editable installation means edits in this checkout are used
without reinstalling the code; keep the checkout in place.

For RealSense support, also run:

```bash
python -m pip install -e './lerobot_robot_gello_ur5e[realsense]'
```

Camera-free operation does not require this extra. Activate `lerobot` in each
new terminal and run the commands below from the repository root.

## 3. Check installation without moving hardware

```bash
python -c "from lerobot_robot_gello_ur5e import UR5E, UR5EConfig; from lerobot_teleoperator_gello import Gello, GelloConfig; print('Plugin imports OK')"
lerobot-teleoperate --robot.type=gello_ur5e --teleop.type=gello_ur5e --help
python scripts/check_gello_mapping.py --check-config
```

These check imports, CLI discovery and parsing of the supplied leader preset.
They do not connect to hardware or validate physical calibration.

## 4. Configure your installation

Make your own copy so the original preset remains available:

```bash
cp configs/gello_ur5e_legacy.yaml configs/my_setup.yaml
```

Edit `configs/my_setup.yaml` before using it:

| Setting | What to configure |
| --- | --- |
| `robot.ip` | Your UR5e controller address |
| `robot.gripper_ip` | Your OnRobot interface address, if enabled |
| `teleop.port` | Your GELLO USB serial device |
| `teleop.baudrate`, `joint_ids`, `gripper_id` | Actual motor bus configuration |
| `teleop.joint_offsets_rad`, `joint_signs` | Mapping verified for your leader |
| `teleop.gripper_open_deg`, `gripper_closed_deg` | Measured leader gripper endpoints |
| `robot.gripper_open_width_mm`, `gripper_closed_width_mm`, `gripper_force_n` | Follower travel and force appropriate for your setup |
| `robot.cameras` | Start with `{}`; add your cameras later |
| `robot.align_to_first_action` | `true` moves to the first leader pose; `false` requires manual alignment |
| `fps`, `robot.servoj_t` | Requested loop rate and servo duration; use matching values |
| `display_data` | Set `false` for initial/headless operation and the optional startup launcher |

For initial operation, replace the entire nested camera block with:

```yaml
  cameras: {}
```

To run without grippers, set **both** `robot.use_gripper: false` and
`teleop.use_gripper: false`. Leader and follower action features must match.

Find the serial device:

```bash
ls -l /dev/serial/by-id/
```

Prefer a stable `/dev/serial/by-id/...` path over `/dev/ttyUSB0`. On Ubuntu,
if your account lacks serial access:

```bash
sudo usermod -aG dialout "$USER"
```

Log out and back in for group membership to apply. Close any other application
using the same serial port. The driver expects motors to already have the
configured IDs and baud rate; it does not provision a new motor bus.

## 5. Verify mapping and calibration

Joint order is `joint_0` through `joint_5`: base, shoulder, elbow, wrist 1,
wrist 2, wrist 3. Actions use **radians**; gripper actions use **0=open, 1=closed**.

### Fixed offsets (the supplied preset)

The preset uses offsets `[4.712, 3.142, 3.142, 3.142, 3.142, 3.142]` radians and
signs `[1, 1, -1, 1, 1, 1]`. Each arm joint is computed as:

```text
angle_rad = (encoder_counts * 2*pi/4096 - joint_offsets_rad[i]) * joint_signs[i]
```

These offsets bypass saved calibration and interactive zeroing;
`calibration_position` is not applied. Existing encoder operating modes are
preserved, torque stays disabled, and angles are not automatically wrapped.
Do not assume the original offsets apply to a newly assembled leader.

Inspect GELLO alone, without constructing a UR5e or OnRobot connection:

```bash
python scripts/check_gello_mapping.py --config configs/my_setup.yaml
```

Move the leader slowly and verify signs, continuity and gripper endpoints.
Compare against the pendant at corresponding physical poses. You can supply
six actual pendant angles in degrees with `--reference-deg` to print differences.
Press Ctrl+C to stop. This helper uses synchronous reads and requires fixed offsets.

The shipped leader endpoints are 184.9855° open and 143.1855° closed. The
**YAML preset** maps them to RG6 widths of **100 mm open / 10 mm closed at 60 N**.
The Python config defaults differ: 160 mm / 0 mm at 40 N. Verify your own endpoints
and travel before enabling the follower gripper.

### Interactive calibration (alternative)

Set `teleop.joint_offsets_rad: null` to use measured calibration. On connection,
if no saved calibration exists for the teleoperator ID, the leader prompts you
to place it at `calibration_position` and open its gripper, then close the gripper
while keeping the arm still. Measurements are saved at LeRobot's calibration
path, which is reported in the log. Use a new `teleop.id` for a new calibration
instead of reusing another assembly's saved values.

To calibrate only the leader, use this Python snippet after replacing the port
and setting the reference pose to match your setup:

```python
from lerobot_teleoperator_gello import Gello, GelloConfig

leader = Gello(GelloConfig(
    id="my_gello",
    port="/dev/ttyUSB0",
    joint_offsets_rad=None,
    calibration_position=[0, -1.57, 1.57, -1.57, -1.57, -1.57],
    use_gripper=True,
    use_async=False,
))
try:
    leader.connect()
    print(leader.get_action())
finally:
    leader.disconnect()
```

Use the same ID, reference pose, motor settings and signs in your YAML afterward.
Interactive calibration configures extended-position mode; fixed-offset mode
preserves the existing mode. Validate the resulting joint angles before motion.

## 6. Start teleoperation

After configuring and verifying the mapping, start without cameras or visualization
at a requested 30 Hz:

```bash
lerobot-teleoperate \
  --config_path=configs/my_setup.yaml \
  --robot.cameras='{}' \
  --display_data=false \
  --teleop.use_async=false \
  --fps=30 \
  --robot.servoj_t=0.0333333333
```

If `align_to_first_action` is true, **this command moves the robot automatically**
to a snapshot of GELLO at startup, using `start_move_speed` and `start_move_acc`
(defaults: 0.05 rad/s and 0.05 rad/s²). Gripper commands begin after positioning.
Hold GELLO still until the move completes. Ctrl+C requests stopping and cleanup;
use the pendant emergency stop when needed.

With `align_to_first_action: false`, manually align first. The default maximum
initial joint error is 0.1 rad. During following, `max_joint_step_rad` defaults
to 0.05 rad and bounds targets relative to previous and measured positions;
excessive tracking error stops the command path.

The original preset requests 100 Hz (`servoj_t: 0.01`) and asynchronous leader
reads with smoothing 0.99. The YAML's inline `1 / 30 Hz` comment beside `0.01`
is incorrect: 0.01 seconds corresponds to 100 Hz. Requested rates are not measured
guarantees; USB, Modbus, camera and processing latency affect achieved timing.

### Optional launcher with an Enter gate

With `display_data: false` in your custom YAML:

```bash
python scripts/teleoperate_gello.py --config configs/my_setup.yaml --fps 30
```

This launcher **moves the arm before asking for Enter**, then waits for Enter to
start live following. It checks alignment again at that point. It forces
synchronous leader reads and sets the servo duration from `--fps`.

## 7. Add cameras

Once camera-free operation works, restore only the cameras you actually have in
`robot.cameras`. The supplied YAML shows RealSense and OpenCV entries. Replace
`serial_number_or_name` and `index_or_path` (the original OpenCV device is
`/dev/video6`), and choose supported resolutions and frame rates. Install the
RealSense extra for that camera. Remove the `--robot.cameras='{}'` override to
use the YAML camera configuration.

The Python config has a default wrist RealSense camera at 5 fps, while the YAML
requests two cameras at 30 fps with different device identifiers. Explicitly
set `cameras: {}` when no camera is connected. Camera frame rate and control-loop
rate are distinct; a 100 Hz request does not imply 100 unique images per second.

## 8. Troubleshooting

| Symptom | Check / action |
| --- | --- |
| `lerobot-teleoperate` missing or unknown `gello_ur5e` type | Activate the correct environment and reinstall both editable packages; repeat the import and help checks. |
| Missing Python dependency or incompatible API | Run `python -m pip check` and `python -m pip show lerobot`; use the documented 0.4.4 baseline. |
| Serial permission denied / device missing | Check USB power, the configured path, `dialout` membership and whether another process owns the port. |
| Missing or incorrect GELLO motor ID | Check power, wiring, IDs, model compatibility and baud rate. |
| GELLO reader stale / stopped | Inspect the underlying read error; try synchronous reads (`--teleop.use_async=false`) at 30 Hz and check USB communication. |
| RTDE connection failure | Check controller IP, PC subnet, Ethernet, controller remote access and pendant status. |
| Modbus/gripper failure | Check the gripper IP, port, device ID and compatible register map; disable grippers on both sides to isolate arm operation. |
| Camera connection failure | Correct the serial/device path and stream profile, or use `--robot.cameras='{}'`. |
| Initial alignment or tracking-error exception | Recheck physical pose, units, offsets and signs; do not simply increase limits to bypass a mapping error. |
| Optional launcher rejects configuration | Set `display_data: false` in the YAML and ensure both gripper flags agree. |

## Repository layout and development

| Path | Contents |
| --- | --- |
| `lerobot_robot_gello_ur5e/` | Follower configuration, RTDE control and OnRobot driver |
| `lerobot_teleoperator_gello/` | Leader configuration, calibration and encoder reading |
| `configs/gello_ur5e_legacy.yaml` | Original hardware-specific preset |
| `scripts/check_gello_mapping.py` | Leader-only inspection; `--check-config` opens no hardware |
| `scripts/teleoperate_gello.py` | Startup positioning followed by an Enter gate |
| `scripts/move_ur5e_to_gello_pose.py` | One-shot tool with a hard-coded target; reads robot state by default, moves only with `--execute` |

The one-shot movement script does **not** read the current GELLO pose. Inspect
its `TARGET_Q` before considering execution.

Some development checkouts contain `tests/`, but that directory is currently
ignored by Git and is not included in a fresh clone. If present locally, run
simulated checks as separate processes because they install fake dependencies:

```bash
python tests/test_gello_simulated.py
python tests/test_ur5e_simulated.py
python tests/test_plugin_integration.py
```

Integration checks require real installed dependencies but do not connect to
hardware. They do not validate collision safety, physical mapping, the gripper
register map, camera profiles or achievable control timing. Generated `outputs/`
are also ignored by Git. This guide covers teleoperation; dataset recording and
policy training are separate LeRobot workflows.
