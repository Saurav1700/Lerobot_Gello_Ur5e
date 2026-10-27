"""Move slowly to a snapshot of GELLO, then wait for Enter to start LeRobot teleop."""
import argparse
import logging
import math
from pathlib import Path

import draccus
from lerobot.scripts.lerobot_teleoperate import TeleoperateConfig, teleop_loop
from lerobot.processor import make_default_processors
from lerobot.utils.import_utils import register_third_party_plugins
from lerobot_robot_gello_ur5e import UR5E, UR5EConfig
from lerobot_teleoperator_gello import Gello, GelloConfig

from move_ur5e_to_gello_pose import move_to_target, show_pose


def align_and_wait(leader, robot, timeout):
    # The caller uses synchronous reads throughout startup: no stale cache or
    # background thread competing with RTDE initialization or the Enter prompt.
    snapshot = leader.get_action()
    target = [snapshot[f'joint_{i}'] for i in range(6)]
    show_pose('GELLO target', target)
    print('Moving UR5e to this captured pose. Hold GELLO still. Ctrl+C requests stop.', flush=True)
    move_to_target(robot.rtde_ctrl, robot.rtde_rec,
                   robot.config.start_move_speed, robot.config.start_move_acc,
                   timeout, target_q=target)
    while True:
        input('UR5e positioned. Press ENTER to start following GELLO (Ctrl+C cancels): ')
        fresh = leader.get_action()
        actual = robot._joint_vector(robot.rtde_rec.getActualQ())
        errors = [abs(fresh[f'joint_{i}'] - actual[i]) for i in range(6)]
        if max(errors) <= robot.config.max_start_error_rad:
            print('Aligned. Starting live teleoperation, including the gripper if enabled.', flush=True)
            return
        print('GELLO moved away from the robot. Return it to the captured pose and press Enter again.')
        print('Joint errors (degrees): ' + ' '.join(f'{math.degrees(v):.2f}' for v in errors))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=Path(__file__).resolve().parents[1] / 'configs/gello_ur5e_legacy.yaml')
    parser.add_argument('--fps', type=int, default=30)
    parser.add_argument('--timeout', type=float, default=180.0, help='Startup move timeout in seconds')
    args = parser.parse_args()
    if args.fps <= 0 or not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error('fps and timeout must be positive and finite')
    register_third_party_plugins()
    cfg = draccus.parse(TeleoperateConfig, config_path=str(args.config), args=[])
    if not isinstance(cfg.robot, UR5EConfig) or not isinstance(cfg.teleop, GelloConfig):
        parser.error('This launcher requires the GELLO and UR5e plugins')
    if cfg.robot.use_gripper != cfg.teleop.use_gripper:
        parser.error('Leader and follower use_gripper settings must match')
    if cfg.display_data:
        parser.error('This startup launcher currently requires display_data=false')
    # This launcher owns startup positioning and captures the live leader pose.
    cfg.robot.align_to_first_action = False
    cfg.teleop.use_async = False
    cfg.robot.servoj_t = 1.0 / args.fps
    leader = Gello(cfg.teleop)
    robot = UR5E(cfg.robot)
    processors = make_default_processors()
    print('Startup will move the arm to GELLO automatically at '
          f'{cfg.robot.start_move_speed} rad/s. Check the motion path before launching.', flush=True)
    try:
        leader.connect()
        robot.connect()
        align_and_wait(leader, robot, args.timeout)
        teleop_loop(
            teleop=leader, robot=robot, fps=args.fps,
            teleop_action_processor=processors[0],
            robot_action_processor=processors[1],
            robot_observation_processor=processors[2],
            duration=cfg.teleop_time_s,
        )
    finally:
        # Stop the follower first; a leader cleanup error must not leave it active.
        for device in (robot, leader):
            try:
                device.disconnect()
            except Exception:
                logging.exception('Failed to cleanly disconnect %s', type(device).__name__)


if __name__ == '__main__':
    try:
        main()
    except (KeyboardInterrupt, EOFError):
        print('\nStopped before or during teleoperation.')
