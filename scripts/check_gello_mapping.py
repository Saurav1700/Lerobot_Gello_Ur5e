"""Print GELLO angles only. Never constructs or connects a UR5e interface."""
import argparse
import math
from pathlib import Path
import time

import draccus
import yaml
from lerobot_teleoperator_gello import Gello, GelloConfig


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=Path(__file__).resolve().parents[1] / 'configs/gello_ur5e_legacy.yaml')
    parser.add_argument('--check-config', action='store_true', help='Validate config and exit without opening any device')
    parser.add_argument('--reference-deg', type=float, nargs=6, metavar='DEG', help='Optional six pendant joint angles in degrees, in joint order')
    args = parser.parse_args()
    data = yaml.safe_load(args.config.read_text())['teleop'].copy()
    if data.pop('type', None) != 'gello_ur5e':
        parser.error('Expected teleop.type=gello_ur5e')
    config = draccus.decode(GelloConfig, data)
    if config.joint_offsets_rad is None:
        parser.error('This check requires fixed offsets')
    if args.reference_deg and not all(math.isfinite(v) for v in args.reference_deg):
        parser.error('Reference angles must be finite')
    # Read fresh samples directly, so smoothing cannot hide jumps during inspection.
    config.use_async = False
    print(f'Port: {config.port}')
    print(f'Motor IDs: {config.joint_ids}')
    print(f'Offsets (rad): {config.joint_offsets_rad}; signs: {config.joint_signs}')
    if args.check_config:
        print('Config parsed successfully. No hardware opened.')
        return
    leader = Gello(config)
    print('GELLO only: motors remain passive. No UR5e connection. Ctrl+C to stop.')
    print('Columns: joint_0/base, joint_1/shoulder, joint_2/elbow, joint_3/wrist1, joint_4/wrist2, joint_5/wrist3')
    previous = None
    try:
        leader.connect(calibrate=False)
        while True:
            action = leader.get_action()
            q = [action[name] for name in leader.JOINT_NAMES]
            degrees = [math.degrees(v) for v in q]
            print('rad: ' + ' '.join(f'{v:9.4f}' for v in q)
                  + ' | deg: ' + ' '.join(f'{v:9.2f}' for v in degrees), flush=True)
            if config.use_gripper:
                print(f"gripper: {action['gripper']:.3f} (0=open, 1=closed)", flush=True)
            if args.reference_deg:
                errors = [a-b for a, b in zip(degrees, args.reference_deg)]
                print('error vs pendant (deg): ' + ' '.join(f'{v:+9.2f}' for v in errors), flush=True)
            if previous is not None and any(abs(a-b) > math.pi for a, b in zip(q, previous)):
                print('Large change (>180 degrees) between samples; check encoder wrap/reference.', flush=True)
            previous = q
            time.sleep(0.2)
    except KeyboardInterrupt:
        print('\nStopped.')
    finally:
        leader.disconnect()


if __name__ == '__main__':
    main()
