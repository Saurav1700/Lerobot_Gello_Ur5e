"""Move the UR5e once to the supplied GELLO pose; no gripper or leader access."""
import argparse
import math
import time

TARGET_Q = [-0.0403, -1.5451, 1.6755, -1.7936, -1.5129, 0.0134]


def show_pose(label, q):
    print(f'{label} rad: ' + ' '.join(f'{v:9.4f}' for v in q))
    print(f'{label} deg: ' + ' '.join(f'{math.degrees(v):9.2f}' for v in q))



def move_to_target(control, receive, speed, acceleration, timeout, target_q=None):
    target_q = list(TARGET_Q if target_q is None else target_q)
    if len(target_q) != 6 or not all(math.isfinite(v) for v in target_q):
        raise ValueError('Target must contain six finite joint angles')
    if not control.isJointsWithinSafetyLimits(target_q):
        raise RuntimeError('Controller rejected target as outside its safety limits')
    previous = control.getAsyncOperationProgress()
    try:
        if not control.moveJ(target_q, speed, acceleration, True):
            raise RuntimeError('Controller rejected moveJ')
        deadline = time.monotonic() + timeout
        started = False
        while True:
            if not control.isConnected() or not receive.isConnected():
                raise ConnectionError('Lost RTDE connection during movement')
            if receive.isProtectiveStopped() or receive.isEmergencyStopped():
                raise RuntimeError('Robot stopped; check the teach pendant')
            progress = control.getAsyncOperationProgress()
            started = started or progress >= 0
            if progress < 0 and (started or progress != previous):
                break
            if time.monotonic() >= deadline:
                raise TimeoutError('Move did not complete within the timeout')
            time.sleep(0.05)
        actual = receive.getActualQ()
        if len(actual) != 6 or any(not math.isfinite(v) for v in actual):
            raise RuntimeError('Invalid final joint measurement')
        show_pose('Final ', actual)
        error = max(abs(a-b) for a, b in zip(actual, target_q))
        if error > 0.01:
            raise RuntimeError(f'Movement ended but target error is {error:.4f} rad')
        print(f'Target reached; maximum joint error {error:.4f} rad.')
    except BaseException:
        try:
            control.stopJ(0.5)
        except Exception as exc:
            print(f'Could not send stopJ: {exc}. Use the pendant to stop the robot.')
        raise


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ip', default='195.148.65.50')
    parser.add_argument('--execute', action='store_true', help='Execute the move; otherwise only read current joints')
    parser.add_argument('--speed', type=float, default=0.05, help='Leading joint speed in rad/s (default 0.05)')
    parser.add_argument('--acceleration', type=float, default=0.05, help='Joint acceleration in rad/s²')
    parser.add_argument('--timeout', type=float, default=180.0)
    args = parser.parse_args()
    for name in ('speed', 'acceleration', 'timeout'):
        if not math.isfinite(getattr(args, name)) or getattr(args, name) <= 0:
            parser.error(f'{name} must be finite and positive')

    import rtde_receive
    receive = rtde_receive.RTDEReceiveInterface(args.ip)
    control = None
    try:
        if not receive.isConnected():
            raise ConnectionError('Could not connect to UR5e receive interface')
        print('Order: base, shoulder, elbow, wrist 1, wrist 2, wrist 3')
        actual = receive.getActualQ()
        if len(actual) != 6 or any(not math.isfinite(v) for v in actual):
            raise RuntimeError('Invalid current joint measurement')
        show_pose('Current', actual)
        show_pose('Target ', TARGET_Q)
        show_pose('Change ', [b-a for a, b in zip(actual, TARGET_Q)])
        if not args.execute:
            print('Preview only. Add --execute to move after checking the entire motion path.')
            return
        if receive.isProtectiveStopped() or receive.isEmergencyStopped():
            raise RuntimeError('Robot is stopped; check the teach pendant')
        import rtde_control
        control = rtde_control.RTDEControlInterface(args.ip)
        print(f'Moving at {args.speed} rad/s. Ctrl+C requests a controlled stop.')
        move_to_target(control, receive, args.speed, args.acceleration, args.timeout)
    finally:
        if control is not None:
            try:
                control.stopScript()
            finally:
                try:
                    control.disconnect()
                finally:
                    receive.disconnect()
        else:
            receive.disconnect()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('\nInterrupted; stop requested.')
