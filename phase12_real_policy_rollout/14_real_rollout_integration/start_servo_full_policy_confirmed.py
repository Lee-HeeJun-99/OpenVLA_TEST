"""Explicit gripper-enabled ServoL entry; legacy arm-only entry is unchanged."""
import signal
from start_full_policy_confirmed import main

if __name__ == '__main__':
    def terminate(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, terminate)
    raise SystemExit(main(controller='servo'))
