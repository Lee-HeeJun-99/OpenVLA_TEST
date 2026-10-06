# Hardware safety interface and operator checklist

No hardware safety state was inferable from the observed graph. Code/static search and the live graph found no verified hardware E-stop state, protective-stop state, servo state, robot mode, controller alarm, Hold acknowledgement, motion result, or measured gripper feedback. All are `NOT_FOUND` and block motion readiness.

| Local operator item | Status |
|---|---|
| Robot area clear | unconfirmed |
| Physical E-stop location and immediate reach | unconfirmed |
| E-stop verified to block controller motion | unconfirmed (not tested) |
| Servo-on state | unconfirmed |
| Manual/auto mode | unconfirmed |
| Robot Home joint | unconfirmed |
| Gripper Home/open state | unconfirmed |
| Approved physical workspace | unconfirmed |
| Maximum joint/TCP velocity | unconfirmed |
| Cube/obstacle positions | unconfirmed |
| Camera rigidly fixed | unconfirmed |

The E-stop was not called or tested. A local operator must fill this checklist at the robot; software may not self-approve it.
