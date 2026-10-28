# Full-policy live trial 003

Result INVALID, reason camera_stream_loss. Three fresh predictions and three
MoveLine commands sent; all three arm service ACKs successful. Only two complete
arm/settle/gripper stages finished. The third arm ACK succeeded, but subsequent
feedback/settle/gripper-boundary checks encountered the camera watchdog fault.
Do not label the third synchronous command ABORTED_IN_FLIGHT or claim that
its full control step completed.

All three gripper raw predictions0.00196078 (OPEN-like), so no inference-driven
gripper transition was requested: initialized OPEN state duplicates suppressed.
Physical gripper writes2 were fault cleanup DO1=0 and DO2=0, not model OPEN/CLOSE.
Arm HOLD(3)1, successful ACK. Both-output OFF cleanup succeeded on first pair
attempt with success=true/value0 readback for both channels.

Runtime valid=false, task_success=null, excluded from task SUCCESS/FAILURE
denominator. No auto retry, Home, recovery, next episode or batch. Prior timeout
attempts remain intact. Fresh operator-confirmed existing OPEN initialization
was explicitly reused, not automatically repeated or promoted to formal TCP
or physical jaw failsafe verification.

Actual unexpected motion and E-stop observations still require operator input.
No additional physical action was performed after this trial ended.
