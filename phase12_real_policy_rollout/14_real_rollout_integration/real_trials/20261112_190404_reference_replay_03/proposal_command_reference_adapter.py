"""Offline reference adapter proposal; no ROS or transport construction."""
from servo_experimental.reference_replay import delta_target, install_reference

def update(tracker, limited_action, actual_pose, source, now, boundary=None):
    anchor, target = install_reference(tracker, limited_action, actual_pose, "command")
    tracker.set_target(target, source, now)
    if boundary is not None:
        boundary.begin_segment(source, limited_action)
    return anchor, target

def tick(tracker, now, feedback, feedback_received, boundary=None):
    row = tracker.tick(now)
    if row is not None and boundary is not None:
        # Replay provides fake publish/stop callbacks; never a live transport.
        boundary.inspect_and_publish(row['pos'], now, feedback, feedback_received)
    return row
