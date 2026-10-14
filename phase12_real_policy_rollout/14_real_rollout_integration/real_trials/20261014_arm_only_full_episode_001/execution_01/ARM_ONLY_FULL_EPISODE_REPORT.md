# Approved episode launch: preflight software error

The approved launch stopped before episode start with `must be real number,
not str`. Named JointState reorder returns a dictionary; the start-pose
comparison incorrectly iterated its string keys instead of joint values.

Original execution summary is preserved as NOT_EXECUTED. Predictions0,
MoveLine0, gripper0, Stop0. No task outcome or system-integration PASS/FAIL
is assigned to an episode that did not start.

The defect was corrected to index joint_1 through joint_6 explicitly and reject
missing joints. A regression test uses reversed joint-name order. Integration
suite118 tests PASS after correction. Safety thresholds and gripper blocking
were not changed.

No automatic retry, recovery, Home or second launch was executed. A new fresh
approval is required for another attempt; this failed preflight directory is
retained and will not be overwritten.
