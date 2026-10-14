# Arm-only episode execution 02

Result **ARM_ONLY_FULL_EPISODE_INVALID**, reason
`robot_not_settled_before_dispatch`. Eight inference attempts, seven completed
arm MoveLine steps, duration10.819s including cleanup. The eighth action was
not dispatched: a fresh JointState reported nonzero joint velocity before
dispatch. Existing arm HOLD was sent once, acknowledged successfully. No
gripper, gripper-abort IO, Home, recovery, automatic retry or next episode.
All seven arm service returns were successful. No extra arm command after abort.

Recorded seven executed-step predictions: raw translation maximum11.054mm,
limited/delivered translation maximum4.000mm; raw and delivered rotation maximum
0.813degrees. These are maxima of the saved seven records, not a fabricated
complete eight-prediction dataset. The failed pre-dispatch observation and
eighth raw response were not separately persisted by this launcher; complete
JointState trace is retained in jointstate.json.

JointState RUNTIME_READY, no runtime fault; max runtime source12.847ms,
receive16.717ms, >=100ms events0. Saved seven camera-start ages max28.882ms.
Final recorded joint velocities allzero. These show sensor freshness remained
normal; the termination was the strict pre-dispatch stationary gate, not the
previous DDS gap issue. Nonzero velocity could indicate residual motion or
small feedback variation; do not assert its cause or weaken the gate without
examining the trace.

TCP formal status stays supported-not-verified; user-approved engineering
execution basis only. Gripper abort behavior remains UNVERIFIED/UNUSED.
Task SUCCESS/FAILURE NOT_APPLICABLE and excluded from policy success rate.

Operator unexpected-motion and E-stop result remain pending actual confirmation.
No PASS is inferred solely from command ACKs. Preserve execution01 preflight
software failure and execution02 INVALID; no failed artifact overwritten.
