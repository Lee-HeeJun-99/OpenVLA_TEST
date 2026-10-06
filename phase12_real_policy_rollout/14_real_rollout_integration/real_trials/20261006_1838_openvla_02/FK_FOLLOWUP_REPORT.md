# Current FK/input gate follow-up

Subscriber-only 60.007s acquisition, 5261 messages, first receive delay7.416s, steady receive100.016Hz. Position/velocity/name validation failures0. FK unit tests4/4PASS. No getter, publish, motion, gripper or stop call was made.

Important distinction: the full-window source jump3.059959s occurs ONLY at sample0→1. Receive interval there is0.001599s, 52.590s before recording end. Thus this event is not a demonstrated 3-second live reception stall. Initial older-header delivery is a candidate explanation, not proven middleware root cause. Raw first sample is retained, never silently removed.

Latest30s window:3000 messages; max source gap0.010939s; max receive gap0.012790s. Latest receive age0.000227s at acquisition end. Recent continuity PASS for that observed window only; full-window script conservatively reports FAIL because of initial header discontinuity. This window does not automatically authorize future motion or imply that earlier trials were normal. Freshness must be continuously checked again immediately before any motion.

Current computed link_6/base_link position: [304.607750, -11.262126, 724.503762]mm. Source FK_ESTIMATED flange; zero tool offset has NOT been applied as a current verified TCP contract. Raw matrix/quaternion and joints are in fk_live_validation.json. All computed values finite. URDF SHA256fc1541ff6acb04e503174c6817c0b0fb721a342a0a7fa85e55be5cf5297a0742.

TCP gate BLOCKED_PENDING_CURRENT_OPERATOR_TOOL_AND_POSE_CROSSCHECK. Historical Tool_v1 zero offset and pose from2026-10-02 cannot establish the current controller tool or current pose. No measured TCP topic was found previously and unstable getter remains unused.

Next requires operator's current pendant TCP name/offset/pose/reference frame/time and physically stationary confirmation. Do not change mode/tool, jog, stop driver or call getter merely to fill this form. If pendant inspection needs launch shutdown, coordinate first because it invalidates current live readiness.

OpenVLA GPU health remains previously PASS; no integrated live Shadow/minimum/short/full stage was executed in this follow-up. All physical command counts0. Command gate disabled; no authorization artifact created.
