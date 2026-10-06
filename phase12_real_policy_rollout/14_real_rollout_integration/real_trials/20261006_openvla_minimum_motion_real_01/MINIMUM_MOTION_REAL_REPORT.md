# Minimum-motion real precheck — blocked, no physical stage

Start HEAD ebb1665d20bff17846567f12b2afb4c9cf5a618d, branch lhj-research, initially clean. No driver restart, mode/tool change or getter call.

Camera recorded 56 real GPU predictions in 27.437s. camera_failure=0, observed maximum selected age 0.493229s (<0.5). However JointState watchdog fault stopped the run before the mandatory30s; camera final-validation FAIL/INCOMPLETE, not promoted to motion approval. Existing fast BGRA→RGB/JPEG and health-before-new-frame path unchanged. Original logs preserved.

JointState pre20s gate PASS (2000 samples99.998Hz,maxsource11.42ms/maxreceive16.15ms). During Shadow, recent continuity failed: captured late source gaps0.1900s and0.1300s and receive gaps0.1912s/0.1320s. Initial3.06s events are separately retained, not confused with this late recent-window failure. Independent watchdog stopped predictions; no physical Hold was invoked because diagnostic sink has no command capability.

TCP automatic inventory: actual /dsr01/robot_description robot=a0509 has base_link,link_1…link_6,world and only world_fixed static joint. No Doosan flange→tool/TCP transform verified. Source a0509.urdf tool0 block is commented out. /doosan/current_pose has no publisher. left_TCP/right_TCP and so101 gripper frames are OTHER robot/scene frames; never reused as A0509 controller TCP. TF source/header/receive times, publisher endpoints, all frame transforms and topic/service types saved in tcp_source_audit.json. TF alone cannot prove controller active tool. Historical Tool_v1/offset0 is not current evidence. Current classification CURRENT_TCP_UNKNOWN.

Hardware/operator confirmation not received: workspace clear,local operator,E-stop access,protective stop,mode/authority UNKNOWN. These were not inferred from the user's request. Gripper physical commands/pulses DISABLED. No setter or query of unstable service performed.

Final BLOCKED_TCP_CONTRACT_UNKNOWN with additional JointState/hardware/camera-completion blocks. Minimum motion NOT_EXECUTED; requested target/delta=null (protocol reference only +X0.5mm), observed delta=null, ACK=null, physical direction/displacement not assessed. No API-success-only PASS artifact created. Short-horizon/full-task NOT_EXECUTED. All physical robot/motion/gripper/Home/trajectory/Hold/Stop/real-rollout calls0. Safety limits unchanged.

Required next evidence: current active TCP/tool name, current flange→TCP6D offset with units/convention/reference, pendant current pose/base frame/time/stationary confirmation, plus current local safety confirmations. Read-only fresh validation must pass again before any separate minimum-motion approval. No software auto-recovery or root-cause expansion undertaken.
