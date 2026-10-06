# Minimum-motion precheck — physical stage blocked

Current HEAD at start01c4fe5945e3d323e1016c676e72501ccfdcba45. Driver was not restarted; no mode/tool/jog/getter call made. Production safety limits unchanged.

Camera freshness: FAIL; actual prediction count 62, camera_failure 1, timing {'health_latency_s': {'median': 0.0030262419022619724, 'max': 0.009860979858785868, 'p95': 0.008535347878932953}, 'jpeg_encode_latency_s': {'median': 0.009227097500115633, 'max': 0.03605697909370065, 'p95': 0.021930024959146976}, 'selected_frame_age_at_snapshot_s': {'median': 0.003968329867348075, 'max': 0.009991202037781477, 'p95': 0.009084189776331186}, 'inference_latency_s': {'median': 0.41563728637993336, 'max': 0.5181175959296525, 'p95': 0.4505759971216321}, 'camera_age_s': {'median': 0.4357333369553089, 'max': 0.5314244902692735, 'p95': 0.47405480686575174}}. Health validation now precedes frame selection; fresh receive-age<=10ms frame required, raw PIL RGB decode is pixel-equivalent including ROS padding/alpha. Inference input size/crop/normalization/JPEGquality95 unchanged. Safety's0.5s threshold not modified. FK/joints used only as unverified diagnostic context; safety context refreshed after inference.

First attempt (precheck01) health-before-snapshot alone still failed10/61 selected frames; healthmedian2.34ms, RGB/JPEGmedian32.02ms, snapshotmax94.59ms. Preserved, not overwritten. Second attempt additionally waits for a NEWframe and uses tested faster raw decoding. These observations support selected-frame processing/age as contributors, not proven controller issues.

JointState fresh window: {'status': 'PASS', 'count': 1995, 'coverage_s': 19.93857246497646, 'latest_receive_age_s': 0.009549092035740614, 'receive_rate_hz': 100.00715966514677, 'max_source_gap_s': 0.011853694915771484, 'max_receive_gap_s': 0.01584192831069231, 'invalid_values': 0}. Watchdog fault: None. This is a diagnostic pre-Shadow gate, not a continuously valid future motion approval.

TCP remainsUNKNOWN. GetCurrentTcp/GetCurrentTool callbacks return names only; no6D offset/pose. Their current live stability is unverified and they were not called. ConfigCreateTcp contains6Dpos but is a setter and NEVER called. PastTool_v1 was not reused. Current operator pendant name/offset/pose/frame/time and safety mode/authority/E-stop/workspace confirmations are still required.

Gripper stateUNKNOWN isolated: this diagnostic has no physical gripper API, pulse or motion sink. Minimum-motion must remain translation-only with gripper disabled; this does not establish real gripper polarity or full-task safety.

Minimum-motion BLOCKED, exactly0 physical pose commands. No service ACK or actual Cartesian displacement exists to score. API-success-only PASS never issued. Short/full-task and model action motion NOT_EXECUTED. All robot/gripper/Home/trajectory/Hold/Stop/real rollout calls0. No authorization artifact created.

Unit integration37PASS,0FAIL after RGBdecoder/gate changes. Next allowed stage remains currentTCP/operator crosscheck; only after ALLgates freshlyPASS can a separate explicit minimum-motion approval be requested.
