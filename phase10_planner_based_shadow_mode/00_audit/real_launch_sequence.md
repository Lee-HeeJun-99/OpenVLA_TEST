# Real launch sequence audit

`runtime.launch.py` loads a selected YAML and starts, in declaration order, camera adapter, chosen inference executable, action adapter, Doosan bridge, and episode manager. It is a complete closed-loop graph and is prohibited for this remote preparation.

`dataset_replay.launch.py` starts `doosan_bridge` as well as `dataset_episode_replay`. Its default replay `dry_run=true` does not make the launch safe under this protocol because the bridge still constructs robot service clients and stream publishers. Do not run it.

`steamvr_teleop.launch.py`, `servol_stream_test`, and package shell launch helpers are also prohibited. No launch command is required for the completed offline tests.

Draft local sequence after explicit approval is intentionally non-executable here: verify hardware E-stop and controller revision; verify ROS graph read-only; verify clocks and timestamped observation topics; run a subscriber-only logger check; conduct a planner-only Real dry run under a local operator; inspect/validate the recorded episode; replay that file into the two offline model adapters. Robot connection, graph verification, planner run, and Shadow collection remain blocked in this phase.
