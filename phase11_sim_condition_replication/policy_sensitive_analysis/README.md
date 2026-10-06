# Phase 11 paired-condition policy-sensitive analysis

Current status: `BLOCKED_MODEL_SERVER`. Data integrity, phase alignment and observation metrics are complete. Actual policy predictions and hidden features have not been generated. See `final_report/final_report.md`.

The analysis directory contains no ROS node, robot publisher, motion service, Home, gripper or closed-loop path. `scripts/run_prediction_only.sh` calls only the Phase 10 recorded-input HTTP runner and must be used only after prediction-only servers are available.
