#!/usr/bin/env python3
"""Replay saved stationary predictions through the hard, null-delivery gate."""
from __future__ import annotations
import argparse, json, sys, time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "02_safety"))
from pre_motion_runtime_facade import PreMotionRuntimeFacade

SAFE_CONFIG = {
    "mode": "command_disabled_pre_motion",
    "include_action_adapter": False,
    "include_doosan_bridge": False,
    "allow_robot_command": False,
    "allow_gripper_command": False,
    "allow_home_command": False,
    "allow_trajectory_execution": False,
    "allow_motion_service": False,
    "allow_stop_service": False,
    "allow_estop_service": False,
    "publish_ai_action": False,
}

def load_first(path: Path) -> dict:
    with path.open() as handle:
        return json.loads(next(line for line in handle if line.strip()))

def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    pose = dict(current_position_m=(0.30710, -0.01275, 0.72239),
                current_abc_deg=(179.74, -155.72, 3.30), phase="alignment")
    sources = {
        "openvla": ROOT / "03_shadow_mode/openvla_stationary_shadow_20261002.jsonl",
        "oft": ROOT / "03_shadow_mode/oft_stationary_shadow_20261002.jsonl",
    }
    rows = []
    ov = load_first(sources["openvla"])
    runtime = PreMotionRuntimeFacade("openvla", operator_confirmed_initial_open=True)
    candidate = runtime.inspect_openvla(ov["canonical_predictions"][0], action_id="saved-openvla-k0",
        source_monotonic=10.0, now_monotonic=10.0, **pose)
    rows.append(candidate)
    oft = load_first(sources["oft"])
    runtime = PreMotionRuntimeFacade("oft", operator_confirmed_initial_open=True)
    runtime.enqueue_oft(oft["canonical_predictions"], "saved-oft-chunk0", 20.0)
    for index in range(5):
        candidate = runtime.inspect_next_oft(20.0 + 0.2 * index, **pose)
        rows.append(candidate)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x") as handle:
        for row in rows:
            handle.write(json.dumps(row, separators=(",", ":")) + "\n")
        handle.flush()
        import os
        os.fsync(handle.fileno())
    summary = {
        "classification": "COMMAND_DISABLED_BOUNDARY_DRY_RUN",
        "created_wall_time": time.time(),
        "records": len(rows),
        "technical_valid": sum(bool(r["technical_valid"]) for r in rows),
        "hard_gate_delivery_blocked": sum(bool(r["hard_gate"]["delivery_blocked"]) for r in rows),
        "all_executed_action_null": all(r["executed_action"] is None for r in rows),
        "all_robot_delivered_command_null": all(r["robot_delivered_command"] is None for r in rows),
        "all_command_issued_false": all(r["command_issued"] is False for r in rows),
        "capabilities": runtime.capability_report,
        "robot_command_count": 0,
        "motion_service_action_count": 0,
        "gripper_command_count": 0,
    }
    args.output.with_suffix(args.output.suffix + ".status.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary, indent=2))
    return 0
if __name__ == "__main__":
    raise SystemExit(main())
