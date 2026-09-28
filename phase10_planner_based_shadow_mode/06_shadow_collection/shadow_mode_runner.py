#!/usr/bin/env python3
"""Recorded-input Shadow Mode runner.

This executable has no ROS imports and no publisher.  It reads an immutable
episode, runs prediction-only adapters, and writes integrated logs.  It cannot
move a real robot.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import time
from typing import Any


PHASE10 = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PHASE10 / "03_logger"))
sys.path.insert(0, str(PHASE10 / "04_planner_reference"))
sys.path.insert(0, str(PHASE10 / "05_model_adapters"))
sys.path.insert(0, str(PHASE10 / "02_safety"))

from action_canonicalizer import canonicalize_action  # noqa: E402
from integrated_logger import IntegratedLogger  # noqa: E402
from oft_adapter import OFTAdapter  # noqa: E402
from openvla_adapter import OpenVLAAdapter  # noqa: E402
from planner_interface import RecordedPlannerEpisode  # noqa: E402
from safety_gate import ShadowCommandGate  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--episode-dir", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--trial-id", required=True)
    parser.add_argument("--condition-id", default="baseline")
    parser.add_argument("--layout-id", default="unknown_layout")
    parser.add_argument("--openvla-server-url")
    parser.add_argument("--oft-server-url")
    parser.add_argument("--openvla-fixture", type=Path, help="Smoke-test only; never research output")
    parser.add_argument("--oft-fixture", type=Path, help="Smoke-test only; never research output")
    parser.add_argument("--max-steps", type=int, default=0)
    return parser.parse_args()


def fixture_predictor(path: Path):
    payload = json.loads(path.read_text(encoding="utf-8"))
    return lambda _request: payload


def adapter(server: str | None, fixture: Path | None, cls):
    if fixture is not None:
        return cls(predictor=fixture_predictor(fixture))
    if server is not None:
        return cls(server_url=server)
    return None


def resolve_image(episode_dir: Path, image_field: str) -> Path:
    candidate = episode_dir / image_field
    if candidate.exists():
        return candidate
    candidate = episode_dir / "images" / "primary" / Path(image_field).name
    if candidate.exists():
        return candidate
    raise FileNotFoundError(image_field)


def main() -> int:
    args = parse_args()
    episode = RecordedPlannerEpisode(args.episode_dir)
    openvla = adapter(args.openvla_server_url, args.openvla_fixture, OpenVLAAdapter)
    oft = adapter(args.oft_server_url, args.oft_fixture, OFTAdapter)
    ShadowCommandGate(shadow_mode=True)  # construction is an explicit hard gate

    steps = list(episode.iter_steps())
    if args.max_steps > 0:
        steps = steps[: args.max_steps]
    instruction = episode.metadata.get("instruction", "Pick up the orange cube.")
    fixture_mode = args.openvla_fixture is not None or args.oft_fixture is not None

    with IntegratedLogger(args.output_dir) as logger:
        for local_index, step in enumerate(steps):
            image_path = resolve_image(episode.episode_dir, str(step["image"]))
            planner_raw = step.get("action")
            planner_canonical = None
            exclusion: list[str] = []
            try:
                planner_canonical = canonicalize_action(
                    planner_raw,
                    source="planner_recorded_action",
                    time_horizon_seconds=0.2,
                ).to_dict()
            except Exception as exc:
                exclusion.append(f"planner_action_invalid:{exc}")

            openvla_result = None
            oft_result = None
            inference_errors: dict[str, str] = {}
            if openvla is not None:
                try:
                    openvla_result = openvla.predict_recorded(image_path, instruction)
                except Exception as exc:
                    inference_errors["openvla"] = repr(exc)
            # OFT is evaluated at 1 Hz against the 5 Hz recorded control stream.
            if oft is not None and local_index % 5 == 0:
                try:
                    oft_result = oft.predict_recorded(image_path, instruction)
                except Exception as exc:
                    inference_errors["oft"] = repr(exc)

            if inference_errors:
                exclusion.extend(f"{name}_inference_failure" for name in sorted(inference_errors))
            record = {
                "trial_id": args.trial_id,
                "condition_id": args.condition_id,
                "layout_id": args.layout_id,
                "episode_id": episode.metadata.get("episode_id", episode.episode_dir.name),
                "frame_id": int(step.get("step_index", local_index)),
                "planner_step": int(step.get("step_index", local_index)),
                "timestamp_camera": step.get("image_timestamp"),
                "timestamp_state": step.get("source_timestamp"),
                "timestamp_planner": step.get("timestamp"),
                "camera_age_sec_at_record": step.get("image_age_sec"),
                "state_age_sec_at_record": step.get("pose_age_sec"),
                "camera_clock_domain": "unknown_source_clock",
                "state_clock_domain": "unknown_source_clock",
                "planner_clock_domain": "episode_relative",
                "timestamp_openvla": time.time() if openvla_result else None,
                "timestamp_oft": time.time() if oft_result else None,
                "raw_image_path": str(image_path),
                "raw_image_sha256": logger.image_sha256(image_path),
                "model_input_image_path": str(image_path),
                "instruction": instruction,
                "raw_joint_position": step.get("joint_position_rad"),
                "raw_joint_velocity": step.get("joint_velocity_rad_s"),
                "raw_ee_position": (step.get("end_effector_pose") or {}).get("position_m"),
                "raw_ee_rotation": (step.get("end_effector_pose") or {}).get("quaternion_wxyz"),
                "raw_gripper_state": step.get("gripper_closedness"),
                "pose_source": step.get("pose_source", episode.pose_source),
                "planner_target": None,
                "planner_raw_action": planner_raw,
                "planner_canonical_action": planner_canonical,
                "planner_executed_action": planner_raw,
                "robot_delivered_command": None,
                "measured_post_state": None,
                "openvla_raw_action": None if openvla_result is None else openvla_result["raw_model_output"],
                "openvla_denormalized_action": None if openvla_result is None else openvla_result["denormalized_action"],
                "openvla_canonical_action": None if openvla_result is None else openvla_result["canonical_action"],
                "openvla_executed_action": None,
                "oft_raw_action_chunk": None if oft_result is None else oft_result["raw_model_output"],
                "oft_denormalized_action_chunk": None if oft_result is None else oft_result["denormalized_action_chunk"],
                "oft_canonical_action_chunk": None if oft_result is None else oft_result["canonical_action_chunk"],
                "oft_executed_action": None,
                "gripper_postprocessed_action": None,
                "inference_latency_openvla": None if openvla_result is None else openvla_result["inference_latency_seconds"],
                "inference_latency_oft": None if oft_result is None else oft_result["inference_latency_seconds"],
                "timeout_flags": {},
                "communication_flags": {},
                "hold_flags": {},
                "emergency_stop_flags": {},
                "phase": step.get("planner_phase", "unknown"),
                "valid": planner_canonical is not None and not inference_errors,
                "exclusion_reason": ";".join(exclusion) or None,
                "inference_errors": inference_errors,
                "collection_mode": "recorded-input shadow",
                "fixture_smoke_test_only": fixture_mode,
            }
            logger.append(record)
    print(json.dumps({
        "status": "COMPLETED_OFFLINE_ONLY",
        "mode": "recorded-input shadow",
        "records": len(steps),
        "robot_commands_published": 0,
        "output_dir": str(args.output_dir),
        "fixture_smoke_test_only": fixture_mode,
    }, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
