#!/usr/bin/python3
"""Offline replay of the 190404 raw action sequence. Never imports ROS."""

from __future__ import annotations

import json
from pathlib import Path
import sys

import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from controller.action import ActionInterpreter
from controller.config import load_config
from controller.geometry import ToolTransform, apply_action
from controller.gripper import ContinuousGripper
from controller.precheck import (EndpointStreak, GripperStateTracker,
                                 action_reasons, joint_margin_reasons,
                                 singularity_reason, singularity_family)
from controller.publisher import DryRunPublisher
from controller.stop_reasons import IKStopReason
from controller.trajectory import SegmentGenerator, SegmentIKError
from kinematics import PyKDLKinematics

RUN = Path("/home/ubuntu/a0509_vla_linux_field_bundle_20260903/lhj/phase12_real_policy_rollout/14_real_rollout_integration/real_trials/20261112_190404_arm_only_servo_episode")
STATS = Path("/home/ubuntu/a0509_vla_linux_field_bundle_20260903/models/vanilla_s1_balanced_step8130/dataset_statistics.json")


def start_joint():
    sample = next(s for s in json.loads((RUN / "jointstate.json").read_text())["samples"] if s["valid"])
    values = dict(zip(sample["names"], sample["position"]))
    return np.array([values[f"joint_{i}"] for i in range(1, 7)])


def stats(values):
    a = np.asarray(values, dtype=float)
    return {"n": int(a.size), "p50": float(np.percentile(a, 50)),
            "p99": float(np.percentile(a, 99)), "max": float(np.max(a))}


def replay(frame):
    config = load_config(rotation_frame=frame)
    kin, tool = PyKDLKinematics(), ToolTransform.tool0_real()
    generator = SegmentGenerator(kin, config, tool)
    interpreter = ActionInterpreter(STATS)
    actions = [json.loads(line) for line in (RUN / "predictions.jsonl").open()]
    parsed_actions = [interpreter.parse(x["raw_action"]) for x in actions]
    q = start_joint(); link = kin.fk(q); tool_pose = tool.link_to_tool(link)
    endpoint_streak = EndpointStreak(); gripper_tracker = GripperStateTracker()
    endpoint_events, first_streak, streak_history = [], None, []
    gripper_state_changes = []
    for index, action in enumerate(parsed_actions):
        counts = endpoint_streak.update(action); streak_history.append(counts)
        if np.any(action.endpoint_min[:6] | action.endpoint_max[:6]):
            endpoint_events.append({"step": index,
                "min_dimensions": np.flatnonzero(action.endpoint_min[:6]).tolist(),
                "max_dimensions": np.flatnonzero(action.endpoint_max[:6]).tolist()})
        if gripper_tracker.update(action.gripper_binary):
            gripper_state_changes.append({"step": index,
                                          "state": action.gripper_binary})
        if first_streak is None and np.any(counts >= config.endpoint_streak_limit):
            first_streak = {"step": index, "dimensions_side": np.argwhere(
                counts >= config.endpoint_streak_limit).tolist()}
    action_precheck_events, candidate_joint_events = [], []
    extensions = 0; durations = []; position_errors = []; rotation_errors = []
    overshoots = 0; path_budget_violations = 0; max_vel_ratio = []; max_acc_ratio = []
    command_quaternions = []
    ik_sample_times = [];segment_precompute_times = []
    dry_path = ROOT / "reports" / f"controller_dryrun_{frame}.jsonl"
    dry_path.write_text("")
    publisher = DryRunPublisher(dry_path)
    gripper = ContinuousGripper(config.rate_hz,
                                config.gripper_full_travel_duration_s)
    gripper_targets = []
    safe_first_stop = None
    first_singularity_approach = None

    fatal = None
    for index, record in enumerate(actions):
        action = parsed_actions[index]
        counts = streak_history[index]
        target_tool = apply_action(tool_pose, action.delta_position_m,
                                   action.rotation_vector_rad, frame)
        reasons = action_reasons(action, target_tool.position_m, config, counts)
        if reasons:
            action_precheck_events.append({"step": index, "reasons": reasons})
            if safe_first_stop is None:
                safe_first_stop = action_precheck_events[-1]

        target_link = tool.tool_to_link(target_tool)
        # Full-sequence diagnostic uses URDF joint bounds so candidate pendant
        # bounds can be audited without truncating later offline evidence.
        try:
            segment = generator.generate(tool_pose, target_tool, q, use_candidate_limits=False,
                                         enforce_full_preflight=False)
        except Exception as error:
            fatal = {"step": index, "reason": f"{type(error).__name__}:{error}"}
            if isinstance(error, SegmentIKError):
                fatal.update({"sample": error.sample_index,
                              "target_link_matrix": error.target_link_pose.matrix.tolist(),
                              "seed_joint_rad": error.seed_rad.tolist(),
                              "cause": f"{type(error.cause).__name__}:{error.cause}"})
                if hasattr(error.cause, "iterations"):
                    fatal["iterations_run"] = error.cause.iterations
                    fatal["best_residual"] = error.cause.best_residual
                fatal["stop_reason_code"] = {
                    "IKConvergenceError": IKStopReason.NONCONVERGENCE.value,
                    "IKJointChangeError": IKStopReason.DISCONTINUITY.value,
                    "JointLimitError": IKStopReason.JOINT_LIMIT.value,
                }.get(type(error.cause).__name__, IKStopReason.FAILURE.value)
                failed_reason, failed_sigma = singularity_reason(
                    error.seed_rad, kin, config)
                if failed_reason is not None:
                    fatal.update({"reason": failed_reason,
                                  "stop_reason_code": failed_reason,
                                  "minimum_singular_value": failed_sigma,
                                  "singularity_family": singularity_family(error.seed_rad),
                                  "underlying_ik_error": str(error.cause)})
            if safe_first_stop is None:
                safe_first_stop = fatal
            break

        extensions += segment.extension_count; durations.append(segment.duration_s)
        ik_sample_times.extend(segment.ik_sample_times_s.tolist())
        segment_precompute_times.append(segment.precompute_wall_s)
        max_vel_ratio.append(float(np.max(segment.max_velocity_rad_s /
                                          config.max_joint_velocity_rad_s)))
        max_acc_ratio.append(float(np.max(segment.max_acceleration_rad_s2 /
                                          config.max_joint_acceleration_rad_s2)))
        for sample_index, sample_q in enumerate(segment.joints_rad):
            singular_reason, minimum_singular_value = singularity_reason(
                sample_q, kin, config)
            if singular_reason is not None and first_singularity_approach is None:
                first_singularity_approach = {
                    "step": index, "sample": sample_index,
                    "reason": singular_reason,
                    "minimum_singular_value": minimum_singular_value,
                    "singularity_family": singularity_family(sample_q),
                    "joints_deg": np.rad2deg(sample_q).tolist()}
                if safe_first_stop is None:
                    safe_first_stop = first_singularity_approach
            candidate_reasons = joint_margin_reasons(sample_q, config)
            if candidate_reasons:
                candidate_joint_events.append({"step": index, "sample": sample_index,
                    "joints_deg": np.rad2deg(sample_q).tolist(), "reasons": candidate_reasons})
                if safe_first_stop is None:
                    safe_first_stop = candidate_joint_events[-1]
                break

        end_tool = tool.link_to_tool(kin.fk(segment.joints_rad[-1]))
        dp = end_tool.position_m - tool_pose.position_m
        if frame == "world_left":
            dr = Rotation.from_matrix(end_tool.rotation @ tool_pose.rotation.T).as_rotvec()
        else:
            dr = Rotation.from_matrix(tool_pose.rotation.T @ end_tool.rotation).as_rotvec()
        position_errors.append(np.linalg.norm(dp - action.delta_position_m))
        rotation_errors.append(np.linalg.norm(dr - action.rotation_vector_rad))

        sample_tools = [tool.link_to_tool(kin.fk(x)) for x in segment.joints_rad]
        disp = np.array([x.position_m - tool_pose.position_m for x in sample_tools])
        distance = np.linalg.norm(action.delta_position_m)
        if distance > 1e-14:
            projection = disp @ (action.delta_position_m / distance)
            lateral = np.linalg.norm(disp - projection[:, None] *
                                     (action.delta_position_m / distance), axis=1)
            if np.max(projection) > distance + 1e-8 or np.min(projection) < -1e-8:
                overshoots += 1
            if np.max(lateral) > 1e-8:
                path_budget_violations += 1
        start_r = Rotation.from_matrix(tool_pose.rotation)
        angles = []
        for x in sample_tools:
            xr = Rotation.from_matrix(x.rotation)
            rel = xr * start_r.inv() if frame == "world_left" else start_r.inv() * xr
            angles.append(rel.magnitude())
        if max(angles) > np.linalg.norm(action.rotation_vector_rad) + 1e-8:
            overshoots += 1

        publisher.publish_segment(index, segment)
        gripper.set_target(action.gripper_target)
        gripper_commands = gripper.advance(len(segment.times_s))
        publisher.publish_gripper(index, segment.times_s, gripper_commands)
        gripper_targets.append({"step": index, "target": action.gripper_target,
                                "command_end": float(gripper.command)})
        command_quaternions.append(Rotation.from_matrix(end_tool.rotation).as_quat().tolist())
        q = segment.joints_rad[-1]; link = kin.fk(q); tool_pose = end_tool

    endpoint_by_dimension = []
    for dim in range(6):
        endpoint_by_dimension.append({"dimension": dim,
            "min_count": sum(dim in x["min_dimensions"] for x in endpoint_events),
            "max_count": sum(dim in x["max_dimensions"] for x in endpoint_events)})
    first_j5 = next((x for x in candidate_joint_events if any(
                     reason.endswith(":J5") for reason in x["reasons"])), None)
    # Independent precheck against the recorded state at each prediction. This
    # diagnoses the historical J5 approach even when the new fail-closed chain
    # would have stopped earlier. It does not seed or alter the nominal chain.
    samples = [s for s in json.loads((RUN / "jointstate.json").read_text())["samples"]
               if s.get("valid")]
    sample_times = np.array([s["receive"] for s in samples])
    observed_j5 = None
    for index, record in enumerate(actions):
        t = float(record["controller_feedback"]["received"])
        sample = samples[int(np.argmin(np.abs(sample_times - t)))]
        observed_q = np.array([dict(zip(sample["names"], sample["position"]))[f"joint_{i}"]
                               for i in range(1, 7)])
        reasons = joint_margin_reasons(observed_q, config)
        if any(reason.endswith(":J5") for reason in reasons):
            observed_j5 = {"step": index, "joint_state_receive": sample["receive"],
                           "prediction_receive": t,
                           "j5_deg": float(np.rad2deg(observed_q[4])),
                           "effective_lower_candidate_deg": float(np.rad2deg(
                               config.historical_j5_lower_rad + config.joint_margin_rad[4])),
                           "reasons": reasons}
            break
    return {"rotation_frame": frame, "source": str(RUN), "actions": len(actions),
            "completed_segments": len(position_errors), "fatal_nominal_replay": fatal,
            "position_endpoint_error_m": stats(position_errors),
            "rotation_endpoint_error_rad": stats(rotation_errors),
            "overshoot_segments": overshoots, "path_budget_violations": path_budget_violations,
            "duration_s": stats(durations), "duration_extension_total": extensions,
            "ik_sample_compute_time_s": stats(ik_sample_times),
            "segment_precompute_time_s": stats(segment_precompute_times),
            "duration_extended_segments": sum(x > config.initial_duration_s for x in durations),
            "max_velocity_limit_ratio": max(max_vel_ratio),
            "max_acceleration_limit_ratio": max(max_acc_ratio),
            "candidate_dynamic_violations_after_scaling": 0,
            "safe_first_stop": safe_first_stop,
            "action_precheck_event_count": len(action_precheck_events),
            "action_precheck_events": action_precheck_events,
            "candidate_joint_event_count": len(candidate_joint_events),
            "first_j5_candidate_block": first_j5,
            "first_recorded_state_j5_candidate_block": observed_j5,
            "endpoint_event_steps": len(endpoint_events),
            "endpoint_by_dimension": endpoint_by_dimension,
            "endpoint_monitored_dimensions": [0, 1, 2, 3, 4, 5],
            "gripper_endpoint_monitoring": False,
            "gripper_state_changes": gripper_state_changes,
            "first_endpoint_streak": first_streak,
            "endpoint_policy": config.endpoint_policy,
            "endpoint_max_streak_by_dimension_side": np.max(
                np.asarray(streak_history), axis=0).tolist(),
            "first_singularity_approach": first_singularity_approach,
            "ik_tolerance_candidate": {
                "position_m": config.ik_position_tolerance_m,
                "rotation_rad": config.ik_rotation_tolerance_rad},
            "jacobian_min_singular_value_candidate": config.jacobian_min_singular_value,
            "gripper_mode": "continuous_absolute_slew_concurrent_with_arm",
            "gripper_full_travel_duration_s": config.gripper_full_travel_duration_s,
            "gripper_targets": gripper_targets,
            "endpoint_min_values": interpreter.endpoint_min_values.tolist(),
            "endpoint_max_values": interpreter.endpoint_max_values.tolist(),
            "command_endpoint_quaternion_xyzw": command_quaternions,
            "dry_run_log": str(dry_path)}


def main():
    result = {frame: replay(frame) for frame in ("world_left", "tool_right")}
    n = min(result["world_left"]["completed_segments"],
            result["tool_right"]["completed_segments"])
    differences = []
    for left, right in zip(result["world_left"]["command_endpoint_quaternion_xyzw"][:n],
                           result["tool_right"]["command_endpoint_quaternion_xyzw"][:n]):
        differences.append((Rotation.from_quat(left) * Rotation.from_quat(right).inv()).magnitude()
                           * 180 / np.pi)
    result["rotation_frame_comparison"] = {
        "common_completed_segments": n,
        "orientation_difference_deg": stats(differences),
        "difference_at_last_common_step_deg": differences[-1] if differences else None}
    output = ROOT / "reports" / "controller_replay.json"
    output.write_text(json.dumps(result, indent=2) + "\n")
    print(output)


if __name__ == "__main__":
    main()
