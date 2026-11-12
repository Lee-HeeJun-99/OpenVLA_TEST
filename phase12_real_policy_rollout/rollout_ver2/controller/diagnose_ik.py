#!/usr/bin/python3
"""Offline numerical diagnosis of the two nominal replay IK failures."""

from __future__ import annotations

import json
from pathlib import Path
import sys

import numpy as np
from scipy.optimize import least_squares

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from controller.config import load_config
from controller.geometry import ToolTransform
from controller.stop_reasons import IKStopReason
from kinematics import DLSIKSolver, Pose, PyKDLKinematics


def pose_from_matrix(values):
    matrix = np.asarray(values, dtype=float)
    return Pose(matrix[:3, 3], matrix[:3, :3])


def residual(kin, q, target):
    return DLSIKSolver._error(kin.fk(q), target)


def diagnose(frame, record):
    kin = PyKDLKinematics(); config = load_config(rotation_frame=frame)
    fail = record["fatal_nominal_replay"]
    seed = np.asarray(fail["seed_joint_rad"], dtype=float)
    target = pose_from_matrix(fail["target_link_matrix"])
    tool_target = ToolTransform.tool0_real().link_to_tool(target)
    jac = kin.jacobian(seed); singular = np.linalg.svd(jac, compute_uv=False)
    initial_error = residual(kin, seed, target)
    attempts = []
    for damping in (1e-2, 1e-3, 1e-4, 1e-5, 1e-6):
        for iterations in (250, 1000, 4000):
            solver = DLSIKSolver(kin, damping=damping, max_iterations=iterations,
                                 position_tolerance_m=1e-8,
                                 rotation_tolerance_rad=1e-8)
            try:
                q = solver.solve(target, seed,
                    joint_lower_rad=kin.urdf_lower_rad,
                    joint_upper_rad=kin.urdf_upper_rad,
                    max_joint_change_rad=config.max_sample_joint_change_rad)
                err = residual(kin, q, target)
                attempts.append({"method": "dls", "damping": damping,
                    "max_iterations": iterations, "success": True,
                    "position_residual_m": float(np.linalg.norm(err[:3])),
                    "rotation_residual_rad": float(np.linalg.norm(err[3:])),
                    "joint_change_rad": (q-seed).tolist()})
            except Exception as error:
                attempts.append({"method": "dls", "damping": damping,
                    "max_iterations": iterations, "success": False,
                    "error": f"{type(error).__name__}:{error}"})
            if attempts[-1]["success"]:
                break

    local_lo = np.maximum(kin.urdf_lower_rad,
                          seed-config.max_sample_joint_change_rad)
    local_hi = np.minimum(kin.urdf_upper_rad,
                          seed+config.max_sample_joint_change_rad)
    lsq = least_squares(lambda q: residual(kin, q, target), seed,
                        bounds=(local_lo, local_hi), xtol=1e-14, ftol=1e-14,
                        gtol=1e-14, max_nfev=10000)
    lsq_error = residual(kin, lsq.x, target)
    rng = np.random.default_rng(9406 + (0 if frame == "world_left" else 1))
    starts = []
    for joint3 in (0.01, 0.03, 0.1, 0.3, -0.01, -0.03, -0.1, -0.3):
        candidate = seed.copy(); candidate[2] = joint3; starts.append(candidate)
    for _ in range(30):
        starts.append(np.clip(seed + rng.normal(0.0, 0.3, 6),
                              kin.urdf_lower_rad, kin.urdf_upper_rad))
    global_best = (float(np.linalg.norm(lsq_error)), lsq_error, lsq.x)
    for start in starts:
        trial = least_squares(lambda q: residual(kin, q, target), start,
                              bounds=(kin.urdf_lower_rad, kin.urdf_upper_rad),
                              xtol=1e-13, ftol=1e-13, gtol=1e-13,
                              max_nfev=5000)
        trial_error = residual(kin, trial.x, target)
        if np.linalg.norm(trial_error) < global_best[0]:
            global_best = (float(np.linalg.norm(trial_error)), trial_error, trial.x)
    successful = [x for x in attempts if x["success"]]
    if successful:
        classification = "ALGORITHM_LIMIT"
    elif (np.min(seed-kin.urdf_lower_rad) < 1e-5 or
          np.min(kin.urdf_upper_rad-seed) < 1e-5):
        classification = "JOINT_LIMIT"
    elif (np.linalg.norm(lsq_error[:3]) > 1e-8 or
          np.linalg.norm(lsq_error[3:]) > 1e-8):
        classification = "UNREACHABLE"
    elif singular[-1] < 1e-5:
        classification = "SINGULARITY"
    else:
        classification = "ALGORITHM_LIMIT"
    return {
        "frame": frame, "failure_step": fail["step"],
        "failure_sample": fail["sample"],
        "target_tool0_position_m": tool_target.position_m.tolist(),
        "target_tool0_rotation_matrix": tool_target.rotation.tolist(),
        "seed_joint_rad": seed.tolist(), "seed_joint_deg": np.rad2deg(seed).tolist(),
        "initial_position_residual_m": float(np.linalg.norm(initial_error[:3])),
        "initial_rotation_residual_rad": float(np.linalg.norm(initial_error[3:])),
        "original_solver_iterations_limit": 250,
        "original_solver_iterations_run": fail.get("iterations_run"),
        "jacobian_singular_values": singular.tolist(),
        "jacobian_min_singular_value": float(singular[-1]),
        "jacobian_condition_number": float(singular[0]/singular[-1]),
        "urdf_margin_to_lower_rad": (seed-kin.urdf_lower_rad).tolist(),
        "urdf_margin_to_upper_rad": (kin.urdf_upper_rad-seed).tolist(),
        "candidate_margin_to_lower_rad": (seed-config.effective_lower_rad).tolist(),
        "candidate_margin_to_upper_rad": (config.effective_upper_rad-seed).tolist(),
        "target_inside_workspace_candidate": bool(np.all(tool_target.position_m >= config.workspace_min_m)
                                                   and np.all(tool_target.position_m <= config.workspace_max_m)),
        "bounded_least_squares": {"success": bool(lsq.success), "nfev": int(lsq.nfev),
            "position_residual_m": float(np.linalg.norm(lsq_error[:3])),
            "rotation_residual_rad": float(np.linalg.norm(lsq_error[3:])),
            "joint_change_rad": (lsq.x-seed).tolist()},
        "multistart_reachability": {"starts": len(starts) + 1,
            "position_residual_m": float(np.linalg.norm(global_best[1][:3])),
            "rotation_residual_rad": float(np.linalg.norm(global_best[1][3:])),
            "joint_change_rad": (global_best[2]-seed).tolist()},
        "dls_attempts": attempts, "classification": classification,
        "classified_stop_reason_code": {
            "UNREACHABLE": IKStopReason.TARGET_UNREACHABLE.value,
            "JOINT_LIMIT": IKStopReason.JOINT_LIMIT.value,
            "SINGULARITY": IKStopReason.SINGULARITY.value,
            "ALGORITHM_LIMIT": IKStopReason.ALGORITHM_LIMIT.value,
        }[classification],
        "recommended_successful_dls": successful[0] if successful else None,
    }


def main():
    replay = json.loads((ROOT/"reports/controller_replay.json").read_text())
    result = {frame: diagnose(frame, replay[frame])
              for frame in ("world_left", "tool_right")}
    path = ROOT/"reports/controller_ik_diagnostics.json"
    path.write_text(json.dumps(result, indent=2)+"\n")
    print(path)


if __name__ == "__main__":
    main()
