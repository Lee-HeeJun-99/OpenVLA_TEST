#!/usr/bin/python3
"""Reproduce offline real-log and sim-data FK validation; never connects to ROS."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from kinematics import PyKDLKinematics

REAL_ROOT = Path("/home/ubuntu/a0509_vla_linux_field_bundle_20260903/lhj/phase12_real_policy_rollout/14_real_rollout_integration/real_trials")
SIM_ROOT = Path("/home/ubuntu/robot_ws/src/doosan-robot2/raw_dataset_oft")


def stats(values):
    a = np.asarray(values, dtype=float)
    return {"count": int(a.size), "p50": float(np.percentile(a, 50)),
            "p99": float(np.percentile(a, 99)), "max": float(np.max(a))}


def ordered_joint(sample):
    by_name = dict(zip(sample["names"], sample["position"]))
    return np.array([by_name[f"joint_{i}"] for i in range(1, 7)])


def validate_real(kin, root):
    pos_mm, rot_deg, time_gap_ms, sources = [], [], [], []
    for path in root.rglob("episode_records.json"):
        try:
            records = json.loads(path.read_text())
        except (json.JSONDecodeError, OSError):
            continue
        if not isinstance(records, list):
            continue
        used = 0
        for item in records:
            try:
                js = item["jointstate_before"]
                obs = item["record"]["observation"]
                tcp = np.asarray(obs["tcp_m_abc"], dtype=float)
                fk = kin.fk(ordered_joint(js))
                # Driver audit in the source tree records controller angles as
                # intrinsic ZYZ degrees and translation as metres here.
                measured_r = Rotation.from_euler("ZYZ", tcp[3:], degrees=True).as_matrix()
                pos_mm.append(np.linalg.norm(tcp[:3] - fk.position_m) * 1000)
                rot_deg.append(np.linalg.norm(Rotation.from_matrix(
                    measured_r @ fk.rotation.T).as_rotvec()) * 180 / np.pi)
                time_gap_ms.append(abs(float(obs["receive_monotonic"]) -
                                       float(js["receive"])) * 1000)
                used += 1
            except (KeyError, TypeError, ValueError):
                continue
        if used:
            sources.append({"path": str(path), "pairs": used})
    if not pos_mm:
        return {"status": "NO_PAIRED_SAMPLES"}
    return {
        "status": "POSX_FALLBACK_NO_F_CURRENT_POS_WORLD_FLANGE_IN_PAIRED_LOG",
        "controller_pose": "current posx; zero-offset engineering assumption in source logs",
        "position_error_mm": stats(pos_mm), "rotation_error_deg": stats(rot_deg),
        "pair_receive_time_gap_ms": stats(time_gap_ms), "sources": sources,
    }


def mean_rotation(rotations):
    # Markley quaternion average, sign invariant.
    quats = np.array([r.as_quat() for r in rotations])
    vals, vecs = np.linalg.eigh(quats.T @ quats)
    return Rotation.from_quat(vecs[:, np.argmax(vals)])


def validate_sim(kin, root):
    relative_positions, relative_rotations, sources = [], [], []
    for path in sorted(root.glob("episodes/*/steps.jsonl")):
        used = 0
        with path.open() as stream:
            for line in stream:
                try:
                    item = json.loads(line)
                    q = np.asarray(item["joint_position_rad"][:6], dtype=float)
                    ee = item["end_effector_pose"]
                    p = np.asarray(ee["position_m"], dtype=float)
                    qwxyz = np.asarray(ee["quaternion_wxyz"], dtype=float)
                    recorded_r = Rotation.from_quat(qwxyz[[1, 2, 3, 0]]).as_matrix()
                    fk = kin.fk(q)
                    relative_positions.append(fk.rotation.T @ (p - fk.position_m))
                    relative_rotations.append(Rotation.from_matrix(fk.rotation.T @ recorded_r))
                    used += 1
                except (KeyError, ValueError, json.JSONDecodeError):
                    continue
        if used:
            sources.append({"path": str(path), "samples": used})
    if not relative_positions:
        return {"status": "NO_JOINT_AND_EE_PAIRS"}
    rel_p = np.asarray(relative_positions)
    mean_p = np.mean(rel_p, axis=0)
    mean_r = mean_rotation(relative_rotations)
    p_residual_mm = np.linalg.norm(rel_p - mean_p, axis=1) * 1000
    r_residual_deg = np.array([(mean_r.inv() * r).magnitude()
                               for r in relative_rotations]) * 180 / np.pi
    return {
        "status": "CONSTANT_OFFSET_TESTED",
        "declared_quaternion_order_used": "wxyz",
        "estimated_link6_to_recorded_tool0_xyz_m": mean_p.tolist(),
        "estimated_link6_to_recorded_tool0_rpy_deg": mean_r.as_euler("xyz", degrees=True).tolist(),
        "after_constant_offset_position_residual_mm": stats(p_residual_mm),
        "after_constant_offset_rotation_residual_deg": stats(r_residual_deg),
        "constant_offset_supported": bool(np.max(p_residual_mm) <= 1.0 and
                                           np.max(r_residual_deg) <= 0.1),
        "sources": sources,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--real-root", type=Path, default=REAL_ROOT)
    parser.add_argument("--sim-root", type=Path, default=SIM_ROOT)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    kin = PyKDLKinematics()
    result = {"urdf": str(kin.urdf_path), "base": kin.base_link, "tip": kin.tip_link,
              "real": validate_real(kin, args.real_root),
              "sim": validate_sim(kin, args.sim_root)}
    text = json.dumps(result, indent=2, ensure_ascii=False) + "\n"
    if args.output:
        args.output.write_text(text)
    else:
        print(text, end="")


if __name__ == "__main__":
    main()
