#!/usr/bin/python3
"""Offline diagnostics for validation_2. No ROS imports or robot I/O."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from kinematics import PyKDLKinematics

REAL_ROOT = Path("/home/ubuntu/a0509_vla_linux_field_bundle_20260903/lhj/phase12_real_policy_rollout/14_real_rollout_integration/real_trials")
SIM_REPLAY_ROOT = Path("/home/ubuntu/a0509_vla_linux_field_bundle_20260903/lhj/phase11_sim_condition_replication/dataset/episodes/baseline")
RAW_CANDIDATE = Path("/home/ubuntu/robot_ws/src/doosan-robot2/raw_dataset_oft")


def summary(x):
    a = np.asarray(x, dtype=float)
    if a.size == 0:
        return {"n": 0, "p50": None, "p99": None, "max": None}
    return {"n": int(a.size), "p50": float(np.percentile(a, 50)),
            "p99": float(np.percentile(a, 99)), "max": float(np.max(a))}


def vector_summary(x):
    a = np.asarray(x, dtype=float)
    return {"mean": np.mean(a, axis=0).tolist(), "std": np.std(a, axis=0).tolist()}


def pose_from_posx(pose):
    a = np.asarray(pose, dtype=float)
    return a[:3] / 1000.0, Rotation.from_euler("ZYZ", a[3:], degrees=True)


def ordered_joint(sample):
    values = dict(zip(sample["names"], sample["position"]))
    return np.array([values[f"joint_{i}"] for i in range(1, 7)])


def load_real_pairs(kin):
    pairs, files = [], []
    for directory in sorted(REAL_ROOT.glob("*servo_episode")):
        js_path, servo_path = directory / "jointstate.json", directory / "servo_commands.jsonl"
        if not js_path.exists() or not servo_path.exists():
            continue
        samples = json.loads(js_path.read_text()).get("samples", [])
        samples = [s for s in samples if s.get("valid") and len(s.get("position", [])) == 6]
        if not samples:
            continue
        times = np.array([s["receive"] for s in samples])
        feedback = {}
        for line in servo_path.open():
            try:
                item = json.loads(line)
                fb = item.get("latest_controller_feedback")
                if fb and len(fb.get("pose", [])) == 6:
                    feedback[float(fb["received"])] = fb
            except (json.JSONDecodeError, TypeError, ValueError):
                pass
        used = 0
        for received, fb in sorted(feedback.items()):
            index = int(np.argmin(np.abs(times - received)))
            js = samples[index]
            q = ordered_joint(js)
            fk = kin.fk(q)
            p_rec, r_rec = pose_from_posx(fb["pose"])
            pairs.append({"run": directory.name, "t": received, "js_t": float(js["receive"]),
                          "velocity": ordered_joint({"names": js["names"], "position": js["velocity"]}),
                          "fk_p": fk.position_m, "fk_r": Rotation.from_matrix(fk.rotation),
                          "rec_p": p_rec, "rec_r": r_rec})
            used += 1
        if used:
            files.append({"directory": str(directory), "pairs": used})
    return pairs, files


def real_analysis(kin):
    pairs, files = load_real_pairs(kin)
    # Analytical selection only, not a proposed controller/safety threshold.
    stopped = [x for x in pairs if np.max(np.abs(x["velocity"])) <= 1e-6]
    pos_vectors_mm = [(x["rec_p"] - x["fk_p"]) * 1000 for x in stopped]
    rot_vectors_deg = [(x["rec_r"] * x["fk_r"].inv()).as_rotvec(degrees=True)
                       for x in stopped]
    gaps_ms = [abs(x["t"] - x["js_t"]) * 1000 for x in pairs]

    delta_pos, delta_rot, delta_dt = [], [], []
    by_run = {}
    for item in pairs:
        by_run.setdefault(item["run"], []).append(item)
    for run in by_run.values():
        run.sort(key=lambda x: x["t"])
        times = np.array([x["t"] for x in run])
        for i, first in enumerate(run):
            target = first["t"] + 0.2
            j = int(np.argmin(np.abs(times - target)))
            dt = times[j] - first["t"]
            if j <= i or abs(dt - 0.2) > 0.03:
                continue
            second = run[j]
            fk_dp = second["fk_p"] - first["fk_p"]
            rec_dp = second["rec_p"] - first["rec_p"]
            fk_dr = second["fk_r"] * first["fk_r"].inv()
            rec_dr = second["rec_r"] * first["rec_r"].inv()
            delta_pos.append(np.linalg.norm(rec_dp - fk_dp) * 1000)
            delta_rot.append((rec_dr * fk_dr.inv()).magnitude() * 180 / np.pi)
            delta_dt.append(dt)
    return {
        "all_pairs": len(pairs), "stopped_definition_rad_s": "max(abs(v[0:6])) <= 1e-6",
        "stopped_pairs": len(stopped),
        "stopped_position_error_norm_mm": summary(np.linalg.norm(pos_vectors_mm, axis=1)),
        "stopped_rotation_error_norm_deg": summary(np.linalg.norm(rot_vectors_deg, axis=1)),
        "stopped_position_bias_and_std_xyz_mm": vector_summary(pos_vectors_mm),
        "stopped_rotation_bias_and_std_rotvec_xyz_deg": vector_summary(rot_vectors_deg),
        "jointstate_posx_receive_gap_ms": summary(gaps_ms),
        "delta_pair_definition": "nearest 0.2 s, accepted |dt-0.2| <= 0.03 s, within run",
        "delta_actual_dt_s": summary(delta_dt),
        "delta_position_disagreement_mm": summary(delta_pos),
        "delta_rotation_disagreement_deg": summary(delta_rot), "files": files,
    }


def load_sim_records():
    out, files = [], []
    for path in sorted(SIM_REPLAY_ROOT.glob("baseline_episode_*.json")):
        data = json.loads(path.read_text())
        used = 0
        for row in data.get("records", []):
            try:
                q = np.asarray(row["actual_joint_position_rad"][:6], dtype=float)
                pose = row["sim_end_effector_pose"]
                out.append((q, np.asarray(pose["position_m"], dtype=float),
                            np.asarray(pose["quaternion_wxyz"], dtype=float)))
                used += 1
            except (KeyError, ValueError, TypeError):
                pass
        if used:
            files.append({"path": str(path), "records": used})
    return out, files


def transform(params):
    return Rotation.from_rotvec(params[:3]), params[3:6]


def apply_pose(a_r, a_p, x_r, x_p, b_r, b_p):
    r = a_r * x_r * b_r
    p = a_p + a_r.apply(x_p + x_r.apply(b_p))
    return r, p


def fit_two_sided(kin, records, order, q_sign=None, q_offset=None):
    sign = np.ones(6) if q_sign is None else np.asarray(q_sign)
    offset = np.zeros(6) if q_offset is None else np.asarray(q_offset)
    xs, ys = [], []
    for q, p, quat in records:
        fk = kin.fk(sign * q + offset)
        xs.append((Rotation.from_matrix(fk.rotation), fk.position_m))
        xyzw = quat[[1, 2, 3, 0]] if order == "wxyz" else quat
        ys.append((Rotation.from_quat(xyzw), p))
    # Known scene/URDF values are initialization only; all 12 variables are fitted.
    initial = np.zeros(12)
    initial[5] = 0.823528
    initial[6:9] = [0.0, 0.0, -np.pi / 2]
    initial[11] = 0.01

    fit_indices = np.linspace(0, len(xs) - 1, min(40, len(xs)), dtype=int)

    def residual(params):
        ar, ap = transform(params[:6]); br, bp = transform(params[6:])
        values = []
        for index in fit_indices:
            xr, xp = xs[index]; yr, yp = ys[index]
            pr, pp = apply_pose(ar, ap, xr, xp, br, bp)
            values.extend((pp - yp) / 0.001)
            values.extend((pr * yr.inv()).as_rotvec() / np.deg2rad(0.1))
        return values
    fit = least_squares(residual, initial, max_nfev=120, method="trf")
    ar, ap = transform(fit.x[:6]); br, bp = transform(fit.x[6:])
    pe, re = [], []
    for (xr, xp), (yr, yp) in zip(xs, ys):
        pr, pp = apply_pose(ar, ap, xr, xp, br, bp)
        pe.append(np.linalg.norm(pp - yp) * 1000)
        re.append((pr * yr.inv()).magnitude() * 180 / np.pi)
    return {"success": bool(fit.success), "cost": float(fit.cost),
            "world_base_xyz_m": ap.tolist(), "world_base_rpy_deg": ar.as_euler("xyz", degrees=True).tolist(),
            "link6_tool0_xyz_m": bp.tolist(), "link6_tool0_rpy_deg": br.as_euler("xyz", degrees=True).tolist(),
            "position_residual_mm": summary(pe), "rotation_residual_deg": summary(re)}


def sim_analysis(kin):
    records, files = load_sim_records()
    base = {order: fit_two_sided(kin, records, order) for order in ("wxyz", "xyzw")}
    # Step 4: one-at-a-time sign flips. Constant joint offsets are structurally
    # confounded with A/B for end joints; test ±pi/2 and pi as explicit hypotheses.
    searches = []
    for joint in range(6):
        sign = np.ones(6); sign[joint] = -1
        result = fit_two_sided(kin, records, "wxyz", q_sign=sign)
        searches.append({"hypothesis": f"flip_joint_{joint+1}", **result})
    for joint in range(6):
        for degrees in (-180, -90, 90, 180):
            offset = np.zeros(6); offset[joint] = np.deg2rad(degrees)
            result = fit_two_sided(kin, records, "wxyz", q_offset=offset)
            searches.append({"hypothesis": f"joint_{joint+1}_offset_{degrees:+d}deg", **result})
    searches.sort(key=lambda x: (x["position_residual_mm"]["p99"],
                                 x["rotation_residual_deg"]["p99"]))
    raw = json.loads(next((RAW_CANDIDATE / "episodes").glob("*/steps.jsonl")).read_text().splitlines()[0])
    metadata = json.loads(next((RAW_CANDIDATE / "episodes").glob("*/metadata.json")).read_text())
    return {"sim_record_count": len(records), "sim_files": files,
            "raw_candidate_actual_sample": raw,
            "raw_candidate_metadata_extract": {k: metadata.get(k) for k in
                ("dataset_format", "coordinate_frame", "pose_source", "position_unit",
                 "rotation_unit", "feedback_pose_samples", "proprio_convention")},
            "two_sided": base, "joint_hypothesis_search_all": searches,
            "joint_hypothesis_search_top10": searches[:10]}


def main():
    kin = PyKDLKinematics()
    result = {"real": real_analysis(kin), "sim": sim_analysis(kin)}
    out = Path(__file__).resolve().parents[1] / "reports" / "kinematics_validation_2.json"
    out.write_text(json.dumps(result, indent=2, ensure_ascii=False) + "\n")
    print(out)


if __name__ == "__main__":
    main()
