#!/usr/bin/env python3
from __future__ import annotations

import csv
import json
import math
from pathlib import Path
from typing import Any

import numpy as np
from PIL import Image


ROOT = Path("/home/ubuntu/a0509_vla_linux_field_bundle_20260903")
PHASE6 = ROOT / "lhj" / "phase6_environment_attribution"
INPUT_ROOT = PHASE6 / "03_camera" / "condition_inputs"
FEATURE_ROOT = PHASE6 / "03_camera" / "full_forward_features"


def read_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def load_rgb(path: str | Path) -> Image.Image:
    return Image.open(path).convert("RGB")


def arr(image: Image.Image) -> np.ndarray:
    return np.asarray(image.convert("RGB"), dtype=np.float64) / 255.0


def luminance(x: np.ndarray) -> np.ndarray:
    return 0.2126 * x[..., 0] + 0.7152 * x[..., 1] + 0.0722 * x[..., 2]


def ssim_simple(x: np.ndarray, y: np.ndarray) -> float:
    gx = luminance(x)
    gy = luminance(y)
    mux = float(gx.mean())
    muy = float(gy.mean())
    vx = float(gx.var())
    vy = float(gy.var())
    cov = float(((gx - mux) * (gy - muy)).mean())
    c1 = 0.01**2
    c2 = 0.03**2
    return float(((2 * mux * muy + c1) * (2 * cov + c2)) / ((mux**2 + muy**2 + c1) * (vx + vy + c2)))


def pair_obs(real_path: Path, sim_path: Path) -> dict[str, float]:
    real = arr(load_rgb(real_path))
    sim = arr(load_rgb(sim_path))
    if real.shape != sim.shape:
        sim = arr(load_rgb(sim_path).resize((real.shape[1], real.shape[0]), Image.Resampling.BICUBIC))
    diff = real - sim
    mse = float(np.mean(diff**2))
    psnr = float("inf") if mse <= 1e-12 else float(10.0 * math.log10(1.0 / mse))
    return {
        "observation_rgb_l2_mean": float(np.linalg.norm(diff.reshape(-1, 3), axis=1).mean()),
        "observation_mse": mse,
        "observation_psnr": psnr,
        "observation_ssim_global": ssim_simple(real, sim),
    }


def feature_records(condition: str, domain: str) -> dict[str, dict[str, Any]]:
    path = FEATURE_ROOT / condition / domain / "feature_manifest.json"
    if not path.is_file():
        return {}
    manifest = read_json(path)
    base = path.parent
    records = {}
    for record in manifest["records"]:
        pair_id = Path(record["source_image"]).stem
        records[pair_id] = {**record, "_feature_base": str(base)}
    return records


def feature(record: dict[str, Any], key: str) -> np.ndarray:
    data = np.load(Path(record["_feature_base"]) / record["feature_file"], allow_pickle=True)
    return np.asarray(data[key], dtype=np.float64)


def action_chunk(record: dict[str, Any]) -> np.ndarray:
    response = record.get("response", {})
    actions = response.get("actions")
    if actions is None:
        actions = [response.get("action")]
    arr = np.asarray(actions, dtype=np.float64)
    if arr.ndim == 1:
        arr = arr.reshape(1, -1)
    return arr


def l2(x: np.ndarray) -> float:
    return float(np.linalg.norm(x))


def token_l2(left: np.ndarray, right: np.ndarray) -> float:
    diff = left - right
    if diff.ndim >= 3:
        diff = diff.reshape(-1, diff.shape[-1])
    return float(np.mean(np.linalg.norm(diff, axis=-1)))


def summarize(values: list[float]) -> float:
    arrv = np.asarray(values, dtype=np.float64)
    return float(arrv.mean()) if arrv.size else float("nan")


def main() -> int:
    manifest = read_json(INPUT_ROOT / "camera_condition_manifest.json")
    summary_rows: list[dict[str, Any]] = []
    frame_rows: list[dict[str, Any]] = []
    for condition, meta in manifest["conditions"].items():
        real_list = [Path(line) for line in Path(meta["real_images_txt"]).read_text(encoding="utf-8").splitlines() if line]
        sim_list = [Path(line) for line in Path(meta["sim_images_txt"]).read_text(encoding="utf-8").splitlines() if line]
        obs_metrics = [pair_obs(r, s) for r, s in zip(real_list, sim_list)]
        real_records = feature_records(condition, "real")
        sim_records = feature_records(condition, "sim")
        common = sorted(set(real_records) & set(sim_records))
        full_forward = len(common) == len(real_list)
        action_vals: dict[str, list[float]] = {k: [] for k in [
            "vision_gap", "projector_gap", "hidden_gap", "action_gap", "chunk_max_l2",
            "first_action_l2", "translation_gap", "rotation_gap", "gripper_gap",
        ]}
        if full_forward:
            for pair_id in common:
                rr = real_records[pair_id]
                ss = sim_records[pair_id]
                rv = feature(rr, "vision_backbone.output")
                sv = feature(ss, "vision_backbone.output")
                rp = feature(rr, "projector.output")
                sp = feature(ss, "projector.output")
                rh = feature(rr, "action_hidden_states.input")
                sh = feature(ss, "action_hidden_states.input")
                ra = action_chunk(rr)
                sa = action_chunk(ss)
                n = min(len(ra), len(sa))
                da = ra[:n] - sa[:n]
                first = da[0]
                row = {
                    "pair_id": pair_id,
                    "condition": condition,
                    "vision_gap": l2(rv.mean(axis=1) - sv.mean(axis=1)),
                    "projector_gap": l2(rp.mean(axis=1) - sp.mean(axis=1)),
                    "hidden_gap": l2(rh - sh),
                    "action_gap": float(np.mean(np.linalg.norm(da, axis=1))),
                    "chunk_max_l2": float(np.max(np.linalg.norm(da, axis=1))),
                    "first_action_l2": l2(first),
                    "translation_gap": l2(first[:3]),
                    "rotation_gap": l2(first[3:6]),
                    "gripper_gap": float(abs(first[6])),
                }
                frame_rows.append(row)
                for key in action_vals:
                    action_vals[key].append(float(row[key]))
        record = {
            "factor": "camera_image_space",
            "condition": condition,
            "description": meta["description"],
            "transform": meta["transform"],
            "value": meta["value"],
            "count": len(real_list),
            "observation_rgb_l2_mean": summarize([m["observation_rgb_l2_mean"] for m in obs_metrics]),
            "observation_mse": summarize([m["observation_mse"] for m in obs_metrics]),
            "observation_psnr": summarize([m["observation_psnr"] for m in obs_metrics]),
            "observation_ssim_global": summarize([m["observation_ssim_global"] for m in obs_metrics]),
            "vision_gap": summarize(action_vals["vision_gap"]) if full_forward else "",
            "projector_gap": summarize(action_vals["projector_gap"]) if full_forward else "",
            "hidden_gap": summarize(action_vals["hidden_gap"]) if full_forward else "",
            "action_gap": summarize(action_vals["action_gap"]) if full_forward else "",
            "chunk_max_l2": summarize(action_vals["chunk_max_l2"]) if full_forward else "",
            "first_action_l2": summarize(action_vals["first_action_l2"]) if full_forward else "",
            "translation_gap": summarize(action_vals["translation_gap"]) if full_forward else "",
            "rotation_gap": summarize(action_vals["rotation_gap"]) if full_forward else "",
            "gripper_gap": summarize(action_vals["gripper_gap"]) if full_forward else "",
            "status": "VERIFIED_FULL_FORWARD" if full_forward else "OBSERVATION_ONLY_FULL_FORWARD_REQUIRED",
        }
        summary_rows.append(record)
    out = PHASE6 / "camera_ablation.csv"
    with out.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(summary_rows[0].keys()))
        writer.writeheader()
        writer.writerows(summary_rows)
    if frame_rows:
        with (PHASE6 / "03_camera" / "camera_frame_metrics.csv").open("w", encoding="utf-8", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=list(frame_rows[0].keys()))
            writer.writeheader()
            writer.writerows(frame_rows)
    write_json(
        PHASE6 / "03_camera" / "camera_sensitivity_summary.json",
        {
            "condition_count": len(summary_rows),
            "full_forward_conditions": sum(row["status"] == "VERIFIED_FULL_FORWARD" for row in summary_rows),
            "status": "VERIFIED_FULL_FORWARD" if frame_rows else "OBSERVATION_ONLY_FULL_FORWARD_REQUIRED",
            "camera_ablation_csv": str(out),
            "interpretation": "Image-space camera sensitivity, not calibrated camera alignment.",
        },
    )
    print(json.dumps({"rows": len(summary_rows), "full_forward": bool(frame_rows)}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
