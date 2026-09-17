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
INPUT_ROOT = PHASE6 / "05_combined" / "condition_inputs"
FEATURE_ROOT = PHASE6 / "05_combined" / "full_forward_features"
BASIS_FILE = PHASE6 / "06_policy_relevance" / "sensitive_energy" / "phase6_p0_sensitive_basis_k128.npz"


def read_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    fields: list[str] = []
    for row in rows:
        for key in row:
            if key not in fields:
                fields.append(key)
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def arr(path: Path) -> np.ndarray:
    return np.asarray(Image.open(path).convert("RGB"), dtype=np.float64) / 255.0


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


def obs_metrics(real_path: Path, sim_path: Path) -> dict[str, float]:
    real = arr(real_path)
    sim = arr(sim_path)
    if real.shape != sim.shape:
        sim_img = Image.open(sim_path).convert("RGB").resize((real.shape[1], real.shape[0]), Image.Resampling.BICUBIC)
        sim = np.asarray(sim_img, dtype=np.float64) / 255.0
    diff = real - sim
    mse = float(np.mean(diff**2))
    return {
        "observation_rgb_l2_mean": float(np.linalg.norm(diff.reshape(-1, 3), axis=1).mean()),
        "observation_mse": mse,
        "observation_psnr": float("inf") if mse <= 1e-12 else float(10.0 * math.log10(1.0 / mse)),
        "observation_ssim_global": ssim_simple(real, sim),
    }


def feature_records(condition: str, domain: str) -> dict[str, dict[str, Any]]:
    path = FEATURE_ROOT / condition / domain / "feature_manifest.json"
    if not path.is_file():
        return {}
    manifest = read_json(path)
    records = {}
    for record in manifest["records"]:
        pair_id = Path(record["source_image"]).stem
        records[pair_id] = {**record, "_feature_base": str(path.parent)}
    return records


def feature(record: dict[str, Any], key: str) -> np.ndarray:
    with np.load(Path(record["_feature_base"]) / record["feature_file"], allow_pickle=True) as data:
        return np.asarray(data[key], dtype=np.float64)


def action_chunk(record: dict[str, Any]) -> np.ndarray:
    response = record.get("response", {})
    actions = response.get("actions") or [response.get("action")]
    out = np.asarray(actions, dtype=np.float64)
    if out.ndim == 1:
        out = out.reshape(1, -1)
    return out


def projection_energy(delta: np.ndarray, basis: np.ndarray) -> dict[str, float]:
    flat = delta.reshape(-1).astype(np.float64)
    total = float(np.linalg.norm(flat))
    if total <= 1e-12 or basis.size == 0:
        return {"sensitive_energy": 0.0, "null_energy": total, "sensitive_ratio": 0.0}
    coeff = basis @ flat
    sensitive = float(np.linalg.norm(coeff))
    null = float(max(total * total - sensitive * sensitive, 0.0) ** 0.5)
    return {"sensitive_energy": sensitive, "null_energy": null, "sensitive_ratio": sensitive / total}


def mean(rows: list[dict[str, Any]], key: str) -> float:
    vals = np.asarray([float(row[key]) for row in rows], dtype=np.float64)
    return float(vals.mean()) if vals.size else float("nan")


def main() -> int:
    manifest = read_json(INPUT_ROOT / "combined_condition_manifest.json")
    basis = np.load(BASIS_FILE, allow_pickle=True)["basis"].astype(np.float64)
    frame_rows: list[dict[str, Any]] = []
    summary_rows: list[dict[str, Any]] = []

    for condition, meta in manifest["conditions"].items():
        real_paths = [Path(line) for line in Path(meta["real_images_txt"]).read_text(encoding="utf-8").splitlines() if line]
        sim_paths = [Path(line) for line in Path(meta["sim_images_txt"]).read_text(encoding="utf-8").splitlines() if line]
        obs = [obs_metrics(r, s) for r, s in zip(real_paths, sim_paths)]
        real_records = feature_records(condition, "real")
        sim_records = feature_records(condition, "sim")
        common = sorted(set(real_records) & set(sim_records))
        condition_rows = []
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
                "condition": condition,
                "pair_id": pair_id,
                "vision_gap": float(np.linalg.norm(rv.mean(axis=1) - sv.mean(axis=1))),
                "projector_gap": float(np.linalg.norm(rp.mean(axis=1) - sp.mean(axis=1))),
                "hidden_gap": float(np.linalg.norm(rh - sh)),
                "action_gap": float(np.mean(np.linalg.norm(da, axis=1))),
                "chunk_max_l2": float(np.max(np.linalg.norm(da, axis=1))),
                "first_action_l2": float(np.linalg.norm(first)),
                "translation_gap": float(np.linalg.norm(first[:3])),
                "rotation_gap": float(np.linalg.norm(first[3:6])),
                "gripper_gap": float(abs(first[6])),
                **projection_energy(sh - rh, basis),
            }
            condition_rows.append(row)
            frame_rows.append(row)

        full = len(condition_rows) == len(real_paths)
        summary = {
            "factor": "combined_image_space",
            "condition": condition,
            "description": meta["description"],
            "count": len(real_paths),
            "observation_rgb_l2_mean": float(np.mean([item["observation_rgb_l2_mean"] for item in obs])),
            "observation_mse": float(np.mean([item["observation_mse"] for item in obs])),
            "observation_psnr": float(np.mean([item["observation_psnr"] for item in obs])),
            "observation_ssim_global": float(np.mean([item["observation_ssim_global"] for item in obs])),
            "status": "VERIFIED_FULL_FORWARD" if full else "OBSERVATION_ONLY_FULL_FORWARD_REQUIRED",
        }
        if full:
            for key in [
                "vision_gap",
                "projector_gap",
                "hidden_gap",
                "action_gap",
                "chunk_max_l2",
                "first_action_l2",
                "translation_gap",
                "rotation_gap",
                "gripper_gap",
                "sensitive_energy",
                "null_energy",
                "sensitive_ratio",
            ]:
                summary[key] = mean(condition_rows, key)
        summary_rows.append(summary)

    if summary_rows and all(row["status"] == "VERIFIED_FULL_FORWARD" for row in summary_rows):
        base = next(row for row in summary_rows if row["condition"] == "K0_P4_letterbox")
        for row in summary_rows:
            for metric in ["action_gap", "hidden_gap", "vision_gap", "sensitive_energy"]:
                row[f"{metric}_reduction_vs_k0_pct"] = (
                    (float(base[metric]) - float(row[metric])) / float(base[metric]) * 100.0 if float(base[metric]) else 0.0
                )

    write_csv(PHASE6 / "combined_ablation.csv", summary_rows)
    write_csv(PHASE6 / "tables" / "combined_ablation.csv", summary_rows)
    write_csv(PHASE6 / "05_combined" / "combined_frame_metrics.csv", frame_rows)
    write_json(
        PHASE6 / "05_combined" / "combined_summary.json",
        {
            "condition_count": len(summary_rows),
            "full_forward_conditions": sum(row["status"] == "VERIFIED_FULL_FORWARD" for row in summary_rows),
            "status": "VERIFIED_FULL_FORWARD" if frame_rows else "OBSERVATION_ONLY_FULL_FORWARD_REQUIRED",
            "combined_ablation_csv": str(PHASE6 / "combined_ablation.csv"),
            "interpretation": "Combined P4+C2 is still an image-space sensitivity condition, not calibrated camera alignment.",
        },
    )
    print(json.dumps({"rows": len(summary_rows), "full_forward": bool(frame_rows)}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
