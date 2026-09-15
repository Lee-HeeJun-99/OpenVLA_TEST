#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Any

import numpy as np
from PIL import Image


ROOT = Path("/home/ubuntu/a0509_vla_linux_field_bundle_20260903")
DEFAULT_ANALYSIS_ROOT = ROOT / "outputs" / "token_distribution_analysis" / "5_episodes"
DEFAULT_OUTPUT_DIR = ROOT / "lhj" / "phase1_observation_token_gap"
FEATURES = ["vision_backbone.output", "projector.output"]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--analysis-root", type=Path, default=DEFAULT_ANALYSIS_ROOT)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--resize-width", type=int, default=224)
    parser.add_argument("--resize-height", type=int, default=224)
    return parser.parse_args()


def read_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def frame_from_record(record: dict[str, Any]) -> str:
    return Path(record["source_image"]).stem


def records_by_frame(manifest_path: Path) -> dict[str, dict[str, Any]]:
    manifest = read_json(manifest_path)
    base = manifest_path.parent
    return {
        frame_from_record(record): {**record, "_feature_path": str(base / record["feature_file"])}
        for record in manifest["records"]
    }


def image_arr(path: Path, size: tuple[int, int] | None = None) -> np.ndarray:
    with Image.open(path) as image:
        image = image.convert("RGB")
        if size is not None:
            image = image.resize(size, Image.Resampling.BICUBIC)
        return np.asarray(image, dtype=np.float32)


def rgb_stats(arr: np.ndarray) -> dict[str, Any]:
    flat = arr.reshape(-1, 3)
    lum = 0.2126 * flat[:, 0] + 0.7152 * flat[:, 1] + 0.0722 * flat[:, 2]
    return {
        "rgb_mean": flat.mean(axis=0).tolist(),
        "rgb_std": flat.std(axis=0).tolist(),
        "brightness": float(lum.mean()),
        "contrast": float(lum.std()),
    }


def psnr(left: np.ndarray, right: np.ndarray) -> float:
    mse = float(np.mean((left - right) ** 2))
    if mse <= 1e-12:
        return float("inf")
    return float(20.0 * math.log10(255.0 / math.sqrt(mse)))


def simple_ssim_luma(left: np.ndarray, right: np.ndarray) -> float:
    x = 0.2126 * left[:, :, 0] + 0.7152 * left[:, :, 1] + 0.0722 * left[:, :, 2]
    y = 0.2126 * right[:, :, 0] + 0.7152 * right[:, :, 1] + 0.0722 * right[:, :, 2]
    c1 = (0.01 * 255.0) ** 2
    c2 = (0.03 * 255.0) ** 2
    mux = float(x.mean())
    muy = float(y.mean())
    vx = float(x.var())
    vy = float(y.var())
    cov = float(((x - mux) * (y - muy)).mean())
    return float(((2 * mux * muy + c1) * (2 * cov + c2)) / ((mux**2 + muy**2 + c1) * (vx + vy + c2)))


def load_tokens(path: Path, feature: str) -> np.ndarray:
    with np.load(path, allow_pickle=True) as data:
        arr = np.asarray(data[feature], dtype=np.float64)
    if arr.ndim == 3 and arr.shape[0] == 1:
        return arr[0]
    if arr.ndim == 3:
        return arr.reshape(-1, arr.shape[-1])
    if arr.ndim == 2:
        return arr
    return arr.reshape(1, -1)


def cosine_distance(left: np.ndarray, right: np.ndarray) -> float:
    denom = float(np.linalg.norm(left) * np.linalg.norm(right))
    if denom <= 1e-12:
        return 1.0
    return float(1.0 - np.dot(left, right) / denom)


def summarize(values: list[float]) -> dict[str, float | int]:
    arr = np.asarray(values, dtype=np.float64)
    return {
        "count": int(arr.size),
        "mean": float(arr.mean()),
        "std": float(arr.std()),
        "min": float(arr.min()),
        "p50": float(np.percentile(arr, 50)),
        "p90": float(np.percentile(arr, 90)),
        "max": float(arr.max()),
    }


def main() -> int:
    args = parse_args()
    analysis_root = args.analysis_root.expanduser().resolve()
    output_dir = args.output_dir.expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    paired = read_json(analysis_root / "paired_inputs" / "paired_manifest.json")["pairs"]
    real_records = records_by_frame(analysis_root / "features" / "real" / "feature_manifest.json")
    sim_records = records_by_frame(analysis_root / "features" / "sim" / "feature_manifest.json")
    frames = sorted(set(real_records) & set(sim_records))

    observation_rows: list[dict[str, Any]] = []
    pair_by_id = {pair["pair_id"]: pair for pair in paired}
    for frame in frames:
        pair = pair_by_id[frame]
        real_path = Path(pair["real"]["paired_image"])
        sim_path = Path(pair["sim"]["paired_image"])
        real_raw = image_arr(real_path)
        sim_raw = image_arr(sim_path)
        real_resized = image_arr(real_path, (args.resize_width, args.resize_height))
        sim_resized = image_arr(sim_path, (args.resize_width, args.resize_height))
        rs = rgb_stats(real_raw)
        ss = rgb_stats(sim_raw)
        row = {
            "frame": frame,
            "episode_id": pair["episode_id"],
            "step_index": pair["step_index"],
            "planner_phase": pair["planner_phase"],
            "real_width": int(real_raw.shape[1]),
            "real_height": int(real_raw.shape[0]),
            "sim_width": int(sim_raw.shape[1]),
            "sim_height": int(sim_raw.shape[0]),
            "real_brightness": rs["brightness"],
            "sim_brightness": ss["brightness"],
            "brightness_abs_diff": abs(rs["brightness"] - ss["brightness"]),
            "real_contrast": rs["contrast"],
            "sim_contrast": ss["contrast"],
            "contrast_abs_diff": abs(rs["contrast"] - ss["contrast"]),
            "rgb_mean_l2": float(np.linalg.norm(np.asarray(rs["rgb_mean"]) - np.asarray(ss["rgb_mean"]))),
            "rgb_std_l2": float(np.linalg.norm(np.asarray(rs["rgb_std"]) - np.asarray(ss["rgb_std"]))),
            "resized_mse": float(np.mean((real_resized - sim_resized) ** 2)),
            "resized_psnr": psnr(real_resized, sim_resized),
            "resized_luma_ssim_global": simple_ssim_luma(real_resized, sim_resized),
        }
        observation_rows.append(row)

    with (output_dir / "observation_gap.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(observation_rows[0].keys()))
        writer.writeheader()
        writer.writerows(observation_rows)

    token_summary: dict[str, Any] = {}
    token_rows: list[dict[str, Any]] = []
    for feature in FEATURES:
        real_stack = []
        sim_stack = []
        for frame in frames:
            real_tokens = load_tokens(Path(real_records[frame]["_feature_path"]), feature)
            sim_tokens = load_tokens(Path(sim_records[frame]["_feature_path"]), feature)
            real_stack.append(real_tokens)
            sim_stack.append(sim_tokens)
        real_arr = np.stack(real_stack, axis=0)
        sim_arr = np.stack(sim_stack, axis=0)
        if real_arr.shape != sim_arr.shape:
            raise ValueError(f"Shape mismatch for {feature}: {real_arr.shape} vs {sim_arr.shape}")
        diff = real_arr - sim_arr
        token_l2 = np.linalg.norm(diff, axis=2)
        token_cos = np.zeros(token_l2.shape, dtype=np.float64)
        for i in range(real_arr.shape[0]):
            for t in range(real_arr.shape[1]):
                token_cos[i, t] = cosine_distance(real_arr[i, t], sim_arr[i, t])
        real_token_var = real_arr.var(axis=(0, 2))
        sim_token_var = sim_arr.var(axis=(0, 2))
        for token_index in range(real_arr.shape[1]):
            token_rows.append(
                {
                    "feature": feature,
                    "token_index": token_index,
                    "token_l2_mean": float(token_l2[:, token_index].mean()),
                    "token_l2_std": float(token_l2[:, token_index].std()),
                    "token_cosine_mean": float(token_cos[:, token_index].mean()),
                    "token_cosine_std": float(token_cos[:, token_index].std()),
                    "real_token_variance": float(real_token_var[token_index]),
                    "sim_token_variance": float(sim_token_var[token_index]),
                    "variance_ratio_sim_over_real": float(sim_token_var[token_index] / max(real_token_var[token_index], 1e-12)),
                }
            )
        token_summary[feature] = {
            "shape": list(real_arr.shape),
            "token_count": int(real_arr.shape[1]),
            "feature_dim": int(real_arr.shape[2]),
            "token_l2_overall": summarize(token_l2.reshape(-1).tolist()),
            "token_cosine_overall": summarize(token_cos.reshape(-1).tolist()),
            "real_token_variance_mean": float(real_token_var.mean()),
            "sim_token_variance_mean": float(sim_token_var.mean()),
            "top_l2_tokens": sorted(
                [
                    {
                        "token_index": int(i),
                        "token_l2_mean": float(token_l2[:, i].mean()),
                        "token_cosine_mean": float(token_cos[:, i].mean()),
                    }
                    for i in range(real_arr.shape[1])
                ],
                key=lambda item: item["token_l2_mean"],
                reverse=True,
            )[:20],
        }

    with (output_dir / "token_gap_by_index.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(token_rows[0].keys()))
        writer.writeheader()
        writer.writerows(token_rows)

    summary = {
        "analysis_root": str(analysis_root),
        "output_dir": str(output_dir),
        "paired_frame_count": len(frames),
        "observation_gap": {
            key: summarize([float(row[key]) for row in observation_rows])
            for key in [
                "brightness_abs_diff",
                "contrast_abs_diff",
                "rgb_mean_l2",
                "rgb_std_l2",
                "resized_mse",
                "resized_psnr",
                "resized_luma_ssim_global",
            ]
        },
        "token_gap": token_summary,
        "notes": [
            "Image PSNR/SSIM are computed after resizing to 224x224 and include geometry/state mismatch.",
            "Do not interpret low image similarity as purely photometric gap.",
            "Token index is not assumed to map directly to a physical object.",
        ],
    }
    write_json(output_dir / "observation_token_gap_summary.json", summary)
    print(json.dumps(summary, indent=2)[:4000])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
