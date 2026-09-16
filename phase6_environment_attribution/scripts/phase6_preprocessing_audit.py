#!/usr/bin/env python3
from __future__ import annotations

import csv
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
from PIL import Image, ImageEnhance


ROOT = Path("/home/ubuntu/a0509_vla_linux_field_bundle_20260903")
PHASE6 = ROOT / "lhj" / "phase6_environment_attribution"
ANALYSIS = ROOT / "outputs" / "token_distribution_analysis" / "5_episodes"


@dataclass(frozen=True)
class Condition:
    name: str
    family: str
    description: str
    status: str = "OBSERVATION_ONLY_FULL_FORWARD_REQUIRED"


CONDITIONS = [
    Condition("P0_current_paired_image", "preprocessing", "Existing paired 1280x720 image."),
    Condition("P1_resize_224_direct", "preprocessing", "Direct resize to 224x224."),
    Condition("P2_center_crop_square_resize_224", "preprocessing", "Center square crop then resize to 224x224."),
    Condition("P3_center_crop_0p875_resize_224", "preprocessing", "Center crop 87.5% of min side then resize to 224x224."),
    Condition("P4_letterbox_224", "preprocessing", "Aspect-preserving resize with black letterbox to 224x224."),
    Condition("P5_brightness_match_sim_to_real", "photometric", "Shift Sim brightness mean toward Real mean."),
    Condition("P6_contrast_match_sim_to_real", "photometric", "Scale Sim contrast toward Real luminance std."),
]


def read_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def load_rgb(path: str | Path) -> Image.Image:
    return Image.open(path).convert("RGB")


def to_float(image: Image.Image) -> np.ndarray:
    return np.asarray(image.convert("RGB"), dtype=np.float64) / 255.0


def center_square(image: Image.Image, scale: float = 1.0) -> Image.Image:
    w, h = image.size
    side = int(min(w, h) * scale)
    left = (w - side) // 2
    top = (h - side) // 2
    return image.crop((left, top, left + side, top + side))


def letterbox(image: Image.Image, size: int = 224) -> Image.Image:
    image = image.copy()
    image.thumbnail((size, size), Image.Resampling.BICUBIC)
    canvas = Image.new("RGB", (size, size), (0, 0, 0))
    left = (size - image.size[0]) // 2
    top = (size - image.size[1]) // 2
    canvas.paste(image, (left, top))
    return canvas


def luminance(arr: np.ndarray) -> np.ndarray:
    return 0.2126 * arr[..., 0] + 0.7152 * arr[..., 1] + 0.0722 * arr[..., 2]


def transform_pair(condition: str, real: Image.Image, sim: Image.Image) -> tuple[Image.Image, Image.Image]:
    if condition == "P0_current_paired_image":
        return real, sim
    if condition == "P1_resize_224_direct":
        return (
            real.resize((224, 224), Image.Resampling.BICUBIC),
            sim.resize((224, 224), Image.Resampling.BICUBIC),
        )
    if condition == "P2_center_crop_square_resize_224":
        return (
            center_square(real).resize((224, 224), Image.Resampling.BICUBIC),
            center_square(sim).resize((224, 224), Image.Resampling.BICUBIC),
        )
    if condition == "P3_center_crop_0p875_resize_224":
        return (
            center_square(real, 0.875).resize((224, 224), Image.Resampling.BICUBIC),
            center_square(sim, 0.875).resize((224, 224), Image.Resampling.BICUBIC),
        )
    if condition == "P4_letterbox_224":
        return letterbox(real), letterbox(sim)
    if condition == "P5_brightness_match_sim_to_real":
        real_arr = to_float(real)
        sim_arr = to_float(sim)
        gain = float(np.mean(luminance(real_arr)) / max(np.mean(luminance(sim_arr)), 1e-6))
        return real, ImageEnhance.Brightness(sim).enhance(gain)
    if condition == "P6_contrast_match_sim_to_real":
        real_arr = to_float(real)
        sim_arr = to_float(sim)
        gain = float(np.std(luminance(real_arr)) / max(np.std(luminance(sim_arr)), 1e-6))
        return real, ImageEnhance.Contrast(sim).enhance(gain)
    raise KeyError(condition)


def psnr(mse: float) -> float:
    if mse <= 1e-12:
        return float("inf")
    return float(10.0 * math.log10(1.0 / mse))


def ssim_simple(x: np.ndarray, y: np.ndarray) -> float:
    # Global grayscale SSIM. This is intentionally lightweight and used only for
    # observation-side screening, not as final policy evidence.
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


def pair_metrics(real_img: Image.Image, sim_img: Image.Image) -> dict[str, float]:
    real = to_float(real_img)
    sim = to_float(sim_img)
    if real.shape != sim.shape:
        sim_img = sim_img.resize(real_img.size, Image.Resampling.BICUBIC)
        sim = to_float(sim_img)
    diff = real - sim
    mse = float(np.mean(diff**2))
    rgb_l2 = float(np.linalg.norm(diff.reshape(-1, 3), axis=1).mean())
    real_l = luminance(real)
    sim_l = luminance(sim)
    return {
        "rgb_l2_mean": rgb_l2,
        "mse": mse,
        "psnr": psnr(mse),
        "ssim_global": ssim_simple(real, sim),
        "real_brightness": float(real_l.mean()),
        "sim_brightness": float(sim_l.mean()),
        "brightness_abs_diff": float(abs(real_l.mean() - sim_l.mean())),
        "real_luma_std": float(real_l.std()),
        "sim_luma_std": float(sim_l.std()),
        "luma_std_abs_diff": float(abs(real_l.std() - sim_l.std())),
    }


def mean(rows: list[dict[str, Any]], key: str) -> float:
    values = [float(row[key]) for row in rows if row.get(key) not in (None, "")]
    return float(np.mean(values)) if values else float("nan")


def main() -> int:
    output_dir = PHASE6
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "01_preprocessing").mkdir(exist_ok=True)
    (output_dir / "02_photometric").mkdir(exist_ok=True)
    (output_dir / "tables").mkdir(exist_ok=True)

    manifest = read_json(ANALYSIS / "paired_inputs" / "paired_manifest.json")
    pairs = manifest["pairs"]
    rows: list[dict[str, Any]] = []
    for pair in pairs:
        real_base = load_rgb(pair["real"]["paired_image"])
        sim_base = load_rgb(pair["sim"]["paired_image"])
        for condition in CONDITIONS:
            real_img, sim_img = transform_pair(condition.name, real_base, sim_base)
            metrics = pair_metrics(real_img, sim_img)
            rows.append(
                {
                    "pair_id": pair["pair_id"],
                    "episode_id": pair["episode_id"],
                    "step_index": pair["step_index"],
                    "planner_phase": pair.get("planner_phase", ""),
                    "condition": condition.name,
                    "factor": condition.family,
                    "description": condition.description,
                    **metrics,
                    "vision_gap": "",
                    "projector_gap": "",
                    "hidden_gap": "",
                    "sensitive_energy": "",
                    "null_energy": "",
                    "sensitive_ratio": "",
                    "action_gap": "",
                    "translation_gap": "",
                    "rotation_gap": "",
                    "gripper_gap": "",
                    "status": condition.status,
                }
            )

    fieldnames = list(rows[0].keys())
    for path in [
        output_dir / "01_preprocessing" / "preprocessing_observation_frame_metrics.csv",
        output_dir / "preprocessing_ablation.csv",
    ]:
        with path.open("w", encoding="utf-8", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(rows)

    summary_rows: list[dict[str, Any]] = []
    for condition in CONDITIONS:
        items = [row for row in rows if row["condition"] == condition.name]
        summary_rows.append(
            {
                "factor": condition.family,
                "condition": condition.name,
                "count": len(items),
                "observation_rgb_l2_mean": mean(items, "rgb_l2_mean"),
                "observation_mse": mean(items, "mse"),
                "observation_psnr": mean(items, "psnr"),
                "observation_ssim_global": mean(items, "ssim_global"),
                "brightness_abs_diff": mean(items, "brightness_abs_diff"),
                "luma_std_abs_diff": mean(items, "luma_std_abs_diff"),
                "vision_gap": "",
                "projector_gap": "",
                "hidden_gap": "",
                "sensitive_energy": "",
                "null_energy": "",
                "sensitive_ratio": "",
                "action_gap": "",
                "translation_gap": "",
                "rotation_gap": "",
                "gripper_gap": "",
                "policy_impact_score": "",
                "policy_efficiency": "",
                "status": condition.status,
            }
        )
    summary_fields = list(summary_rows[0].keys())
    for path in [
        output_dir / "01_preprocessing" / "preprocessing_observation_summary.csv",
        output_dir / "environment_factor_table.csv",
        output_dir / "tables" / "environment_factor_table.csv",
    ]:
        with path.open("w", encoding="utf-8", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=summary_fields)
            writer.writeheader()
            writer.writerows(summary_rows)

    payload = {
        "analysis_root": str(ANALYSIS),
        "pair_count": len(pairs),
        "conditions": [condition.__dict__ for condition in CONDITIONS],
        "outputs": {
            "frame_metrics": str(output_dir / "preprocessing_ablation.csv"),
            "summary": str(output_dir / "environment_factor_table.csv"),
        },
        "status": "OBSERVATION_ONLY_FULL_FORWARD_REQUIRED",
        "reason": "Current runtime has no CUDA-capable device; OFTRuntime raises `CUDA GPU is required` before model load.",
    }
    write_json(output_dir / "experiment_manifest.json", payload)
    print(json.dumps(payload, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
