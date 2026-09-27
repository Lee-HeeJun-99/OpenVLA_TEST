#!/usr/bin/env python3
"""Compute Phase 8 observation gap metrics from aligned image pairs."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from statistics import mean, median, pstdev
from typing import Any

import cv2
import numpy as np
from skimage.metrics import structural_similarity


DEFAULT_BUNDLE = Path("/home/ubuntu/a0509_vla_linux_field_bundle_20260903")
PHASE_ORDER = ["hold", "alignment", "descent_to_grasp", "grasp_close", "lift"]


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open("r", encoding="utf-8", newline="") as handle:
        return list(csv.DictReader(handle))


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    keys = list(rows[0].keys())
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=keys)
        writer.writeheader()
        writer.writerows(rows)


def imread_rgb(path: str) -> np.ndarray:
    bgr = cv2.imread(path, cv2.IMREAD_COLOR)
    if bgr is None:
        raise FileNotFoundError(path)
    return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)


def resize_to_match(a: np.ndarray, b: np.ndarray) -> tuple[np.ndarray, np.ndarray, str]:
    if a.shape == b.shape:
        return a, b, "none"
    # Preserve left/reference resolution and resize right.
    h, w = a.shape[:2]
    b2 = cv2.resize(b, (w, h), interpolation=cv2.INTER_AREA)
    return a, b2, f"right_resized_to_{w}x{h}"


def to_gray(img: np.ndarray) -> np.ndarray:
    return cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)


def hue_sat_val(img: np.ndarray) -> np.ndarray:
    return cv2.cvtColor(img, cv2.COLOR_RGB2HSV)


def hist_distance(a: np.ndarray, b: np.ndarray, space: str = "rgb") -> float:
    if space == "hsv":
        a_img = hue_sat_val(a)
        b_img = hue_sat_val(b)
        ranges = [0, 180, 0, 256, 0, 256]
    else:
        a_img = a
        b_img = b
        ranges = [0, 256, 0, 256, 0, 256]
    hist_a = cv2.calcHist([a_img], [0, 1, 2], None, [16, 16, 16], ranges)
    hist_b = cv2.calcHist([b_img], [0, 1, 2], None, [16, 16, 16], ranges)
    hist_a = cv2.normalize(hist_a, hist_a).flatten()
    hist_b = cv2.normalize(hist_b, hist_b).flatten()
    return float(cv2.compareHist(hist_a.astype(np.float32), hist_b.astype(np.float32), cv2.HISTCMP_BHATTACHARYYA))


def estimate_shift(gray_a: np.ndarray, gray_b: np.ndarray) -> tuple[float, float, float]:
    a = gray_a.astype(np.float32)
    b = gray_b.astype(np.float32)
    if a.shape != b.shape:
        b = cv2.resize(b, (a.shape[1], a.shape[0]), interpolation=cv2.INTER_AREA)
    shift, response = cv2.phaseCorrelate(a, b)
    return float(shift[0]), float(shift[1]), float(response)


def metrics_for_pair(left_img: np.ndarray, right_img: np.ndarray) -> dict[str, Any]:
    left_img, right_img, resize_note = resize_to_match(left_img, right_img)
    a = left_img.astype(np.float32) / 255.0
    b = right_img.astype(np.float32) / 255.0
    diff = a - b
    abs_diff = np.abs(diff)
    sq = diff * diff

    gray_a = to_gray(left_img)
    gray_b = to_gray(right_img)
    edge_a = cv2.Canny(gray_a, 80, 160)
    edge_b = cv2.Canny(gray_b, 80, 160)
    edge_abs = np.abs(edge_a.astype(np.float32) - edge_b.astype(np.float32)) / 255.0

    hsv_a = hue_sat_val(left_img).astype(np.float32)
    hsv_b = hue_sat_val(right_img).astype(np.float32)
    mse = float(np.mean(sq))
    psnr = float("inf") if mse <= 1e-12 else float(10.0 * math.log10(1.0 / mse))
    try:
        ssim = float(structural_similarity(gray_a, gray_b, data_range=255))
    except Exception:
        ssim = float("nan")
    shift_x, shift_y, shift_response = estimate_shift(gray_a, gray_b)
    return {
        "resize_note": resize_note,
        "height": int(left_img.shape[0]),
        "width": int(left_img.shape[1]),
        "pixel_l1_mean": float(np.mean(abs_diff)),
        "pixel_l2_rmse": float(math.sqrt(mse)),
        "pixel_mse": mse,
        "pixel_psnr": psnr,
        "ssim_gray": ssim,
        "rgb_mean_abs_diff": float(np.mean(np.abs(np.mean(a, axis=(0, 1)) - np.mean(b, axis=(0, 1))))),
        "r_mean_diff": float(np.mean(a[:, :, 0]) - np.mean(b[:, :, 0])),
        "g_mean_diff": float(np.mean(a[:, :, 1]) - np.mean(b[:, :, 1])),
        "b_mean_diff": float(np.mean(a[:, :, 2]) - np.mean(b[:, :, 2])),
        "brightness_diff": float(np.mean(gray_a) / 255.0 - np.mean(gray_b) / 255.0),
        "contrast_diff": float(np.std(gray_a) / 255.0 - np.std(gray_b) / 255.0),
        "saturation_diff": float(np.mean(hsv_a[:, :, 1]) / 255.0 - np.mean(hsv_b[:, :, 1]) / 255.0),
        "value_diff": float(np.mean(hsv_a[:, :, 2]) / 255.0 - np.mean(hsv_b[:, :, 2]) / 255.0),
        "rgb_hist_bhattacharyya": hist_distance(left_img, right_img, "rgb"),
        "hsv_hist_bhattacharyya": hist_distance(left_img, right_img, "hsv"),
        "edge_l1_mean": float(np.mean(edge_abs)),
        "edge_density_left": float(np.mean(edge_a > 0)),
        "edge_density_right": float(np.mean(edge_b > 0)),
        "estimated_shift_x_px": shift_x,
        "estimated_shift_y_px": shift_y,
        "estimated_shift_response": shift_response,
    }


def boolish(value: str) -> bool:
    return str(value).lower() in {"true", "1", "yes"}


def finite_values(rows: list[dict[str, Any]], key: str) -> list[float]:
    vals = []
    for row in rows:
        value = row.get(key)
        try:
            f = float(value)
        except Exception:
            continue
        if math.isfinite(f):
            vals.append(f)
    return vals


def aggregate(rows: list[dict[str, Any]], group_keys: list[str], metric_keys: list[str]) -> list[dict[str, Any]]:
    groups: dict[tuple[Any, ...], list[dict[str, Any]]] = {}
    for row in rows:
        key = tuple(row.get(k) for k in group_keys)
        groups.setdefault(key, []).append(row)
    out: list[dict[str, Any]] = []
    for key, subset in sorted(groups.items(), key=lambda item: tuple(str(x) for x in item[0])):
        agg = {k: v for k, v in zip(group_keys, key)}
        agg["count"] = len(subset)
        for metric in metric_keys:
            vals = finite_values(subset, metric)
            if vals:
                agg[f"{metric}_mean"] = mean(vals)
                agg[f"{metric}_median"] = median(vals)
                agg[f"{metric}_std"] = pstdev(vals) if len(vals) > 1 else 0.0
                agg[f"{metric}_min"] = min(vals)
                agg[f"{metric}_max"] = max(vals)
            else:
                agg[f"{metric}_mean"] = None
                agg[f"{metric}_median"] = None
                agg[f"{metric}_std"] = None
                agg[f"{metric}_min"] = None
                agg[f"{metric}_max"] = None
        out.append(agg)
    return out


def make_triplet_figure(left_path: str, right_path: str, output_path: Path, title: str) -> None:
    left = imread_rgb(left_path)
    right = imread_rgb(right_path)
    left, right, _ = resize_to_match(left, right)
    diff = np.abs(left.astype(np.int16) - right.astype(np.int16)).astype(np.uint8)
    # boost diff visibility without changing stored metrics
    diff_vis = np.clip(diff.astype(np.float32) * 3.0, 0, 255).astype(np.uint8)
    h, w = left.shape[:2]
    header = np.full((42, w * 3, 3), 255, dtype=np.uint8)
    canvas = np.concatenate([left, right, diff_vis], axis=1)
    canvas = np.concatenate([header, canvas], axis=0)
    bgr = cv2.cvtColor(canvas, cv2.COLOR_RGB2BGR)
    cv2.putText(bgr, title[:110], (12, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 0), 2, cv2.LINE_AA)
    cv2.putText(bgr, "left", (12, 62), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.putText(bgr, "right", (w + 12, 62), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.putText(bgr, "abs diff x3", (2 * w + 12, 62), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2, cv2.LINE_AA)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(output_path), bgr)


def write_report(path: Path, summary_rows: list[dict[str, Any]], phase_rows: list[dict[str, Any]]) -> None:
    metric_cols = [
        "pixel_l1_mean_mean",
        "pixel_l2_rmse_mean",
        "ssim_gray_mean",
        "brightness_diff_mean",
        "contrast_diff_mean",
        "saturation_diff_mean",
        "edge_l1_mean_mean",
        "estimated_shift_x_px_mean",
        "estimated_shift_y_px_mean",
    ]
    lines = [
        "# Phase 8 Observation Gap Report",
        "",
        "## Method",
        "",
        "Metrics were computed on `pair_valid=true` rows from `03_pair_alignment/aligned_pairs.csv`.",
        "Images were compared in RGB after resizing the right image only if dimensions differed.",
        "",
        "## Comparison Summary",
        "",
        "| Comparison | N | L1 | RMSE | SSIM | Brightness diff | Contrast diff | Saturation diff | Edge L1 | Shift x | Shift y |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for row in summary_rows:
        vals = [row.get(c) for c in metric_cols]
        lines.append(
            f"| {row['comparison_id']} | {row['count']} | "
            + " | ".join("n/a" if v is None else f"{float(v):.6f}" for v in vals)
            + " |"
        )
    lines.extend([
        "",
        "## Interpretation Guardrails",
        "",
        "- These are observation-level image metrics only.",
        "- A larger image gap is not automatically a larger policy-relevant gap.",
        "- The next step must compare representation/action-facing features before making policy-impact claims.",
        "",
        "## Phase Summary",
        "",
        "See `phase_summary.csv` for phase-conditioned metrics.",
    ])
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--phase8-root", type=Path, default=DEFAULT_BUNDLE / "lhj" / "phase8_shadow_mode_distribution_analysis")
    args = parser.parse_args()

    phase8 = args.phase8_root.expanduser().resolve()
    pair_path = phase8 / "03_pair_alignment" / "aligned_pairs.csv"
    out_dir = phase8 / "05_observation_gap"
    fig_dir = out_dir / "figures"
    out_dir.mkdir(parents=True, exist_ok=True)
    fig_dir.mkdir(parents=True, exist_ok=True)

    pairs = read_csv(pair_path)
    valid_pairs = [row for row in pairs if boolish(row.get("pair_valid", ""))]
    frame_rows: list[dict[str, Any]] = []
    for row in valid_pairs:
        left_path = row["left_image_path"]
        right_path = row["right_image_path"]
        metrics = metrics_for_pair(imread_rgb(left_path), imread_rgb(right_path))
        frame_rows.append({
            "comparison_id": row["comparison_id"],
            "left_domain": row["left_domain"],
            "left_episode": row["left_episode"],
            "left_condition": row["left_condition"],
            "left_frame": row["left_frame"],
            "right_domain": row["right_domain"],
            "right_episode": row["right_episode"],
            "right_condition": row["right_condition"],
            "right_frame": row["right_frame"],
            "phase": row["left_phase"],
            "time_difference": row["time_difference"],
            "ee_translation_error_m": row["ee_translation_error_m"],
            "left_image_path": left_path,
            "right_image_path": right_path,
            **metrics,
        })

    metric_keys = [
        "pixel_l1_mean",
        "pixel_l2_rmse",
        "pixel_mse",
        "pixel_psnr",
        "ssim_gray",
        "rgb_mean_abs_diff",
        "brightness_diff",
        "contrast_diff",
        "saturation_diff",
        "value_diff",
        "rgb_hist_bhattacharyya",
        "hsv_hist_bhattacharyya",
        "edge_l1_mean",
        "edge_density_left",
        "edge_density_right",
        "estimated_shift_x_px",
        "estimated_shift_y_px",
        "estimated_shift_response",
    ]
    summary_rows = aggregate(frame_rows, ["comparison_id"], metric_keys)
    phase_rows = aggregate(frame_rows, ["comparison_id", "phase"], metric_keys)

    write_csv(out_dir / "frame_metrics.csv", frame_rows)
    write_csv(out_dir / "comparison_summary.csv", summary_rows)
    write_csv(out_dir / "phase_summary.csv", phase_rows)

    # Representative figures: frames near start, grasp close, lift for each comparison.
    representative_frames = [0, 18, 27, 37, 44]
    made_figures: list[str] = []
    by_comparison: dict[str, list[dict[str, Any]]] = {}
    for row in frame_rows:
        by_comparison.setdefault(row["comparison_id"], []).append(row)
    for comparison_id, rows in by_comparison.items():
        rows_by_frame = {int(r["left_frame"]): r for r in rows}
        for frame in representative_frames:
            r = rows_by_frame.get(frame)
            if r is None:
                continue
            out_path = fig_dir / f"{comparison_id}_frame{frame:06d}.jpg"
            make_triplet_figure(
                r["left_image_path"],
                r["right_image_path"],
                out_path,
                f"{comparison_id} frame {frame} phase={r['phase']}",
            )
            made_figures.append(str(out_path))

    summary = {
        "status": "COMPLETED",
        "pair_source": str(pair_path),
        "valid_pair_count": len(valid_pairs),
        "frame_metric_rows": len(frame_rows),
        "comparison_count": len(summary_rows),
        "phase_summary_rows": len(phase_rows),
        "figures": made_figures,
        "outputs": {
            "frame_metrics": str(out_dir / "frame_metrics.csv"),
            "comparison_summary": str(out_dir / "comparison_summary.csv"),
            "phase_summary": str(out_dir / "phase_summary.csv"),
            "report": str(out_dir / "observation_gap_report.md"),
        },
        "interpretation": "Observation metrics only; policy impact requires representation/action analysis.",
    }
    (out_dir / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    write_report(out_dir / "observation_gap_report.md", summary_rows, phase_rows)
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
