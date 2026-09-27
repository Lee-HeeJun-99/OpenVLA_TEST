#!/usr/bin/env python3
"""Aggregate Phase 8 representation/action gaps after feature extraction."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from statistics import mean, median, pstdev
from typing import Any

import numpy as np


DEFAULT_BUNDLE = Path("/home/ubuntu/a0509_vla_linux_field_bundle_20260903")
FEATURE_KEYS = [
    "vision_backbone.output",
    "projector.output",
    "action_hidden_states.input",
    "action_head.output",
]


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


def read_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def frame_id_from_path(path: str) -> int:
    return int(Path(path).stem)


def episode_id_from_path(path: str) -> str | None:
    parts = Path(path).parts
    for part in parts:
        if part.startswith("episode_"):
            return part
    return None


def load_records(manifest_path: Path, domain: str, output_root: Path) -> dict[tuple[str, int], dict[str, Any]]:
    manifest = read_json(manifest_path)
    records: dict[tuple[str, int], dict[str, Any]] = {}
    for record in manifest["records"]:
        source_image = record["source_image"]
        episode_id = episode_id_from_path(source_image)
        if episode_id is None and domain == "sim":
            episode_id = "episode_000004"
        if episode_id is None:
            raise ValueError(f"Could not infer episode id from {source_image}")
        frame = frame_id_from_path(source_image)
        feature_file = output_root / record["feature_file"]
        records[(episode_id, frame)] = {**record, "_feature_path": str(feature_file)}
    return records


def load_feature(record: dict[str, Any], key: str) -> np.ndarray:
    with np.load(record["_feature_path"], allow_pickle=True) as data:
        if key not in data:
            raise KeyError(f"{key} missing from {record['_feature_path']}")
        return np.asarray(data[key], dtype=np.float64)


def pooled(arr: np.ndarray) -> np.ndarray:
    if arr.ndim == 0:
        return arr.reshape(1)
    if arr.ndim == 1:
        return arr
    if arr.ndim == 2:
        return arr.mean(axis=0)
    if arr.ndim == 3:
        return arr.reshape(-1, arr.shape[-1]).mean(axis=0)
    return arr.reshape(-1)


def token_mean_l2(a: np.ndarray, b: np.ndarray) -> float | None:
    aa = a
    bb = b
    if aa.ndim == 3 and aa.shape[0] == 1:
        aa = aa[0]
    if bb.ndim == 3 and bb.shape[0] == 1:
        bb = bb[0]
    if aa.ndim != 2 or bb.ndim != 2 or aa.shape != bb.shape:
        return None
    return float(np.linalg.norm(aa - bb, axis=1).mean())


def l2(a: np.ndarray, b: np.ndarray) -> float:
    return float(np.linalg.norm(a.reshape(-1) - b.reshape(-1)))


def cosine_distance(a: np.ndarray, b: np.ndarray) -> float:
    aa = a.reshape(-1)
    bb = b.reshape(-1)
    denom = float(np.linalg.norm(aa) * np.linalg.norm(bb))
    if denom <= 1e-12:
        return 1.0
    return float(1.0 - np.dot(aa, bb) / denom)


def mmd_rbf(x: np.ndarray, y: np.ndarray) -> float:
    if len(x) == 0 or len(y) == 0:
        return float("nan")
    z = np.concatenate([x, y], axis=0)
    # Median heuristic on a capped pair sample for stability.
    diff = z[:, None, :] - z[None, :, :]
    d2 = np.sum(diff * diff, axis=-1)
    vals = d2[np.triu_indices_from(d2, k=1)]
    finite = vals[np.isfinite(vals) & (vals > 0)]
    gamma = 1.0 / float(np.median(finite)) if finite.size else 1.0
    kxx = np.exp(-gamma * np.sum((x[:, None, :] - x[None, :, :]) ** 2, axis=-1)).mean()
    kyy = np.exp(-gamma * np.sum((y[:, None, :] - y[None, :, :]) ** 2, axis=-1)).mean()
    kxy = np.exp(-gamma * np.sum((x[:, None, :] - y[None, :, :]) ** 2, axis=-1)).mean()
    return float(kxx + kyy - 2.0 * kxy)


def aggregate(rows: list[dict[str, Any]], group_keys: list[str], metric_keys: list[str]) -> list[dict[str, Any]]:
    groups: dict[tuple[Any, ...], list[dict[str, Any]]] = {}
    for row in rows:
        key = tuple(row.get(k) for k in group_keys)
        groups.setdefault(key, []).append(row)
    out: list[dict[str, Any]] = []
    for key, subset in sorted(groups.items(), key=lambda item: tuple(str(x) for x in item[0])):
        result = {k: v for k, v in zip(group_keys, key)}
        result["count"] = len(subset)
        for metric in metric_keys:
            vals = []
            for row in subset:
                value = row.get(metric)
                try:
                    value = float(value)
                except Exception:
                    continue
                if math.isfinite(value):
                    vals.append(value)
            if vals:
                result[f"{metric}_mean"] = mean(vals)
                result[f"{metric}_median"] = median(vals)
                result[f"{metric}_std"] = pstdev(vals) if len(vals) > 1 else 0.0
                result[f"{metric}_min"] = min(vals)
                result[f"{metric}_max"] = max(vals)
            else:
                result[f"{metric}_mean"] = None
                result[f"{metric}_median"] = None
                result[f"{metric}_std"] = None
                result[f"{metric}_min"] = None
                result[f"{metric}_max"] = None
        out.append(result)
    return out


def write_blocked(out_dir: Path, missing: list[str]) -> int:
    out_dir.mkdir(parents=True, exist_ok=True)
    payload = {
        "status": "BLOCKED_MISSING_FEATURES",
        "missing": missing,
        "required_action": "Run 06_representation_gap/run_phase8_feature_extraction.sh on a GPU-enabled environment, then rerun phase8_representation_gap.py.",
    }
    (out_dir / "summary.json").write_text(json.dumps(payload, indent=2), encoding="utf-8")
    report = [
        "# Phase 8 Representation Gap",
        "",
        "Status: `BLOCKED_MISSING_FEATURES`",
        "",
        "Missing feature manifests:",
        "",
        *[f"- `{item}`" for item in missing],
        "",
        "Run:",
        "",
        "```bash",
        "/home/ubuntu/a0509_vla_linux_field_bundle_20260903/lhj/phase8_shadow_mode_distribution_analysis/06_representation_gap/run_phase8_feature_extraction.sh",
        "/usr/bin/python /home/ubuntu/a0509_vla_linux_field_bundle_20260903/lhj/phase8_shadow_mode_distribution_analysis/scripts/phase8_representation_gap.py",
        "```",
    ]
    (out_dir / "representation_gap_report.md").write_text("\n".join(report) + "\n", encoding="utf-8")
    print(json.dumps(payload, indent=2))
    return 0


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--phase8-root", type=Path, default=DEFAULT_BUNDLE / "lhj" / "phase8_shadow_mode_distribution_analysis")
    args = parser.parse_args()
    phase8 = args.phase8_root.expanduser().resolve()
    out_dir = phase8 / "06_representation_gap"
    feature_root = out_dir / "full_forward_features"
    real_manifest = feature_root / "real" / "feature_manifest.json"
    sim_manifest = feature_root / "sim_episode_000004_reference" / "feature_manifest.json"
    missing = [str(p) for p in [real_manifest, sim_manifest] if not p.exists()]
    if missing:
        return write_blocked(out_dir, missing)

    pairs = [row for row in read_csv(phase8 / "03_pair_alignment" / "aligned_pairs.csv") if row["pair_valid"].lower() == "true"]
    real_records = load_records(real_manifest, "real", real_manifest.parent)
    sim_records = load_records(sim_manifest, "sim", sim_manifest.parent)

    def record_for(domain: str, episode_id: str, frame: int) -> dict[str, Any]:
        if domain == "sim":
            return sim_records[("episode_000004", frame)]
        return real_records[(episode_id, frame)]

    frame_rows: list[dict[str, Any]] = []
    pooled_by_comparison: dict[tuple[str, str, str], dict[str, list[np.ndarray]]] = {}
    for pair in pairs:
        comparison = pair["comparison_id"]
        left = record_for(pair["left_domain"], pair["left_episode"], int(pair["left_frame"]))
        right = record_for(pair["right_domain"], pair["right_episode"], int(pair["right_frame"]))
        row: dict[str, Any] = {
            "comparison_id": comparison,
            "phase": pair["left_phase"],
            "left_episode": pair["left_episode"],
            "left_frame": pair["left_frame"],
            "right_episode": pair["right_episode"],
            "right_frame": pair["right_frame"],
        }
        for key in FEATURE_KEYS:
            lf = load_feature(left, key)
            rf = load_feature(right, key)
            lp = pooled(lf)
            rp = pooled(rf)
            safe = key.replace(".", "_")
            row[f"{safe}_pooled_l2"] = l2(lp, rp)
            row[f"{safe}_pooled_cosine_distance"] = cosine_distance(lp, rp)
            row[f"{safe}_full_l2"] = l2(lf, rf)
            tml2 = token_mean_l2(lf, rf)
            row[f"{safe}_token_mean_l2"] = tml2
            pooled_by_comparison.setdefault((comparison, key, "left"), {}).setdefault("values", []).append(lp)
            pooled_by_comparison.setdefault((comparison, key, "right"), {}).setdefault("values", []).append(rp)
        frame_rows.append(row)

    metric_keys = [k for k in frame_rows[0].keys() if k not in {"comparison_id", "phase", "left_episode", "left_frame", "right_episode", "right_frame"}]
    comparison_summary = aggregate(frame_rows, ["comparison_id"], metric_keys)
    phase_summary = aggregate(frame_rows, ["comparison_id", "phase"], metric_keys)

    # Distribution MMD on pooled vectors per feature/comparison.
    mmd_rows = []
    for comparison in sorted({r["comparison_id"] for r in frame_rows}):
        for key in FEATURE_KEYS:
            left_vals = np.stack(pooled_by_comparison[(comparison, key, "left")]["values"], axis=0)
            right_vals = np.stack(pooled_by_comparison[(comparison, key, "right")]["values"], axis=0)
            mmd_rows.append({
                "comparison_id": comparison,
                "feature": key,
                "pooled_mmd_rbf": mmd_rbf(left_vals, right_vals),
                "left_count": len(left_vals),
                "right_count": len(right_vals),
            })

    write_csv(out_dir / "frame_representation_metrics.csv", frame_rows)
    write_csv(out_dir / "comparison_summary.csv", comparison_summary)
    write_csv(out_dir / "phase_summary.csv", phase_summary)
    write_csv(out_dir / "distribution_mmd.csv", mmd_rows)
    summary = {
        "status": "COMPLETED",
        "frame_metric_rows": len(frame_rows),
        "comparison_count": len(comparison_summary),
        "features": FEATURE_KEYS,
        "outputs": {
            "frame_metrics": str(out_dir / "frame_representation_metrics.csv"),
            "comparison_summary": str(out_dir / "comparison_summary.csv"),
            "phase_summary": str(out_dir / "phase_summary.csv"),
            "distribution_mmd": str(out_dir / "distribution_mmd.csv"),
        },
        "interpretation": "Representation metrics only. Policy impact requires action-specific analysis.",
    }
    (out_dir / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    report = [
        "# Phase 8 Representation Gap",
        "",
        "Status: `COMPLETED`",
        "",
        "Computed paired representation metrics for:",
        "",
        *[f"- `{key}`" for key in FEATURE_KEYS],
        "",
        "See CSV outputs for comparison and phase summaries.",
    ]
    (out_dir / "representation_gap_report.md").write_text("\n".join(report) + "\n", encoding="utf-8")
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
