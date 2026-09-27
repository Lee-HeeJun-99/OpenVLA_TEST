#!/usr/bin/env python3
from __future__ import annotations

import csv
import json
from collections import defaultdict
from pathlib import Path
from typing import Any

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "10_policy_sensitive_projection"
FEATURE_ROOT = ROOT / "06_representation_gap" / "full_forward_features"
BASIS_FILE = (
    Path("/home/ubuntu/a0509_vla_linux_field_bundle_20260903")
    / "lhj"
    / "phase6_environment_attribution"
    / "06_policy_relevance"
    / "sensitive_energy"
    / "phase6_p0_sensitive_basis_k128.npz"
)


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as fp:
        return list(csv.DictReader(fp))


def write_csv(path: Path, rows: list[dict[str, Any]], fields: list[str] | None = None) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows and fields is None:
        path.write_text("", encoding="utf-8")
        return
    if fields is None:
        fields = list(rows[0].keys())
    with path.open("w", newline="", encoding="utf-8") as fp:
        writer = csv.DictWriter(fp, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")


def f(row: dict[str, str], key: str, default: float = 0.0) -> float:
    value = row.get(key, "")
    if value == "" or value is None:
        return default
    return float(value)


def load_manifest(domain_dir: Path) -> dict[str, Path]:
    manifest_path = domain_dir / "feature_manifest.json"
    payload = json.loads(manifest_path.read_text(encoding="utf-8"))
    records = payload["records"]
    out: dict[str, Path] = {}
    for rec in records:
        image = str(Path(rec["source_image"]).resolve())
        feature_file = Path(rec["feature_file"])
        if not feature_file.is_absolute():
            feature_file = domain_dir / feature_file
        out[image] = feature_file.resolve()
    return out


def hidden(path: Path) -> np.ndarray:
    with np.load(path, allow_pickle=True) as z:
        return z["action_hidden_states.input"].reshape(-1).astype(np.float64)


def projection_energy(delta: np.ndarray, basis: np.ndarray) -> dict[str, float]:
    total = float(np.linalg.norm(delta))
    if total <= 1e-12 or basis.size == 0:
        return {
            "hidden_total_energy": total,
            "hidden_sensitive_energy": 0.0,
            "hidden_null_energy": total,
            "hidden_sensitive_ratio": 0.0,
        }
    coeff = basis @ delta
    sensitive = float(np.linalg.norm(coeff))
    null = float(max(total * total - sensitive * sensitive, 0.0) ** 0.5)
    return {
        "hidden_total_energy": total,
        "hidden_sensitive_energy": sensitive,
        "hidden_null_energy": null,
        "hidden_sensitive_ratio": sensitive / total,
    }


def mean(values: list[float]) -> float:
    return float(sum(values) / len(values)) if values else 0.0


def percentile(values: list[float], q: float) -> float:
    if not values:
        return 0.0
    xs = sorted(values)
    pos = (len(xs) - 1) * q / 100.0
    lo = int(pos)
    hi = min(lo + 1, len(xs) - 1)
    frac = pos - lo
    return float(xs[lo] * (1 - frac) + xs[hi] * frac)


def aggregate(rows: list[dict[str, Any]], keys: list[str]) -> list[dict[str, Any]]:
    grouped: dict[tuple[Any, ...], list[dict[str, Any]]] = defaultdict(list)
    for row in rows:
        grouped[tuple(row[key] for key in keys)].append(row)
    metrics = [
        "hidden_total_energy",
        "hidden_sensitive_energy",
        "hidden_null_energy",
        "hidden_sensitive_ratio",
        "chunk_mean_l2",
        "chunk_mean_translation_l2",
        "chunk_mean_rotation_l2",
        "chunk_mean_gripper_abs",
        "gripper_disagreement_rate",
    ]
    out = []
    for group_key, items in sorted(grouped.items(), key=lambda kv: kv[0]):
        rec = {key: value for key, value in zip(keys, group_key)}
        rec["count"] = len(items)
        for metric in metrics:
            vals = [float(item[metric]) for item in items]
            rec[f"{metric}_mean"] = mean(vals)
            rec[f"{metric}_median"] = percentile(vals, 50)
            rec[f"{metric}_p90"] = percentile(vals, 90)
        out.append(rec)
    return out


def build_report(overall: list[dict[str, Any]], by_phase: list[dict[str, Any]]) -> str:
    def fmt(x: Any) -> str:
        return f"{float(x):.6f}" if isinstance(x, (float, int)) else str(x)

    def table(rows: list[dict[str, Any]], fields: list[tuple[str, str]]) -> str:
        lines = [
            "| " + " | ".join(label for _, label in fields) + " |",
            "| " + " | ".join("---" for _ in fields) + " |",
        ]
        for row in rows:
            lines.append("| " + " | ".join(fmt(row[k]) for k, _ in fields) + " |")
        return "\n".join(lines)

    overall_fields = [
        ("comparison_id", "Comparison"),
        ("count", "N"),
        ("hidden_total_energy_mean", "Total"),
        ("hidden_sensitive_energy_mean", "Sensitive"),
        ("hidden_null_energy_mean", "Null"),
        ("hidden_sensitive_ratio_mean", "Ratio"),
        ("chunk_mean_l2_mean", "Action L2"),
        ("chunk_mean_gripper_abs_mean", "Gripper"),
    ]
    phase_fields = [
        ("comparison_id", "Comparison"),
        ("phase", "Phase"),
        ("count", "N"),
        ("hidden_sensitive_ratio_mean", "Ratio"),
        ("hidden_sensitive_energy_mean", "Sensitive"),
        ("chunk_mean_l2_mean", "Action L2"),
        ("chunk_mean_gripper_abs_mean", "Gripper"),
    ]
    real = [r for r in overall if str(r["comparison_id"]).startswith("real")]
    phase_real = [r for r in by_phase if str(r["comparison_id"]).startswith("real")]
    return f"""# Phase 8 Policy-Sensitive Projection

## Purpose

This analysis projects Phase 8 `action_hidden_states.input` differences onto the Phase 6 P0-derived policy-sensitive low-rank basis (`k=128`).

It tests whether each environment change moves hidden states mostly in the previously identified policy-sensitive subspace or mostly in the residual/null component.

## Important Scope

- Basis source: Phase 6 `phase6_p0_sensitive_basis_k128.npz`.
- Dataset: Phase 8 episodes 4, 8, 9, 10 plus fixed sim episode 4 reference.
- Policy: `oftplus_h5_vision`, checkpoint step 28560.
- This is an offline projection analysis. It is not a new causal intervention or rollout result.

## Real-Only Overall Summary

{table(real, overall_fields)}

## Real-Only Phase Summary

{table(phase_real, phase_fields)}

## Interpretation

The key quantity is `hidden_sensitive_ratio`, the fraction of hidden delta norm captured by the Phase 6 policy-sensitive subspace.

If a condition has high total hidden energy but low sensitive ratio, it may be visually/representationally different while not strongly aligned with the previously identified action-sensitive structure.

If a condition has high sensitive energy or ratio and also high action/gripper gap, it is a stronger candidate for a policy-relevant environment perturbation.

## Limitations

- The basis was built from the previous 5-episode P0 Real-Sim setting, not from Phase 8 real-only perturbations.
- Projection onto this basis is compatibility evidence, not proof that the same hidden directions causally drive the new action gaps.
- Because Phase 8 action gap is gripper-dominated, a future gripper-specific sensitive basis may be more diagnostic.
"""


def main() -> int:
    OUT.mkdir(parents=True, exist_ok=True)
    basis_payload = np.load(BASIS_FILE, allow_pickle=True)
    basis = basis_payload["basis"].astype(np.float64)
    if basis.shape[1] != 35 * 4096:
        raise RuntimeError(f"Unexpected basis shape: {basis.shape}")

    feature_maps = {
        "real": load_manifest(FEATURE_ROOT / "real"),
        "sim": load_manifest(FEATURE_ROOT / "sim_episode_000004_reference"),
    }

    aligned = read_csv(ROOT / "03_pair_alignment" / "aligned_pairs.csv")
    action_rows = read_csv(ROOT / "07_shadow_action_analysis" / "action_frame_metrics.csv")
    action_key = {
        (
            row["comparison_id"],
            row["left_domain"],
            row["left_episode"],
            int(row["left_frame"]),
            row["right_domain"],
            row["right_episode"],
            int(row["right_frame"]),
        ): row
        for row in action_rows
    }

    frame_rows: list[dict[str, Any]] = []
    missing: list[dict[str, Any]] = []
    for pair in aligned:
        if pair["pair_valid"] != "True":
            continue
        left_domain = pair["left_domain"]
        right_domain = pair["right_domain"]
        left_image = str(Path(pair["left_image_path"]).resolve())
        right_image = str(Path(pair["right_image_path"]).resolve())
        left_feature = feature_maps[left_domain].get(left_image)
        right_feature = feature_maps[right_domain].get(right_image)
        if left_feature is None or right_feature is None:
            missing.append({"comparison_id": pair["comparison_id"], "left_image": left_image, "right_image": right_image})
            continue
        left_hidden = hidden(left_feature)
        right_hidden = hidden(right_feature)
        delta = right_hidden - left_hidden
        energies = projection_energy(delta, basis)
        key = (
            pair["comparison_id"],
            left_domain,
            pair["left_episode"],
            int(float(pair["left_frame"])),
            right_domain,
            pair["right_episode"],
            int(float(pair["right_frame"])),
        )
        action = action_key.get(key, {})
        frame_rows.append(
            {
                "comparison_id": pair["comparison_id"],
                "phase": pair["left_phase"],
                "left_domain": left_domain,
                "left_episode": pair["left_episode"],
                "left_frame": int(float(pair["left_frame"])),
                "right_domain": right_domain,
                "right_episode": pair["right_episode"],
                "right_frame": int(float(pair["right_frame"])),
                "progress": float(pair["left_progress"]),
                **energies,
                "chunk_mean_l2": f(action, "chunk_mean_l2"),
                "chunk_mean_translation_l2": f(action, "chunk_mean_translation_l2"),
                "chunk_mean_rotation_l2": f(action, "chunk_mean_rotation_l2"),
                "chunk_mean_gripper_abs": f(action, "chunk_mean_gripper_abs"),
                "gripper_disagreement_rate": f(action, "gripper_disagreement_rate"),
            }
        )

    overall = aggregate(frame_rows, ["comparison_id"])
    by_phase = aggregate(frame_rows, ["comparison_id", "phase"])

    write_csv(OUT / "sensitive_projection_frame_metrics.csv", frame_rows)
    write_csv(OUT / "sensitive_projection_summary_overall.csv", overall)
    write_csv(OUT / "sensitive_projection_summary_by_phase.csv", by_phase)
    write_json(
        OUT / "summary.json",
        {
            "basis_file": str(BASIS_FILE),
            "basis_shape": list(basis.shape),
            "frame_rows": len(frame_rows),
            "missing_feature_pairs": len(missing),
            "status": "VERIFIED_OFFLINE_PROJECTION" if not missing else "PARTIAL_MISSING_FEATURES",
            "outputs": {
                "frame_metrics": str(OUT / "sensitive_projection_frame_metrics.csv"),
                "summary_overall": str(OUT / "sensitive_projection_summary_overall.csv"),
                "summary_by_phase": str(OUT / "sensitive_projection_summary_by_phase.csv"),
            },
        },
    )
    if missing:
        write_csv(OUT / "missing_feature_pairs.csv", missing)
    (OUT / "sensitive_projection_report.md").write_text(build_report(overall, by_phase), encoding="utf-8")
    print(json.dumps({"rows": len(frame_rows), "missing": len(missing), "basis_shape": list(basis.shape)}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
