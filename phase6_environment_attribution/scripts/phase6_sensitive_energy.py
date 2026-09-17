#!/usr/bin/env python3
from __future__ import annotations

import csv
import json
import sys
from pathlib import Path
from typing import Any

import numpy as np


ROOT = Path("/home/ubuntu/a0509_vla_linux_field_bundle_20260903")
LHJ = ROOT / "lhj"
BASE_SCRIPT_DIR = LHJ / "scripts"
PHASE6_SCRIPT_DIR = LHJ / "phase6_environment_attribution" / "scripts"
for path in [str(BASE_SCRIPT_DIR), str(PHASE6_SCRIPT_DIR)]:
    if path not in sys.path:
        sys.path.insert(0, path)

from action_hidden_sensitive_null_decomposition import grad_direction, project_onto  # noqa: E402
from action_hidden_token_sensitivity import CHECKPOINT, DATASET_KEY, load_action_head, read_json, write_csv, write_json  # noqa: E402
from phase6_env_x_hidden_correction import load_condition, progress_phase_shift  # noqa: E402


PHASE6 = LHJ / "phase6_environment_attribution"
OUTPUT_DIR = PHASE6 / "06_policy_relevance" / "sensitive_energy"
K = 128


def flatten(arr: np.ndarray) -> np.ndarray:
    return arr.reshape(-1).astype(np.float64)


def build_basis(vectors: list[np.ndarray], k_max: int) -> np.ndarray:
    x = np.stack([flatten(v) for v in vectors], axis=0)
    x = x - x.mean(axis=0, keepdims=True)
    gram = x @ x.T
    vals, vecs = np.linalg.eigh(gram)
    order = np.argsort(vals)[::-1]
    vals = vals[order]
    vecs = vecs[:, order]
    keep = vals > 1e-10
    vals = vals[keep]
    vecs = vecs[:, keep]
    k = min(k_max, vals.size)
    if k == 0:
        return np.zeros((0, x.shape[1]), dtype=np.float64)
    basis = (vecs[:, :k].T @ x) / np.sqrt(vals[:k, None])
    basis /= np.maximum(np.linalg.norm(basis, axis=1, keepdims=True), 1e-12)
    return basis.astype(np.float64)


def projection_energy(delta: np.ndarray, basis: np.ndarray) -> dict[str, float]:
    flat = flatten(delta)
    total = float(np.linalg.norm(flat))
    if basis.shape[0] == 0 or total <= 1e-12:
        return {
            "total_energy": total,
            "sensitive_energy": 0.0,
            "null_energy": total,
            "sensitive_ratio": 0.0,
        }
    coeff = basis @ flat
    sensitive = float(np.linalg.norm(coeff))
    null = float(max(total * total - sensitive * sensitive, 0.0) ** 0.5)
    return {
        "total_energy": total,
        "sensitive_energy": sensitive,
        "null_energy": null,
        "sensitive_ratio": sensitive / total,
    }


def aggregate(rows: list[dict[str, Any]], group_keys: list[str]) -> list[dict[str, Any]]:
    grouped: dict[tuple[Any, ...], list[dict[str, Any]]] = {}
    for row in rows:
        grouped.setdefault(tuple(row[key] for key in group_keys), []).append(row)
    metrics = ["total_energy", "sensitive_energy", "null_energy", "sensitive_ratio"]
    out = []
    for key, items in sorted(grouped.items(), key=lambda kv: kv[0]):
        rec = {k: v for k, v in zip(group_keys, key)}
        rec["count"] = len(items)
        for metric in metrics:
            vals = np.asarray([float(item[metric]) for item in items], dtype=np.float64)
            rec[f"{metric}_mean"] = float(vals.mean())
            rec[f"{metric}_p50"] = float(np.percentile(vals, 50))
            rec[f"{metric}_p90"] = float(np.percentile(vals, 90))
        out.append(rec)
    return out


def variant_deltas() -> list[dict[str, Any]]:
    p0 = load_condition("P0_current_paired_image")
    p4 = load_condition("P4_letterbox_224")
    rows = []
    for condition_name, samples in [("original_P0", p0), ("aligned_P4", p4)]:
        for sample in samples:
            rows.append(
                {
                    "variant": f"{condition_name}_raw",
                    "frame": sample["frame"],
                    "episode_id": sample["episode_id"],
                    "progress": sample["progress"],
                    "planner_phase": sample["planner_phase"],
                    "delta": sample["sim_hidden"] - sample["real_hidden"],
                }
            )

    for condition_name, samples in [("original_P0", p0), ("aligned_P4", p4)]:
        episodes = sorted({sample["episode_id"] for sample in samples})
        for episode in episodes:
            train = [sample for sample in samples if sample["episode_id"] != episode]
            test = [sample for sample in samples if sample["episode_id"] == episode]
            for sample in test:
                shift = progress_phase_shift(train, int(sample["progress"]), sample["planner_phase"])
                rows.append(
                    {
                        "variant": f"{condition_name}_condition_specific_progress_phase",
                        "frame": sample["frame"],
                        "episode_id": sample["episode_id"],
                        "progress": sample["progress"],
                        "planner_phase": sample["planner_phase"],
                        "delta": sample["sim_hidden"] - (sample["real_hidden"] + shift),
                    }
                )

    episodes = sorted({sample["episode_id"] for sample in p4})
    for episode in episodes:
        train_p0 = [sample for sample in p0 if sample["episode_id"] != episode]
        test_p4 = [sample for sample in p4 if sample["episode_id"] == episode]
        for sample in test_p4:
            shift = progress_phase_shift(train_p0, int(sample["progress"]), sample["planner_phase"])
            rows.append(
                {
                    "variant": "aligned_P4_original_P0_trained_progress_phase_transfer",
                    "frame": sample["frame"],
                    "episode_id": sample["episode_id"],
                    "progress": sample["progress"],
                    "planner_phase": sample["planner_phase"],
                    "delta": sample["sim_hidden"] - (sample["real_hidden"] + shift),
                }
            )
    return rows


def main() -> int:
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    p0 = load_condition("P0_current_paired_image")
    action_stats = read_json(CHECKPOINT / "dataset_statistics.json")[DATASET_KEY]["action"]
    action_head, device, action_head_checkpoint = load_action_head(CHECKPOINT)

    sensitive_vectors = []
    for sample in p0:
        grad, _ = grad_direction(action_head, sample["real_hidden"].astype(np.float32), sample["sim_action"], action_stats, device)
        sensitive_vectors.append(project_onto(sample["delta"], -grad))
    basis = build_basis(sensitive_vectors, K)
    np.savez_compressed(
        OUTPUT_DIR / "phase6_p0_sensitive_basis_k128.npz",
        basis=basis.astype(np.float32),
        k=np.asarray([basis.shape[0]], dtype=np.int32),
        source=np.asarray(["P0_current_paired_image"], dtype=object),
    )

    frame_rows = []
    for row in variant_deltas():
        energies = projection_energy(row["delta"], basis)
        frame_rows.append(
            {
                "variant": row["variant"],
                "frame": row["frame"],
                "episode_id": row["episode_id"],
                "progress": row["progress"],
                "planner_phase": row["planner_phase"],
                **energies,
            }
        )
    write_csv(OUTPUT_DIR / "phase6_sensitive_energy_frame_metrics.csv", frame_rows)
    overall = aggregate(frame_rows, ["variant"])
    by_phase = aggregate(frame_rows, ["variant", "planner_phase"])
    write_csv(OUTPUT_DIR / "phase6_sensitive_energy_summary_overall.csv", overall)
    write_csv(OUTPUT_DIR / "phase6_sensitive_energy_summary_by_phase.csv", by_phase)

    # Root-level policy attribution table.
    write_csv(PHASE6 / "policy_sensitive_attribution.csv", overall)
    write_json(
        OUTPUT_DIR / "phase6_sensitive_energy_summary.json",
        {
            "basis_file": str(OUTPUT_DIR / "phase6_p0_sensitive_basis_k128.npz"),
            "basis_rank": int(basis.shape[0]),
            "action_head_checkpoint": action_head_checkpoint,
            "variant_count": len(overall),
            "frame_rows": len(frame_rows),
            "status": "VERIFIED_OFFLINE_SENSITIVE_SUBSPACE_PROJECTION",
            "notes": [
                "Basis is reconstructed from P0 local action-sensitive components.",
                "Sensitive energy is a projection onto this P0-derived subspace, not a proof of causal neuron identity.",
            ],
        },
    )
    print(json.dumps({"basis_rank": int(basis.shape[0]), "rows": len(frame_rows)}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
