#!/usr/bin/env python3
from __future__ import annotations

import csv
import json
from pathlib import Path
from typing import Any

import numpy as np


ROOT = Path("/home/ubuntu/a0509_vla_linux_field_bundle_20260903")
PHASE6 = ROOT / "lhj" / "phase6_environment_attribution"
CAMERA_DIR = PHASE6 / "03_camera"
FEATURE_ROOT = CAMERA_DIR / "full_forward_features"
BASIS_FILE = PHASE6 / "06_policy_relevance" / "sensitive_energy" / "phase6_p0_sensitive_basis_k128.npz"
OUT_DIR = CAMERA_DIR / "sensitive_energy"


def read_json(path: Path) -> Any:
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2, sort_keys=True)


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
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def load_hidden_map(manifest_path: Path) -> dict[str, np.ndarray]:
    manifest = read_json(manifest_path)
    out: dict[str, np.ndarray] = {}
    for record in manifest["records"]:
        image_stem = Path(record["source_image"]).stem
        feature_path = manifest_path.parent / record["feature_file"]
        with np.load(feature_path, allow_pickle=True) as data:
            out[image_stem] = data["action_hidden_states.input"].astype(np.float64)
    return out


def projection_energy(delta: np.ndarray, basis: np.ndarray) -> dict[str, float]:
    flat = delta.reshape(-1).astype(np.float64)
    total = float(np.linalg.norm(flat))
    if total <= 1e-12 or basis.size == 0:
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


def aggregate(rows: list[dict[str, Any]], keys: list[str]) -> list[dict[str, Any]]:
    grouped: dict[tuple[Any, ...], list[dict[str, Any]]] = {}
    for row in rows:
        grouped.setdefault(tuple(row[key] for key in keys), []).append(row)
    metrics = ["total_energy", "sensitive_energy", "null_energy", "sensitive_ratio"]
    out: list[dict[str, Any]] = []
    for group_key, items in sorted(grouped.items(), key=lambda item: item[0]):
        rec = {key: value for key, value in zip(keys, group_key)}
        rec["count"] = len(items)
        for metric in metrics:
            values = np.asarray([float(item[metric]) for item in items], dtype=np.float64)
            rec[f"{metric}_mean"] = float(values.mean())
            rec[f"{metric}_median"] = float(np.median(values))
            rec[f"{metric}_p90"] = float(np.percentile(values, 90))
        out.append(rec)
    return out


def parse_episode(image_stem: str) -> str:
    # Stems are episode_000001_000000.
    parts = image_stem.split("_")
    return "_".join(parts[:2]) if len(parts) >= 3 else image_stem


def main() -> int:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    basis_payload = np.load(BASIS_FILE, allow_pickle=True)
    basis = basis_payload["basis"].astype(np.float64)

    frame_rows: list[dict[str, Any]] = []
    condition_dirs = sorted(path for path in FEATURE_ROOT.iterdir() if path.is_dir())
    for condition_dir in condition_dirs:
        condition = condition_dir.name
        real_hidden = load_hidden_map(condition_dir / "real" / "feature_manifest.json")
        sim_hidden = load_hidden_map(condition_dir / "sim" / "feature_manifest.json")
        common = sorted(set(real_hidden) & set(sim_hidden))
        for image_stem in common:
            delta = sim_hidden[image_stem] - real_hidden[image_stem]
            energies = projection_energy(delta, basis)
            frame_rows.append(
                {
                    "condition": condition,
                    "pair_id": image_stem,
                    "episode_id": parse_episode(image_stem),
                    **energies,
                }
            )

    overall = aggregate(frame_rows, ["condition"])
    by_episode = aggregate(frame_rows, ["condition", "episode_id"])
    write_csv(OUT_DIR / "camera_sensitive_energy_frame_metrics.csv", frame_rows)
    write_csv(OUT_DIR / "camera_sensitive_energy_summary_overall.csv", overall)
    write_csv(OUT_DIR / "camera_sensitive_energy_summary_by_episode.csv", by_episode)

    # Merge overall sensitive energy into the camera ablation table.
    camera_table_path = PHASE6 / "camera_ablation.csv"
    camera_rows = []
    with camera_table_path.open("r", newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            camera_rows.append(dict(row))
    by_condition = {row["condition"]: row for row in overall}
    for row in camera_rows:
        energy = by_condition.get(row["condition"])
        if not energy:
            continue
        for key in [
            "total_energy_mean",
            "sensitive_energy_mean",
            "null_energy_mean",
            "sensitive_ratio_mean",
        ]:
            row[f"camera_{key}"] = energy[key]

    c0 = next(row for row in camera_rows if row["condition"] == "C0_original")
    base_sensitive = float(c0["camera_sensitive_energy_mean"])
    base_ratio = float(c0["camera_sensitive_ratio_mean"])
    for row in camera_rows:
        sensitive = float(row.get("camera_sensitive_energy_mean", "nan"))
        ratio = float(row.get("camera_sensitive_ratio_mean", "nan"))
        row["camera_sensitive_energy_reduction_pct"] = (
            (base_sensitive - sensitive) / base_sensitive * 100.0 if base_sensitive > 0 else np.nan
        )
        row["camera_sensitive_ratio_delta"] = ratio - base_ratio
    write_csv(camera_table_path, camera_rows)
    write_csv(PHASE6 / "tables" / "camera_ablation.csv", camera_rows)

    best_sensitive = max(
        camera_rows,
        key=lambda row: float(row.get("camera_sensitive_energy_reduction_pct", "-inf")),
    )
    write_json(
        OUT_DIR / "camera_sensitive_energy_summary.json",
        {
            "basis_file": str(BASIS_FILE),
            "basis_rank": int(basis.shape[0]),
            "condition_count": len(condition_dirs),
            "frame_rows": len(frame_rows),
            "best_sensitive_energy_condition": best_sensitive["condition"],
            "best_sensitive_energy_reduction_pct": float(best_sensitive["camera_sensitive_energy_reduction_pct"]),
            "status": "VERIFIED_OFFLINE_CAMERA_SENSITIVE_SUBSPACE_PROJECTION",
            "notes": [
                "Projection uses the P0-derived Phase6 sensitive basis.",
                "This is a policy-relevance proxy, not calibrated camera geometry evidence.",
            ],
        },
    )
    print(
        json.dumps(
            {
                "conditions": len(condition_dirs),
                "frame_rows": len(frame_rows),
                "best_sensitive": best_sensitive["condition"],
                "best_sensitive_reduction_pct": float(best_sensitive["camera_sensitive_energy_reduction_pct"]),
            },
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
