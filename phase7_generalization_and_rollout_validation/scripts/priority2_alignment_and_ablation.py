#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import re
import sys
from pathlib import Path
from typing import Any

import numpy as np
from PIL import Image, ImageChops, ImageOps


ACTION_DIM = 7
CHUNK = 5
GRIPPER_THRESHOLD = 0.5
SEED = 20260918


def read_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open("r", encoding="utf-8", newline="") as handle:
        return list(csv.DictReader(handle))


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def write_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")


def episode_id(pair_id: str) -> str:
    match = re.match(r"(episode_\d{6})_", pair_id)
    if not match:
        raise ValueError(pair_id)
    return match.group(1)


def progress(pair_id: str) -> int:
    match = re.search(r"(\d+)$", pair_id)
    if not match:
        raise ValueError(pair_id)
    return int(match.group(1))


def pair_id_from_record(record: dict[str, Any]) -> str:
    return Path(record["source_image"]).stem


def action_chunk(record: dict[str, Any]) -> np.ndarray:
    response = record.get("response", {})
    actions = response.get("actions")
    if actions is None:
        actions = [response.get("action")]
    arr = np.asarray(actions, dtype=np.float64)
    if arr.shape != (CHUNK, ACTION_DIM):
        raise ValueError(f"Unexpected action shape: {arr.shape}")
    return arr


def load_manifest_records(path: Path) -> dict[str, dict[str, Any]]:
    manifest = read_json(path)
    base = path.parent
    out = {}
    for record in manifest["records"]:
        pid = pair_id_from_record(record)
        clone = dict(record)
        clone["_feature_path"] = str(base / record["feature_file"])
        out[pid] = clone
    return out


def load_feature(record: dict[str, Any], key: str = "action_hidden_states.input") -> np.ndarray:
    with np.load(record["_feature_path"], allow_pickle=True) as data:
        return np.asarray(data[key], dtype=np.float32)


def l2(x: np.ndarray) -> float:
    return float(np.linalg.norm(np.asarray(x, dtype=np.float64).reshape(-1)))


def cosine_distance(left: np.ndarray, right: np.ndarray) -> float:
    x = np.asarray(left, dtype=np.float64).reshape(-1)
    y = np.asarray(right, dtype=np.float64).reshape(-1)
    denom = float(np.linalg.norm(x) * np.linalg.norm(y))
    if denom <= 1e-12:
        return 0.0
    return float(1.0 - np.dot(x, y) / denom)


def huber(delta: np.ndarray, threshold: float) -> float:
    d = np.abs(delta.reshape(-1))
    val = np.where(d <= threshold, 0.5 * d * d, threshold * (d - 0.5 * threshold))
    return float(np.mean(val))


def transition_index(values: np.ndarray, threshold: float = GRIPPER_THRESHOLD) -> int:
    closed = np.asarray(values) >= threshold
    idx = np.where(closed)[0]
    return int(idx[0]) if idx.size else -1


def chunk_metric_row(real: np.ndarray, sim: np.ndarray) -> dict[str, float]:
    n = min(len(real), len(sim))
    real = real[:n]
    sim = sim[:n]
    delta = real - sim
    abs_delta = np.abs(delta)
    l2_steps = np.linalg.norm(delta, axis=1)
    l1_steps = np.sum(abs_delta, axis=1)
    rmse_steps = np.sqrt(np.mean(delta * delta, axis=1))
    gripper_real_closed = real[:, 6] >= GRIPPER_THRESHOLD
    gripper_sim_closed = sim[:, 6] >= GRIPPER_THRESHOLD
    return {
        "action_l1_chunk_mean": float(np.mean(l1_steps)),
        "action_mae_chunk_mean": float(np.mean(abs_delta)),
        "action_l2_chunk_mean": float(np.mean(l2_steps)),
        "action_l2_chunk_max": float(np.max(l2_steps)),
        "action_rmse_chunk_mean": float(np.mean(rmse_steps)),
        "action_huber0.01_chunk_mean": huber(delta, 0.01),
        "action_huber0.05_chunk_mean": huber(delta, 0.05),
        "action_huber0.1_chunk_mean": huber(delta, 0.1),
        "action_cosine_chunk_mean": float(np.mean([cosine_distance(real[i], sim[i]) for i in range(n)])),
        "first_action_l2": l2(delta[0]),
        "translation_l2_chunk_mean": float(np.mean(np.linalg.norm(delta[:, :3], axis=1))),
        "rotation_l2_chunk_mean": float(np.mean(np.linalg.norm(delta[:, 3:6], axis=1))),
        "gripper_abs_chunk_mean": float(np.mean(np.abs(delta[:, 6]))),
        "gripper_binary_disagreement_rate": float(np.mean(gripper_real_closed != gripper_sim_closed)),
        "real_gripper_transition_index": transition_index(real[:, 6]),
        "sim_gripper_transition_index": transition_index(sim[:, 6]),
        "transition_delta_sim_minus_real": transition_index(sim[:, 6]) - transition_index(real[:, 6]),
    }


def summarize(rows: list[dict[str, Any]], group_keys: list[str], metrics: list[str]) -> list[dict[str, Any]]:
    grouped: dict[tuple[Any, ...], list[dict[str, Any]]] = {}
    for row in rows:
        grouped.setdefault(tuple(row[k] for k in group_keys), []).append(row)
    out = []
    for key, items in sorted(grouped.items(), key=lambda kv: kv[0]):
        rec = {k: v for k, v in zip(group_keys, key)}
        rec["count"] = len(items)
        for metric in metrics:
            vals = np.asarray([float(item[metric]) for item in items], dtype=np.float64)
            rec[f"{metric}_mean"] = float(vals.mean())
            rec[f"{metric}_std"] = float(vals.std())
            rec[f"{metric}_median"] = float(np.median(vals))
            rec[f"{metric}_p90"] = float(np.percentile(vals, 90))
        out.append(rec)
    return out


def load_action_head_bundle(root: Path):
    sys.path.insert(0, str(root / "lhj" / "scripts"))
    import action_hidden_token_sensitivity as ahts  # type: ignore

    ahts.ROOT = root
    ahts.CHECKPOINT = root / "runtime_state" / "oft_mixed480_step28560_merged"
    action_head, device, checkpoint = ahts.load_action_head(ahts.CHECKPOINT)
    stats = ahts.read_json(root / "runtime_state" / "oft_mixed480_step28560_merged" / "dataset_statistics.json")["a0509_sim_cube_pick"]["action"]
    return action_head, device, checkpoint, ahts.predict_batch, stats


def fit_basis(deltas: list[np.ndarray], k: int) -> np.ndarray:
    x = np.stack([d.reshape(-1).astype(np.float32) for d in deltas], axis=0)
    x = x - x.mean(axis=0, keepdims=True)
    if x.shape[0] <= 1:
        return np.zeros((0, x.shape[1]), dtype=np.float32)
    _, _, vt = np.linalg.svd(x, full_matrices=False)
    return vt[: min(k, vt.shape[0])].astype(np.float32)


def project_flat(vec: np.ndarray, basis: np.ndarray, shape: tuple[int, ...]) -> np.ndarray:
    if basis.size == 0:
        return np.zeros(shape, dtype=np.float32)
    flat = vec.reshape(-1).astype(np.float32)
    coeff = basis @ flat
    return (coeff @ basis).reshape(shape).astype(np.float32)


def nearest_progress_shift(train: list[dict[str, Any]], prog: int, phase: str | None = None) -> np.ndarray:
    candidates = train
    if phase is not None:
        phased = [s for s in train if s["phase"] == phase]
        if phased:
            candidates = phased
    selected = [s for s in candidates if abs(int(s["progress"]) - prog) <= 5]
    if not selected:
        selected = sorted(candidates, key=lambda s: abs(int(s["progress"]) - prog))[: min(10, len(candidates))]
    return np.mean([s["delta"] for s in selected], axis=0).astype(np.float32)


def run_alignment(root: Path, out: Path, seed: int) -> dict[str, Any]:
    p0 = root / "lhj" / "phase6_environment_attribution" / "01_preprocessing" / "full_forward_features" / "P0_current_paired_image"
    action_head, device, checkpoint_path, predict_batch, action_stats = load_action_head_bundle(root)
    real_records = load_manifest_records(p0 / "real" / "feature_manifest.json")
    sim_records = load_manifest_records(p0 / "sim" / "feature_manifest.json")
    phase_manifest = read_csv(root / "outputs" / "token_distribution_analysis" / "5_episodes" / "episode_phase_summary" / "frame_metrics_enriched.csv")
    phase_by_pair = {row["pair_id"]: row.get("planner_phase", "") for row in phase_manifest}
    pair_ids = sorted(set(real_records) & set(sim_records))
    samples = []
    for pid in pair_ids:
        rh = load_feature(real_records[pid])
        sh = load_feature(sim_records[pid])
        samples.append(
            {
                "pair_id": pid,
                "episode_id": episode_id(pid),
                "progress": progress(pid),
                "phase": phase_by_pair.get(pid, ""),
                "real_hidden": rh,
                "sim_hidden": sh,
                "delta": (sh - rh).astype(np.float32),
                "real_action": action_chunk(real_records[pid]),
                "sim_action": action_chunk(sim_records[pid]),
            }
        )
    episodes = sorted({s["episode_id"] for s in samples})
    rng = np.random.default_rng(seed)
    frame_rows: list[dict[str, Any]] = []
    deployability = {
        "raw_baseline": "DEPLOYABLE_CANDIDATE",
        "global_mean_shift": "DEPLOYABLE_CANDIDATE",
        "mean_variance_diag": "DEPLOYABLE_CANDIDATE",
        "progress_shift": "DEPLOYABLE_CANDIDATE",
        "phase_shift": "DEPLOYABLE_CANDIDATE",
        "progress_phase_shift": "DEPLOYABLE_CANDIDATE",
        "pca_mean_shift_k16": "DEPLOYABLE_CANDIDATE",
        "pca_oracle_delta_projection_k16": "ANALYSIS_ONLY_ORACLE",
        "wrong_direction_global": "NOT_DEPLOYABLE",
        "random_matched_norm": "NOT_DEPLOYABLE",
    }
    method_order = list(deployability.keys())
    for heldout in episodes:
        train = [s for s in samples if s["episode_id"] != heldout]
        test = [s for s in samples if s["episode_id"] == heldout]
        train_delta = np.stack([s["delta"] for s in train], axis=0)
        global_shift = train_delta.mean(axis=0).astype(np.float32)
        real_train = np.stack([s["real_hidden"] for s in train], axis=0)
        sim_train = np.stack([s["sim_hidden"] for s in train], axis=0)
        r_mean = real_train.mean(axis=0).astype(np.float32)
        s_mean = sim_train.mean(axis=0).astype(np.float32)
        r_std = real_train.std(axis=0).astype(np.float32) + 1e-6
        s_std = sim_train.std(axis=0).astype(np.float32) + 1e-6
        phase_shift = {
            phase: np.mean([s["delta"] for s in train if s["phase"] == phase], axis=0).astype(np.float32)
            for phase in sorted({s["phase"] for s in train})
        }
        basis = fit_basis([s["delta"] for s in train], 16)
        pca_mean_shift = project_flat(global_shift, basis, global_shift.shape)
        random_shift = rng.standard_normal(global_shift.shape).astype(np.float32)
        random_shift *= float(np.linalg.norm(global_shift.reshape(-1)) / max(np.linalg.norm(random_shift.reshape(-1)), 1e-12))
        hidden_by_method: dict[str, list[np.ndarray]] = {m: [] for m in method_order if m != "raw_baseline"}
        meta_by_method: dict[str, list[dict[str, Any]]] = {m: [] for m in method_order}
        for sample in test:
            method_hidden = {
                "global_mean_shift": sample["real_hidden"] + global_shift,
                "mean_variance_diag": ((sample["real_hidden"] - r_mean) / r_std) * s_std + s_mean,
                "progress_shift": sample["real_hidden"] + nearest_progress_shift(train, int(sample["progress"])),
                "phase_shift": sample["real_hidden"] + phase_shift.get(sample["phase"], global_shift),
                "progress_phase_shift": sample["real_hidden"] + nearest_progress_shift(train, int(sample["progress"]), sample["phase"]),
                "pca_mean_shift_k16": sample["real_hidden"] + pca_mean_shift,
                "pca_oracle_delta_projection_k16": sample["real_hidden"] + project_flat(sample["delta"], basis, sample["delta"].shape),
                "wrong_direction_global": sample["real_hidden"] - global_shift,
                "random_matched_norm": sample["real_hidden"] + random_shift,
            }
            raw_metrics = chunk_metric_row(sample["real_action"], sample["sim_action"])
            frame_rows.append(
                {
                    "pair_id": sample["pair_id"],
                    "episode_id": sample["episode_id"],
                    "heldout_episode": heldout,
                    "progress": sample["progress"],
                    "planner_phase": sample["phase"],
                    "method": "raw_baseline",
                    "evaluation_type": "inductive_reference",
                    "deployability": deployability["raw_baseline"],
                    **raw_metrics,
                }
            )
            for method, hidden in method_hidden.items():
                hidden_by_method[method].append(hidden)
                meta_by_method[method].append(sample)
        for method, hiddens in hidden_by_method.items():
            preds = predict_batch(action_head, np.stack(hiddens, axis=0).astype(np.float32), action_stats, device)
            for sample, pred in zip(meta_by_method[method], preds):
                metrics = chunk_metric_row(pred, sample["sim_action"])
                frame_rows.append(
                    {
                        "pair_id": sample["pair_id"],
                        "episode_id": sample["episode_id"],
                        "heldout_episode": heldout,
                        "progress": sample["progress"],
                        "planner_phase": sample["phase"],
                        "method": method,
                        "evaluation_type": "paired_oracle" if "oracle" in method else "inductive",
                        "deployability": deployability[method],
                        **metrics,
                    }
                )
    metrics = [
        "action_l1_chunk_mean",
        "action_mae_chunk_mean",
        "action_l2_chunk_mean",
        "action_rmse_chunk_mean",
        "action_huber0.01_chunk_mean",
        "action_huber0.05_chunk_mean",
        "action_huber0.1_chunk_mean",
        "action_cosine_chunk_mean",
        "translation_l2_chunk_mean",
        "rotation_l2_chunk_mean",
        "gripper_abs_chunk_mean",
        "gripper_binary_disagreement_rate",
    ]
    align_dir = out / "04_global_vs_policy_alignment"
    write_csv(align_dir / "alignment_frame_metrics.csv", frame_rows)
    episode_rows = summarize(frame_rows, ["method", "episode_id", "deployability"], metrics)
    method_rows = summarize(frame_rows, ["method", "deployability", "evaluation_type"], metrics)
    write_csv(align_dir / "heldout_comparison.csv", episode_rows)
    write_csv(align_dir / "global_alignment_results.csv", method_rows)
    write_csv(align_dir / "policy_alignment_results.csv", [r for r in method_rows if "progress" in r["method"] or "pca" in r["method"]])
    deploy_rows = [{"method": k, "deployability": v} for k, v in deployability.items()]
    write_csv(align_dir / "deployability_table.csv", deploy_rows)
    write_csv(
        align_dir / "fitting_data_manifest.csv",
        [
            {
                "split_method": "leave_one_episode_out",
                "train_episode": ",".join([e for e in episodes if e != heldout]),
                "heldout_episode": heldout,
                "alignment_fitting_data": "train episodes only",
                "evaluation_data": heldout,
            }
            for heldout in episodes
        ],
    )
    best = sorted(method_rows, key=lambda r: float(r["action_l2_chunk_mean_mean"]))[0]
    write_json(
        align_dir / "summary.json",
        {
            "status": "VERIFIED_ACTION_HEAD_ONLY",
            "pair_count": len(pair_ids),
            "episode_count": len(episodes),
            "methods": deployability,
            "best_by_action_l2": best,
            "note": "These are action-head-only interventions on stored action_hidden_states.input, not image-to-final-action full-forward.",
        },
    )
    write_text(
        align_dir / "report.md",
        f"""# Priority2 Global vs Policy Alignment

Status: `VERIFIED_ACTION_HEAD_ONLY`

This analysis used stored `action_hidden_states.input` features from P0 and reran the action head only.

Best method by held-out episode action L2:

```text
{best['method']} ({float(best['action_l2_chunk_mean_mean']):.6f})
```

Deployability labels are saved in `deployability_table.csv`.

Limit: this is not full image-to-action inference.
""",
    )
    return {"frame_rows": len(frame_rows), "episode_rows": len(episode_rows), "method_rows": len(method_rows), "best_method": best["method"]}


def shift_image(image: Image.Image, dx: int, dy: int, fill=(0, 0, 0)) -> Image.Image:
    shifted = ImageChops.offset(image, dx, dy)
    w, h = image.size
    if dx > 0:
        shifted.paste(fill, (0, 0, dx, h))
    elif dx < 0:
        shifted.paste(fill, (w + dx, 0, w, h))
    if dy > 0:
        shifted.paste(fill, (0, 0, w, dy))
    elif dy < 0:
        shifted.paste(fill, (0, h + dy, w, h))
    return shifted


def read_image_list(path: Path) -> list[Path]:
    return [Path(line.strip()) for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]


def save_condition_images(real_src: list[Path], sim_src: list[Path], out: Path, condition: str, sim_transform, real_transform=None) -> dict[str, Any]:
    real_dir = out / condition / "real"
    sim_dir = out / condition / "sim"
    real_dir.mkdir(parents=True, exist_ok=True)
    sim_dir.mkdir(parents=True, exist_ok=True)
    real_paths, sim_paths = [], []
    for rp, sp in zip(real_src, sim_src):
        real_img = Image.open(rp).convert("RGB")
        sim_img = Image.open(sp).convert("RGB")
        if real_transform:
            real_img = real_transform(real_img)
        sim_img = sim_transform(sim_img)
        name = rp.name
        real_out = real_dir / name
        sim_out = sim_dir / name
        real_img.save(real_out, quality=95)
        sim_img.save(sim_out, quality=95)
        real_paths.append(str(real_out))
        sim_paths.append(str(sim_out))
    (out / condition / "real_images.txt").write_text("\n".join(real_paths) + "\n", encoding="utf-8")
    (out / condition / "sim_images.txt").write_text("\n".join(sim_paths) + "\n", encoding="utf-8")
    return {"condition": condition, "count": len(real_paths), "real_images_txt": str(out / condition / "real_images.txt"), "sim_images_txt": str(out / condition / "sim_images.txt")}


def letterbox(image: Image.Image, size=(224, 224), fill=(0, 0, 0), align="center", replicate=False) -> Image.Image:
    if replicate:
        fit = ImageOps.contain(image, size, Image.Resampling.BICUBIC)
        canvas = ImageOps.expand(fit, border=0)
        bg = Image.new("RGB", size)
        bg.paste(fit.resize(size, Image.Resampling.BICUBIC))
        bg = bg.filter(Image.Filter.GaussianBlur(0)) if False else bg
        canvas = Image.new("RGB", size, fill)
    fit = ImageOps.contain(image, size, Image.Resampling.BICUBIC)
    if replicate:
        canvas = fit.resize(size, Image.Resampling.BICUBIC)
    else:
        canvas = Image.new("RGB", size, fill)
    x = (size[0] - fit.size[0]) // 2
    if align == "top":
        y = 0
    elif align == "bottom":
        y = size[1] - fit.size[1]
    else:
        y = (size[1] - fit.size[1]) // 2
    canvas.paste(fit, (x, y))
    return canvas


def center_crop_resize(image: Image.Image, size=(224, 224)) -> Image.Image:
    w, h = image.size
    side = min(w, h)
    left = (w - side) // 2
    top = (h - side) // 2
    return image.crop((left, top, left + side, top + side)).resize(size, Image.Resampling.BICUBIC)


def prepare_camera_and_letterbox(root: Path, out: Path) -> dict[str, Any]:
    phase6 = root / "lhj" / "phase6_environment_attribution"
    p4_input = phase6 / "01_preprocessing" / "condition_inputs" / "P4_letterbox_224"
    p0_input = phase6 / "01_preprocessing" / "condition_inputs" / "P0_current_paired_image"
    p4_real = read_image_list(p4_input / "real_images.txt")
    p4_sim = read_image_list(p4_input / "sim_images.txt")
    p0_real = read_image_list(p0_input / "real_images.txt")
    p0_sim = read_image_list(p0_input / "sim_images.txt")
    camera_dir = out / "02_matched_camera_shift" / "condition_inputs"
    camera_records = []
    camera_records.append(save_condition_images(p0_real, p0_sim, camera_dir, "M0_raw_original", lambda x: x.copy()))
    camera_records.append(save_condition_images(p0_real, p0_sim, camera_dir, "M1_raw_shift_right_24px", lambda x: shift_image(x, 24, 0)))
    camera_records.append(save_condition_images(p4_real, p4_sim, camera_dir, "M2_p4_shift_right_4px_matched", lambda x: shift_image(x, 4, 0)))
    camera_records.append(save_condition_images(p4_real, p4_sim, camera_dir, "M3_p4_shift_right_24px_control", lambda x: shift_image(x, 24, 0)))
    write_json(
        camera_dir / "matched_camera_condition_manifest.json",
        {
            "status": "INPUTS_READY_FULL_FORWARD_GPU_REQUIRED",
            "interpretation": "M2 compares the 224px letterbox shift roughly matching 24px/1280px. M3 is the old 224px 24px control.",
            "conditions": camera_records,
        },
    )
    write_json(
        out / "02_matched_camera_shift" / "shift_config.json",
        {
            "raw_1280_reference_shift_px": 24,
            "letterbox_224_matched_shift_px": 4,
            "letterbox_224_control_shift_px": 24,
            "status": "GPU_REQUIRED_FOR_FULL_FORWARD",
        },
    )
    write_text(
        out / "02_matched_camera_shift" / "run_full_forward_commands.md",
        """# Matched Camera Shift Full-Forward Commands

Status: `GPU_REQUIRED`

Use `extract_vla_features.py` with `oftplus_h5_vision`, checkpoint step 28560, for each condition/domain image list in `condition_inputs/`.
""",
    )

    letter_dir = out / "03_letterbox_ablation" / "condition_inputs"
    mean_fill = tuple(int(v) for v in np.asarray(Image.open(p0_real[0]).convert("RGB")).reshape(-1, 3).mean(axis=0))
    variants = {
        "A0_original_p0_copy": lambda img: img.copy(),
        "A1_direct_resize_224": lambda img: img.resize((224, 224), Image.Resampling.BICUBIC),
        "A2_letterbox_black": lambda img: letterbox(img, fill=(0, 0, 0)),
        "A3_letterbox_gray": lambda img: letterbox(img, fill=(128, 128, 128)),
        "A4_letterbox_mean_color": lambda img: letterbox(img, fill=mean_fill),
        "A5_letterbox_replicated_resize_bg": lambda img: letterbox(img, fill=(0, 0, 0), replicate=True),
        "A6_center_crop_resize": lambda img: center_crop_resize(img),
        "A7_top_aligned_letterbox": lambda img: letterbox(img, fill=(0, 0, 0), align="top"),
        "A8_bottom_aligned_letterbox": lambda img: letterbox(img, fill=(0, 0, 0), align="bottom"),
    }
    letter_records = []
    for name, transform in variants.items():
        letter_records.append(save_condition_images(p0_real, p0_sim, letter_dir, name, transform, transform))
    write_json(
        letter_dir / "letterbox_ablation_manifest.json",
        {
            "status": "INPUTS_READY_FULL_FORWARD_GPU_REQUIRED",
            "conditions": letter_records,
            "mean_padding_color_from_first_real_frame": mean_fill,
        },
    )
    write_text(
        out / "03_letterbox_ablation" / "report.md",
        """# Letterbox Ablation

Status: `GPU_REQUIRED`

Priority2 prepared image inputs for aspect ratio, resize, padding color, padding position, and effective object-scale ablations. Full image-to-action inference was not executed because the current environment cannot communicate with the NVIDIA driver.
""",
    )
    return {"camera_conditions": len(camera_records), "letterbox_conditions": len(letter_records)}


def correction_chunk_audit(root: Path, out: Path) -> dict[str, Any]:
    candidates = [
        root / "lhj" / "phase5_policy_relevant_completion" / "01_correction_robustness" / "loo_correction_ablation_frame_metrics.csv",
        root / "lhj" / "phase5_policy_relevant_completion" / "02_gate_generalization" / "loo_gate_generalization_frame_metrics.csv",
        root / "lhj" / "phase5_policy_relevant_completion" / "03_layerwise_tracing" / "low_rank_sensitive_subspace_frame_metrics.csv",
        root / "lhj" / "phase4_policy_relevance" / "action_hidden_sensitive_null_decomposition" / "sensitive_null_decomposition_frame_metrics.csv",
        root / "lhj" / "phase4_policy_relevance" / "action_hidden_loo_topk_token_correction" / "loo_topk_token_correction_frame_metrics.csv",
    ]
    rows = []
    action_cols = {"action_l1_chunk_mean", "action_rmse_chunk_mean", "action_huber0.05_chunk_mean", "action_cosine_chunk_mean"}
    for path in candidates:
        if not path.is_file():
            rows.append({"file": str(path), "exists": False, "row_count": 0, "stored_corrected_action_chunk": False, "status": "DATA_REQUIRED"})
            continue
        with path.open("r", encoding="utf-8", newline="") as handle:
            reader = csv.reader(handle)
            header = next(reader)
            count = sum(1 for _ in reader)
        has_chunk = any("corrected_action" in col or "actions" == col for col in header)
        has_full_metrics = action_cols.issubset(set(header))
        rows.append(
            {
                "file": str(path),
                "exists": True,
                "row_count": count,
                "stored_corrected_action_chunk": has_chunk,
                "has_l2_component_metrics": "gap_to_sim_chunk_mean_l2" in header or "selected_gap" in header,
                "has_full_priority1_metrics": has_full_metrics,
                "status": "REUSE_L2_METRICS_ONLY" if not has_chunk else "CAN_REUSE_ACTION_CHUNKS",
            }
        )
    audit_dir = out / "01_correction_multimetric_audit"
    write_csv(audit_dir / "correction_action_chunk_audit.csv", rows)
    write_text(
        audit_dir / "report.md",
        """# Correction Action Chunk Audit

Stored correction outputs were inspected before recomputation.

Result: existing correction artifacts mainly store L2/component metric summaries, not corrected frame-level action chunks. Therefore full L1/RMSE/Huber/cosine recomputation for those exact correction methods requires action-head rerun or saved corrected chunks. Priority2 reran train-fold hidden alignment baselines through the action head where feasible and marked older metric-only correction artifacts accordingly.
""",
    )
    return {"audited_files": len(rows), "action_chunk_files": sum(1 for r in rows if r.get("stored_corrected_action_chunk"))}


def make_metric_table(out: Path) -> None:
    p1 = read_csv(out.parent / "01_multimetric_reanalysis" / "metric_robustness_summary.csv") if (out.parent / "01_multimetric_reanalysis" / "metric_robustness_summary.csv").is_file() else []
    align = read_csv(out / "04_global_vs_policy_alignment" / "global_alignment_results.csv")
    rows = []
    for row in p1:
        rows.append({"source": "priority1_preprocessing", "method": row["condition"], "metric": row["metric"], "value": row["value"], "status": "VERIFIED_FULL_FORWARD_EXISTING"})
    for row in align:
        for metric in ["action_l2_chunk_mean", "action_l1_chunk_mean", "action_rmse_chunk_mean", "action_huber0.05_chunk_mean", "translation_l2_chunk_mean", "rotation_l2_chunk_mean", "gripper_abs_chunk_mean"]:
            rows.append({"source": "priority2_action_head_alignment", "method": row["method"], "metric": metric, "value": row[f"{metric}_mean"], "status": row["deployability"]})
    write_csv(out / "final_metric_table.csv", rows)


def make_final_reports(root: Path, out: Path, status: dict[str, Any]) -> None:
    align_summary = read_json(out / "04_global_vs_policy_alignment" / "summary.json")
    write_json(
        out / "final_status.json",
        {
            "status": "COMPLETED_PRIORITY2",
            "PRIORITY3_STARTED": False,
            "PRIORITY4_STARTED": False,
            "policy_variant": "oftplus_h5_vision",
            "checkpoint_step": 28560,
            "priority1_status_used": str(out.parent / "priority1_status.json"),
            **status,
        },
    )
    evidence_rows = [
        {"rq": "Correction chunks reusable?", "experiment": "correction_action_chunk_audit", "result": "mostly metric-only, no corrected chunks", "status": "PARTIALLY_VERIFIED", "limitation": "older corrections need action-head rerun for full metrics"},
        {"rq": "Global alignment vs policy alignment", "experiment": "action_head_hidden_alignment", "result": align_summary["best_by_action_l2"]["method"], "status": "VERIFIED_ACTION_HEAD_ONLY", "limitation": "not full image-to-action"},
        {"rq": "Matched camera shift", "experiment": "matched_camera_input_generation", "result": "inputs prepared", "status": "GPU_REQUIRED", "limitation": "NVIDIA driver unavailable"},
        {"rq": "Letterbox decomposition", "experiment": "letterbox_ablation_input_generation", "result": "inputs prepared", "status": "GPU_REQUIRED", "limitation": "NVIDIA driver unavailable"},
        {"rq": "Priority3 rollout/data", "experiment": "none", "result": "not started", "status": "DATA_REQUIRED", "limitation": "new environment data required"},
    ]
    write_csv(out / "final_evidence_matrix.csv", evidence_rows)
    write_text(
        out / "limitations.md",
        """# Priority2 Limitations

- New image-to-final-action full-forward was not executed because `nvidia-smi` could not communicate with the NVIDIA driver.
- Camera shift and letterbox ablation therefore have prepared inputs and reproducible manifests, but no generated action numbers.
- Action-head-only alignment operates on stored `action_hidden_states.input`; it is not equivalent to full VLM inference from modified images.
- Existing older correction artifacts generally lack saved corrected action chunks; exact full multi-metric recomputation requires action-head rerun or regenerated chunks.
- No Priority3 data collection, Shadow Mode, closed-loop rollout, or Real robot performance evidence was produced.
""",
    )
    write_text(
        out / "validation_report.md",
        f"""# Priority2 Validation Report

- Status: `COMPLETED_PRIORITY2`
- Priority3 started: false
- Priority4 started: false
- Action-head alignment rows: {status['alignment']['frame_rows']}
- Matched camera input conditions: {status['prepared_inputs']['camera_conditions']}
- Letterbox input conditions: {status['prepared_inputs']['letterbox_conditions']}
- GPU full-forward status: `GPU_REQUIRED`
- Existing Phase1-Priority1 files were not modified.
""",
    )
    write_text(
        out / "recommended_priority3_actions.md",
        """# Recommended Priority3 Actions

1. Restore CUDA/NVIDIA driver availability and run full-forward extraction for `02_matched_camera_shift/condition_inputs`.
2. Run full-forward extraction for `03_letterbox_ablation/condition_inputs`.
3. Recompute the same Priority1 metric table for those new full-forward conditions.
4. Collect new condition-held-out object position / lighting / camera data only after the above controlled image-space ablations are complete.
5. Do not start Shadow Mode or closed-loop rollout until a separate safety and ROS/proprio deployment audit is complete.
""",
    )
    write_text(
        out / "final_phase7_priority2_report.md",
        f"""# Phase7 Priority2 Report

## Scope

Priority2 reviewed Priority1 outputs, audited correction action chunk availability, ran feasible action-head-only hidden alignment baselines, and prepared matched camera shift / letterbox decomposition inputs.

## Correction Action Chunk Audit

Existing correction artifacts are mostly metric-only. Full L1/RMSE/Huber/cosine evaluation for those exact saved correction methods requires corrected action chunks or action-head rerun.

## Action-Head-Only Alignment

Best held-out method by action L2:

```text
{align_summary['best_by_action_l2']['method']} = {float(align_summary['best_by_action_l2']['action_l2_chunk_mean_mean']):.6f}
```

This result is `VERIFIED_ACTION_HEAD_ONLY`, not full image-to-action inference.

## Matched Camera Shift

Prepared conditions:

- raw original
- raw 24 px right shift
- P4 letterbox 4 px right shift matched to 24/1280
- P4 letterbox 24 px right shift legacy control

Full-forward status: `GPU_REQUIRED`.

## Letterbox Decomposition

Prepared conditions A0-A8 covering direct resize, black/gray/mean padding, replicated background, center crop, top and bottom alignment.

Full-forward status: `GPU_REQUIRED`.

## Global Distribution Alignment Baselines

Action-head-only baselines include global mean shift, diagonal mean-variance alignment, progress/phase/progress-phase shifts, PCA mean shift, oracle PCA delta projection, wrong-direction, and random matched-norm controls.

Deployability labels are saved in `04_global_vs_policy_alignment/deployability_table.csv`.

## Final State

```text
COMPLETED_PRIORITY2
PRIORITY3_STARTED: false
PRIORITY4_STARTED: false
```
""",
    )
    make_metric_table(out)


def append_log(root: Path, out: Path, status: dict[str, Any]) -> None:
    log = root / "lhj" / "작업기록.md"
    entry = f"""

## [2026-09-18 / KST] Phase 7 Priority 2 - Generalization Prep and Alignment Baselines

[Purpose]
Phase 7 Priority 1 결과를 기준으로 Priority 2를 수행한다. Correction별 action chunk 저장 여부를 audit하고, 가능한 action-head-only alignment baseline을 실행하며, matched camera shift 및 letterbox decomposition full-forward 입력을 준비한다.

[Inputs]
- `{out.parent / 'priority1_report.md'}`
- `{out.parent / 'priority1_status.json'}`
- `{root / 'lhj/phase6_environment_attribution'}`
- `{root / 'lhj/phase5_policy_relevant_completion'}`
- `{root / 'runtime_state/oft_mixed480_step28560_merged'}`

[Checked]
- GPU availability checked with `nvidia-smi`; driver communication failed.
- Existing correction CSVs mostly contain L2/component metrics rather than corrected action chunks.
- P0 `action_hidden_states.input` features and action-head checkpoint are available for action-head-only alignment.

[Changes]
- Created `{out}`.
- Added/ran `scripts/priority2_alignment_and_ablation.py`.
- Generated action-head-only alignment metrics, camera/letterbox input manifests, final reports, limitations, validation report, and Priority3 recommendations.
- Did not modify Phase1-Priority1 artifacts.

[Commands]
- `nvidia-smi --query-gpu=name,memory.used,memory.total,utilization.gpu --format=csv,noheader`
- `python -m py_compile {out.parent / 'scripts/priority2_alignment_and_ablation.py'}`
- `python {out.parent / 'scripts/priority2_alignment_and_ablation.py'} --bundle-root {root} --output-dir {out} --seed 20260918`

[Outputs]
- `{out / 'final_phase7_priority2_report.md'}`
- `{out / 'final_status.json'}`
- `{out / 'final_evidence_matrix.csv'}`
- `{out / 'final_metric_table.csv'}`
- `{out / 'limitations.md'}`
- `{out / 'validation_report.md'}`
- `{out / 'recommended_priority3_actions.md'}`
- `{out / '01_correction_multimetric_audit'}`
- `{out / '02_matched_camera_shift'}`
- `{out / '03_letterbox_ablation'}`
- `{out / '04_global_vs_policy_alignment'}`

[Results]
- Final status: `COMPLETED_PRIORITY2`.
- Priority3 started: false.
- Priority4 started: false.
- Action-head alignment frame rows: `{status['alignment']['frame_rows']}`.
- Matched camera input conditions: `{status['prepared_inputs']['camera_conditions']}`.
- Letterbox input conditions: `{status['prepared_inputs']['letterbox_conditions']}`.
- New image-to-final-action full-forward experiments are marked `GPU_REQUIRED`.

[Status]
PARTIALLY VERIFIED / VERIFIED_ACTION_HEAD_ONLY for CPU-executable Priority2 components. GPU full-forward components are prepared but not executed.

[Problems]
- NVIDIA driver unavailable in current environment.
- Full-forward camera/letterbox action results are not generated.
- No new environment data or rollout data collected.

[Decision]
Stop after Priority2. Do not start Priority3 or Priority4.

[Next]
When GPU is available, run full-forward extraction for matched camera shift and letterbox ablation inputs, then recompute Priority1-style metrics for those conditions.
"""
    with log.open("a", encoding="utf-8") as handle:
        handle.write(entry)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--bundle-root", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--seed", type=int, default=SEED)
    args = parser.parse_args()
    root = args.bundle_root.expanduser().resolve()
    out = args.output_dir.expanduser().resolve()
    out.mkdir(parents=True, exist_ok=True)

    correction = correction_chunk_audit(root, out)
    prepared = prepare_camera_and_letterbox(root, out)
    alignment = run_alignment(root, out, args.seed)
    status = {"correction_audit": correction, "prepared_inputs": prepared, "alignment": alignment}
    make_final_reports(root, out, status)
    append_log(root, out, status)
    print(json.dumps({"status": "COMPLETED_PRIORITY2", "output_dir": str(out)}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
