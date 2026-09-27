#!/usr/bin/env python3
"""Phase 8 gripper-specific hidden sensitivity analysis.

For each aligned pair, compute a local gradient at the left hidden state using
only the gripper component of the right policy action chunk as target. Then
measure how much of the observed hidden delta lies in that gripper-sensitive
direction.
"""

from __future__ import annotations

import csv
import json
import os
import sys
from collections import defaultdict
from pathlib import Path
from typing import Any

import numpy as np
import torch
import torch.nn as nn


BUNDLE = Path("/home/ubuntu/a0509_vla_linux_field_bundle_20260903")
ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "11_gripper_sensitive_direction"
FEATURE_ROOT = ROOT / "06_representation_gap" / "full_forward_features"
CHECKPOINT = BUNDLE / "runtime_state" / "oft_mixed480_step28560_merged"
OFT_REPO = BUNDLE / "runtime" / "openvla-oft"
DATASET_KEY = "a0509_sim_cube_pick"
CHUNK = 5
ACTION_DIM = 7
GRIPPER_INDEX = 6


def read_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")


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


def f(row: dict[str, str], key: str, default: float = 0.0) -> float:
    value = row.get(key, "")
    if value == "" or value is None:
        return default
    return float(value)


def load_action_head(checkpoint: Path):
    os.environ.setdefault("A0509_ACTION_CHUNK_SIZE", str(CHUNK))
    training_config = read_json(checkpoint / "a0509_training_config.json")
    action_head = L1RegressionActionHeadLocal(
        input_dim=4096,
        hidden_dim=4096,
        action_dim=ACTION_DIM,
        bounded_gripper=bool(training_config.get("bounded_gripper", False)),
        gripper_loss_weight=float(training_config.get("gripper_loss_weight", 3.0)),
    )
    matches = sorted(checkpoint.glob("action_head*checkpoint*.pt"))
    if len(matches) != 1:
        raise FileNotFoundError(f"Expected one action_head checkpoint, found {len(matches)}")
    state = torch.load(matches[0], map_location="cpu", weights_only=True)
    state = {(k[7:] if k.startswith("module.") else k): v for k, v in state.items()}
    action_head.load_state_dict(state)
    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
    return action_head.to(device=device, dtype=torch.float32).eval(), device, str(matches[0])


class MLPResNetBlockLocal(nn.Module):
    def __init__(self, dim: int):
        super().__init__()
        self.ffn = nn.Sequential(nn.LayerNorm(dim), nn.Linear(dim, dim), nn.ReLU())

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.ffn(x) + x


class MLPResNetLocal(nn.Module):
    def __init__(self, num_blocks: int, input_dim: int, hidden_dim: int, output_dim: int):
        super().__init__()
        self.layer_norm1 = nn.LayerNorm(input_dim)
        self.fc1 = nn.Linear(input_dim, hidden_dim)
        self.relu = nn.ReLU()
        self.mlp_resnet_blocks = nn.ModuleList([MLPResNetBlockLocal(hidden_dim) for _ in range(num_blocks)])
        self.layer_norm2 = nn.LayerNorm(hidden_dim)
        self.fc2 = nn.Linear(hidden_dim, output_dim)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        x = self.relu(self.fc1(self.layer_norm1(x)))
        for block in self.mlp_resnet_blocks:
            x = block(x)
        return self.fc2(self.layer_norm2(x))


class L1RegressionActionHeadLocal(nn.Module):
    """Minimal local copy of the OFT L1RegressionActionHead.

    This avoids importing the whole `prismatic` package, which may require
    unrelated dependencies such as `huggingface_hub` in lightweight analysis
    environments.
    """

    def __init__(
        self,
        input_dim: int = 4096,
        hidden_dim: int = 4096,
        action_dim: int = 7,
        bounded_gripper: bool = False,
        gripper_loss_weight: float = 3.0,
    ):
        super().__init__()
        self.action_dim = action_dim
        self.bounded_gripper = bool(bounded_gripper)
        self.gripper_loss_weight = float(gripper_loss_weight)
        self.model = MLPResNetLocal(
            num_blocks=2,
            input_dim=input_dim * ACTION_DIM,
            hidden_dim=hidden_dim,
            output_dim=action_dim,
        )

    def predict_action(self, actions_hidden_states: torch.Tensor) -> torch.Tensor:
        batch_size = actions_hidden_states.shape[0]
        rearranged = actions_hidden_states.reshape(batch_size, CHUNK, -1)
        action = self.model(rearranged)
        if self.bounded_gripper:
            action = torch.cat([action[..., :-1], torch.sigmoid(action[..., -1:])], dim=-1)
        return action


def unnormalize_torch(normalized_actions: torch.Tensor, stats: dict[str, Any], device: torch.device) -> torch.Tensor:
    mask = torch.as_tensor(stats.get("mask", np.ones_like(stats["q01"], dtype=bool)), device=device, dtype=torch.bool)
    high = torch.as_tensor(stats["q99"], device=device, dtype=normalized_actions.dtype)
    low = torch.as_tensor(stats["q01"], device=device, dtype=normalized_actions.dtype)
    return torch.where(mask, 0.5 * (normalized_actions + 1.0) * (high - low + 1e-8) + low, normalized_actions)


def load_manifest(domain_dir: Path) -> dict[str, dict[str, Any]]:
    manifest_path = domain_dir / "feature_manifest.json"
    payload = read_json(manifest_path)
    out: dict[str, dict[str, Any]] = {}
    for rec in payload["records"]:
        image = str(Path(rec["source_image"]).resolve())
        feature_file = Path(rec["feature_file"])
        if not feature_file.is_absolute():
            feature_file = domain_dir / feature_file
        out[image] = {**rec, "_feature_path": feature_file.resolve()}
    return out


def hidden(record: dict[str, Any]) -> np.ndarray:
    with np.load(record["_feature_path"], allow_pickle=True) as z:
        return np.asarray(z["action_hidden_states.input"], dtype=np.float32)


def actions(record: dict[str, Any]) -> np.ndarray:
    response = record.get("response", {})
    arr = np.asarray(response.get("actions", [response.get("action")]), dtype=np.float32)
    if arr.shape != (CHUNK, ACTION_DIM):
        raise ValueError(f"Unexpected action shape: {arr.shape}")
    return arr


def gripper_grad(
    action_head: torch.nn.Module,
    left_hidden: np.ndarray,
    right_action: np.ndarray,
    action_stats: dict[str, Any],
    device: torch.device,
) -> tuple[np.ndarray, float, float]:
    tensor = torch.as_tensor(left_hidden, device=device, dtype=torch.float32).clone().detach().requires_grad_(True)
    pred_norm = action_head.predict_action(tensor).reshape(CHUNK, ACTION_DIM)
    pred = unnormalize_torch(pred_norm, action_stats, device)
    target = torch.as_tensor(right_action, device=device, dtype=torch.float32)
    diff = pred[:, GRIPPER_INDEX] - target[:, GRIPPER_INDEX]
    loss = 0.5 * torch.mean(diff * diff)
    loss.backward()
    grad = tensor.grad.detach().cpu().numpy().astype(np.float64)
    pred_grip = pred[:, GRIPPER_INDEX].detach().cpu().numpy().astype(np.float64)
    return grad, float(loss.detach().cpu().item()), float(np.mean(np.abs(pred_grip - right_action[:, GRIPPER_INDEX])))


def projection(delta: np.ndarray, direction: np.ndarray) -> dict[str, float]:
    flat_delta = delta.reshape(-1).astype(np.float64)
    flat_dir = direction.reshape(-1).astype(np.float64)
    delta_norm = float(np.linalg.norm(flat_delta))
    dir_norm = float(np.linalg.norm(flat_dir))
    if delta_norm <= 1e-12 or dir_norm <= 1e-12:
        return {
            "hidden_total_energy": delta_norm,
            "gripper_grad_norm": dir_norm,
            "gripper_sensitive_energy": 0.0,
            "gripper_null_energy": delta_norm,
            "gripper_sensitive_ratio": 0.0,
            "delta_cosine_with_negative_grad": 0.0,
            "signed_delta_dot_grad": 0.0,
        }
    unit = flat_dir / dir_norm
    coeff = float(np.dot(flat_delta, unit))
    sensitive = abs(coeff)
    null = float(max(delta_norm * delta_norm - sensitive * sensitive, 0.0) ** 0.5)
    return {
        "hidden_total_energy": delta_norm,
        "gripper_grad_norm": dir_norm,
        "gripper_sensitive_energy": sensitive,
        "gripper_null_energy": null,
        "gripper_sensitive_ratio": sensitive / delta_norm,
        "delta_cosine_with_negative_grad": -coeff / delta_norm,
        "signed_delta_dot_grad": float(np.dot(flat_delta, flat_dir)),
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
    return float(xs[lo] * (1.0 - frac) + xs[hi] * frac)


def aggregate(rows: list[dict[str, Any]], keys: list[str]) -> list[dict[str, Any]]:
    grouped: dict[tuple[Any, ...], list[dict[str, Any]]] = defaultdict(list)
    for row in rows:
        grouped[tuple(row[key] for key in keys)].append(row)
    metrics = [
        "hidden_total_energy",
        "gripper_grad_norm",
        "gripper_sensitive_energy",
        "gripper_null_energy",
        "gripper_sensitive_ratio",
        "delta_cosine_with_negative_grad",
        "gripper_loss_at_left",
        "gripper_abs_error_at_left",
        "chunk_mean_l2",
        "chunk_mean_translation_l2",
        "chunk_mean_rotation_l2",
        "chunk_mean_gripper_abs",
        "gripper_disagreement_rate",
    ]
    out = []
    for group, items in sorted(grouped.items(), key=lambda kv: kv[0]):
        rec = {k: v for k, v in zip(keys, group)}
        rec["count"] = len(items)
        for metric in metrics:
            vals = [float(item[metric]) for item in items]
            rec[f"{metric}_mean"] = mean(vals)
            rec[f"{metric}_median"] = percentile(vals, 50)
            rec[f"{metric}_p90"] = percentile(vals, 90)
        out.append(rec)
    return out


def report(overall: list[dict[str, Any]], by_phase: list[dict[str, Any]], action_head_checkpoint: str, device: str) -> str:
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

    real = [r for r in overall if str(r["comparison_id"]).startswith("real")]
    real_phase = [r for r in by_phase if str(r["comparison_id"]).startswith("real")]
    overall_fields = [
        ("comparison_id", "Comparison"),
        ("count", "N"),
        ("gripper_sensitive_energy_mean", "Grip-sensitive"),
        ("gripper_sensitive_ratio_mean", "Ratio"),
        ("delta_cosine_with_negative_grad_mean", "cos(delta,-grad)"),
        ("gripper_loss_at_left_mean", "Grip loss"),
        ("chunk_mean_l2_mean", "Action L2"),
        ("chunk_mean_gripper_abs_mean", "Grip gap"),
    ]
    phase_fields = [
        ("comparison_id", "Comparison"),
        ("phase", "Phase"),
        ("count", "N"),
        ("gripper_sensitive_ratio_mean", "Ratio"),
        ("delta_cosine_with_negative_grad_mean", "cos(delta,-grad)"),
        ("chunk_mean_l2_mean", "Action L2"),
        ("chunk_mean_gripper_abs_mean", "Grip gap"),
    ]
    return f"""# Phase 8 Gripper-Specific Sensitive Direction

## Purpose

This analysis computes a local hidden-space gradient using only the gripper component of the action chunk. It then measures how much of each Phase 8 hidden delta lies along that local gripper-sensitive direction.

## Method

For each aligned pair:

1. Use the left hidden state as the gradient point.
2. Use the right policy action chunk as the target.
3. Compute loss only on action dimension 6, the gripper output.
4. Compute the gradient of gripper loss with respect to `action_hidden_states.input`.
5. Project the observed hidden delta onto the gripper gradient direction.

Positive `cos(delta,-grad)` means the observed hidden delta points in the local direction that should reduce the gripper mismatch to the right-side action.

## Runtime

- Action head checkpoint: `{action_head_checkpoint}`
- Device: `{device}`
- Policy context: `oftplus_h5_vision`, checkpoint step 28560.

## Real-Only Overall Summary

{table(real, overall_fields)}

## Real-Only Phase Summary

{table(real_phase, phase_fields)}

## Interpretation

The comparison with the largest gripper-specific sensitive energy is the strongest candidate for a perturbation that changes the policy through gripper-sensitive hidden components.

This is more specific than the Phase 6 low-rank projection because it targets only the gripper output, which dominates Phase 8 action gap.

## Limitations

- This remains an offline local-gradient analysis, not a closed-loop rollout result.
- It uses right-side policy output as a disagreement target, not ground-truth correctness.
- A local gradient is a first-order approximation around the left hidden state.
"""


def main() -> int:
    OUT.mkdir(parents=True, exist_ok=True)
    action_stats = read_json(CHECKPOINT / "dataset_statistics.json")[DATASET_KEY]["action"]
    action_head, device, action_head_checkpoint = load_action_head(CHECKPOINT)

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

    rows: list[dict[str, Any]] = []
    missing: list[dict[str, Any]] = []
    for pair in aligned:
        if pair["pair_valid"] != "True":
            continue
        left_domain, right_domain = pair["left_domain"], pair["right_domain"]
        left_image = str(Path(pair["left_image_path"]).resolve())
        right_image = str(Path(pair["right_image_path"]).resolve())
        left_record = feature_maps[left_domain].get(left_image)
        right_record = feature_maps[right_domain].get(right_image)
        if left_record is None or right_record is None:
            missing.append({"comparison_id": pair["comparison_id"], "left_image": left_image, "right_image": right_image})
            continue
        left_hidden = hidden(left_record)
        right_hidden = hidden(right_record)
        right_action = actions(right_record)
        grad, loss, grip_abs = gripper_grad(action_head, left_hidden, right_action, action_stats, device)
        proj = projection(right_hidden - left_hidden, grad)
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
        rows.append(
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
                **proj,
                "gripper_loss_at_left": loss,
                "gripper_abs_error_at_left": grip_abs,
                "chunk_mean_l2": f(action, "chunk_mean_l2"),
                "chunk_mean_translation_l2": f(action, "chunk_mean_translation_l2"),
                "chunk_mean_rotation_l2": f(action, "chunk_mean_rotation_l2"),
                "chunk_mean_gripper_abs": f(action, "chunk_mean_gripper_abs"),
                "gripper_disagreement_rate": f(action, "gripper_disagreement_rate"),
            }
        )

    overall = aggregate(rows, ["comparison_id"])
    by_phase = aggregate(rows, ["comparison_id", "phase"])
    write_csv(OUT / "gripper_sensitive_frame_metrics.csv", rows)
    write_csv(OUT / "gripper_sensitive_summary_overall.csv", overall)
    write_csv(OUT / "gripper_sensitive_summary_by_phase.csv", by_phase)
    write_json(
        OUT / "summary.json",
        {
            "frame_rows": len(rows),
            "missing_feature_pairs": len(missing),
            "action_head_checkpoint": action_head_checkpoint,
            "device": str(device),
            "gripper_index": GRIPPER_INDEX,
            "status": "VERIFIED_OFFLINE_GRIPPER_LOCAL_GRADIENT" if not missing else "PARTIAL_MISSING_FEATURES",
            "outputs": {
                "frame_metrics": str(OUT / "gripper_sensitive_frame_metrics.csv"),
                "summary_overall": str(OUT / "gripper_sensitive_summary_overall.csv"),
                "summary_by_phase": str(OUT / "gripper_sensitive_summary_by_phase.csv"),
            },
        },
    )
    if missing:
        write_csv(OUT / "missing_feature_pairs.csv", missing)
    (OUT / "gripper_sensitive_report.md").write_text(report(overall, by_phase, action_head_checkpoint, str(device)), encoding="utf-8")
    print(json.dumps({"rows": len(rows), "missing": len(missing), "device": str(device)}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
