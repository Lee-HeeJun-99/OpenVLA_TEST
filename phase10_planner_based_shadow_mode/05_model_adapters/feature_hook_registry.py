#!/usr/bin/env python3
"""Architecture-aware hook labels without cross-model vector equivalence claims."""

from __future__ import annotations

from typing import Any


HOOK_SEMANTICS = (
    "vision_intermediate",
    "vision_output",
    "visual_projector_output",
    "vlm_visual_input",
    "multimodal_hidden",
    "action_hidden",
    "action_head_output",
)


def phase8_oft_hook_map(runtime: Any) -> dict[str, Any]:
    """Return Phase 8-compatible OFT hooks when exposed by a loaded runtime."""
    candidates = {
        "vision_output": getattr(runtime, "vision_backbone", None),
        "visual_projector_output": getattr(runtime, "projector", None),
        "action_hidden": getattr(runtime, "action_hidden_states", None),
        "action_head_output": getattr(runtime, "action_head", None),
    }
    return {name: module for name, module in candidates.items() if module is not None}


def validate_hook_set(model: str, hooks: dict[str, Any]) -> None:
    unknown = set(hooks) - set(HOOK_SEMANTICS)
    if unknown:
        raise ValueError(f"unknown {model} hook semantics: {sorted(unknown)}")
    if not hooks:
        raise ValueError(f"no feature hooks resolved for {model}")


def comparison_policy(left_model: str, right_model: str) -> str:
    if left_model == right_model:
        return "paired_within_model_metrics_allowed"
    return "no_raw_l2_use_cka_or_representational_similarity"

