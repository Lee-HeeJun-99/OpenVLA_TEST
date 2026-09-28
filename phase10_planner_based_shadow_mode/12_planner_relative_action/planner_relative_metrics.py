#!/usr/bin/env python3
"""Planner-relative metrics with explicit equal-horizon comparisons."""

from __future__ import annotations

import math
from typing import Sequence


def metrics(model: Sequence[float], planner: Sequence[float]) -> dict[str, float | None]:
    if len(model) != 7 or len(planner) != 7:
        raise ValueError("both actions must be canonical 7-vectors")
    delta = [float(a) - float(b) for a, b in zip(model, planner)]
    dot = sum(float(a) * float(b) for a, b in zip(model, planner))
    norms = math.sqrt(sum(float(a) ** 2 for a in model) * sum(float(b) ** 2 for b in planner))
    return {
        "l1": sum(abs(value) for value in delta),
        "l2": math.sqrt(sum(value * value for value in delta)),
        "rmse": math.sqrt(sum(value * value for value in delta) / 7),
        "translation_l2": math.sqrt(sum(value * value for value in delta[:3])),
        "rotation_l2": math.sqrt(sum(value * value for value in delta[3:6])),
        "gripper_abs": abs(delta[6]),
        "cosine_distance": None if norms <= 1e-12 else 1.0 - dot / norms,
        "direction_agreement_translation": sum(float(a) * float(b) for a, b in zip(model[:3], planner[:3])),
        "magnitude_error_translation": abs(
            math.sqrt(sum(float(a) ** 2 for a in model[:3]))
            - math.sqrt(sum(float(b) ** 2 for b in planner[:3]))
        ),
    }

