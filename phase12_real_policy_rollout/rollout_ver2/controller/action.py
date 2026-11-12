from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path

import numpy as np


@dataclass(frozen=True)
class ParsedAction:
    delta_position_m: np.ndarray
    rotation_vector_rad: np.ndarray
    gripper_binary: int
    gripper_target: float
    raw: np.ndarray
    endpoint_min: np.ndarray
    endpoint_max: np.ndarray

    @property
    def endpoint_any(self):
        return bool(np.any(self.endpoint_min | self.endpoint_max))


class ActionInterpreter:
    """Interpret already de-normalized server actions and audit endpoint bins."""

    NORMALIZED_BIN_CENTER = 1.0 - 1.0 / 255.0

    def __init__(self, statistics_path: str | Path, dataset_key="a0509_sim_cube_pick"):
        stats = json.loads(Path(statistics_path).read_text())[dataset_key]["action"]
        self.q01 = np.asarray(stats["q01"], dtype=float)
        self.q99 = np.asarray(stats["q99"], dtype=float)
        span = self.q99 - self.q01
        c = self.NORMALIZED_BIN_CENTER
        self.endpoint_min_values = self.q01 + ((-c + 1.0) / 2.0) * span
        self.endpoint_max_values = self.q01 + ((c + 1.0) / 2.0) * span

    def parse(self, raw_action) -> ParsedAction:
        raw = np.asarray(raw_action, dtype=float)
        if raw.shape != (7,) or not np.all(np.isfinite(raw)):
            raise ValueError("raw action must contain 7 finite values")
        atol = np.maximum(1e-12, np.abs(self.q99 - self.q01) * 1e-9)
        at_min = np.isclose(raw, self.endpoint_min_values, rtol=0.0, atol=atol)
        at_max = np.isclose(raw, self.endpoint_max_values, rtol=0.0, atol=atol)
        if not 0.0 <= raw[6] <= 1.0:
            raise ValueError("gripper absolute target must be in [0,1]")
        return ParsedAction(raw[:3].copy(), raw[3:6].copy(), int(raw[6] >= 0.5),
                            float(raw[6]), raw.copy(), at_min, at_max)
