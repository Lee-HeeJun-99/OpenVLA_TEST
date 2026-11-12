from __future__ import annotations

from dataclasses import dataclass
import numpy as np


@dataclass
class ContinuousGripper:
    """Matched15 absolute target with concurrent, fixed-rate continuous slew."""

    rate_hz: float
    full_travel_duration_s: float
    command: float = 0.0
    target: float = 0.0

    def set_target(self, value: float) -> None:
        value = float(value)
        if not np.isfinite(value) or not 0.0 <= value <= 1.0:
            raise ValueError("GRIPPER_TARGET_OUT_OF_RANGE")
        self.target = value

    @property
    def max_delta_per_sample(self) -> float:
        return 1.0 / (self.rate_hz * self.full_travel_duration_s)

    def advance(self, sample_count: int) -> np.ndarray:
        output = []
        for _ in range(int(sample_count)):
            error = self.target - self.command
            if error == 0.0:
                output.append(self.command)
                continue
            delta = min(abs(error), self.max_delta_per_sample)
            self.command += np.copysign(delta, error)
            output.append(self.command)
        return np.asarray(output, dtype=float)
