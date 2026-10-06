"""Single canonical action representation shared by OpenVLA and OFT."""
from __future__ import annotations
from dataclasses import dataclass
import math
from typing import Sequence


@dataclass(frozen=True)
class CanonicalAction:
    translation_m: tuple[float, float, float]
    rotation_rotvec_rad: tuple[float, float, float]
    gripper_closedness: float
    timestamp_monotonic: float
    sequence_id: str
    source_model: str
    chunk_index: int = 0
    chunk_size: int = 1

    def __post_init__(self):
        values = (*self.translation_m, *self.rotation_rotvec_rad,
                  self.gripper_closedness, self.timestamp_monotonic)
        if len(self.translation_m) != 3 or len(self.rotation_rotvec_rad) != 3:
            raise ValueError("canonical_action_requires_3d_translation_and_rotation")
        if not all(math.isfinite(float(v)) for v in values):
            raise ValueError("canonical_action_nonfinite")
        if not 0.0 <= float(self.gripper_closedness) <= 1.0:
            raise ValueError("gripper_closedness_out_of_range")
        if self.source_model not in {"openvla", "oft", "recorded", "mock"}:
            raise ValueError("unsupported_source_model")
        if not self.sequence_id:
            raise ValueError("sequence_id_required")
        if self.chunk_size < 1 or not 0 <= self.chunk_index < self.chunk_size:
            raise ValueError("invalid_chunk_index")

    @classmethod
    def from_vector(cls, values: Sequence[float], *, timestamp_monotonic: float,
                    sequence_id: str, source_model: str, chunk_index: int = 0,
                    chunk_size: int = 1) -> "CanonicalAction":
        if len(values) != 7:
            raise ValueError("canonical_action_requires_7_values")
        v = tuple(float(x) for x in values)
        return cls(v[:3], v[3:6], v[6], float(timestamp_monotonic),
                   str(sequence_id), str(source_model), int(chunk_index), int(chunk_size))

    def as_vector(self) -> tuple[float, ...]:
        return (*self.translation_m, *self.rotation_rotvec_rad,
                float(self.gripper_closedness))

    def as_dict(self) -> dict:
        return {
            "translation_m": list(self.translation_m),
            "rotation_rotvec_rad": list(self.rotation_rotvec_rad),
            "gripper_closedness": self.gripper_closedness,
            "timestamp_monotonic": self.timestamp_monotonic,
            "sequence_id": self.sequence_id,
            "source_model": self.source_model,
            "chunk_index": self.chunk_index,
            "chunk_size": self.chunk_size,
        }

