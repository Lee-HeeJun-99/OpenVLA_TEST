from __future__ import annotations

from typing import Iterable, Sequence

import numpy as np
from std_msgs.msg import Float64MultiArray


ACTION_DIM = 7
POSE_DIM = 6


def array_message(values: Iterable[float]) -> Float64MultiArray:
    msg = Float64MultiArray()
    msg.data = [float(v) for v in values]
    return msg


def checked_array(
    values: Sequence[float],
    expected_size: int,
    name: str,
) -> np.ndarray:
    array = np.asarray(values, dtype=np.float64)
    if array.shape != (expected_size,):
        raise ValueError(
            f"{name} must contain {expected_size} values, got shape {array.shape}"
        )
    if not np.all(np.isfinite(array)):
        raise ValueError(f"{name} contains NaN or Inf")
    return array
