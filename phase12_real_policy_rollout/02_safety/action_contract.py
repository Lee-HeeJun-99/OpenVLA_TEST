"""Offline action-contract conversions. This module has no ROS or command API."""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

import numpy as np
from scipy.spatial.transform import Rotation


M_TO_MM = 1000.0


@dataclass(frozen=True)
class TranslationConversion:
    delta_m: tuple[float, float, float]
    delta_mm: tuple[float, float, float]
    unit_scale: float = M_TO_MM
    empirical_gain: float = 1.0


def translation_m_to_mm(delta_m: Sequence[float], *, empirical_gain: float = 1.0) -> TranslationConversion:
    """Convert physical metres to millimetres; gains other than 1 are rejected.

    The historic runtime value 2800 mixed a 1000 mm/m unit conversion with an
    undocumented 2.8x control gain. It is not a checkpoint denormalization.
    """
    values = np.asarray(delta_m, dtype=np.float64)
    if values.shape != (3,) or not np.all(np.isfinite(values)):
        raise ValueError("delta_m must be three finite values")
    if not math.isclose(float(empirical_gain), 1.0, rel_tol=0.0, abs_tol=1e-12):
        raise RuntimeError("UNVERIFIED_EMPIRICAL_TRANSLATION_GAIN")
    converted = values * M_TO_MM
    return TranslationConversion(tuple(values), tuple(converted))


def doosan_zyz_deg_to_matrix(abc_deg: Sequence[float]) -> np.ndarray:
    """Decode Doosan A/B/C documented as intrinsic Euler ZYZ in degrees."""
    values = np.asarray(abc_deg, dtype=np.float64)
    if values.shape != (3,) or not np.all(np.isfinite(values)):
        raise ValueError("Doosan A/B/C must be three finite values")
    return Rotation.from_euler("ZYZ", values, degrees=True).as_matrix()


def matrix_to_doosan_zyz_deg(matrix: Sequence[Sequence[float]]) -> np.ndarray:
    """Encode a rotation matrix as an equivalent Doosan Euler-ZYZ triple.

    Euler values are non-unique at singularities; callers must compare matrices,
    not raw A/B/C values. No value returned here is authorized as a command.
    """
    value = np.asarray(matrix, dtype=np.float64)
    if value.shape != (3, 3) or not np.all(np.isfinite(value)):
        raise ValueError("matrix must be finite 3x3")
    return Rotation.from_matrix(value).as_euler("ZYZ", degrees=True)


def compose_world_rotvec_with_doosan_zyz(
    current_abc_deg: Sequence[float], delta_rotvec_rad: Sequence[float]
) -> tuple[np.ndarray, np.ndarray]:
    """Apply the dataset world/base-frame relative rotvec to Doosan ZYZ pose.

    Collector semantics use q_next * inverse(q_previous), and replay reconstructs
    with R_next = R_delta @ R_current. The returned pair is (ABC degrees, matrix)
    for offline validation only.
    """
    delta = np.asarray(delta_rotvec_rad, dtype=np.float64)
    if delta.shape != (3,) or not np.all(np.isfinite(delta)):
        raise ValueError("delta_rotvec_rad must be three finite values")
    current = doosan_zyz_deg_to_matrix(current_abc_deg)
    composed = Rotation.from_rotvec(delta).as_matrix() @ current
    return matrix_to_doosan_zyz_deg(composed), composed

