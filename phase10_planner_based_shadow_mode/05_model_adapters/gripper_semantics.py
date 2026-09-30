#!/usr/bin/env python3
"""Pure copied gripper semantics for offline parity tests; no ROS imports."""
from __future__ import annotations
from typing import Iterable


def dataset_closedness(value: float, threshold: float = 0.5) -> bool:
    """Dataset/model contract: 0=open, 1=closed."""
    return float(value) >= float(threshold)


def runtime_gripper_open(value: float, last_open: bool | None,
                         open_threshold: float = 0.7,
                         close_threshold: float = 0.3) -> bool:
    """Copied from ActionAdapterNode._resolve_gripper_state."""
    value=float(value)
    if value >= open_threshold: return True
    if value <= close_threshold: return False
    return True if last_open is None else bool(last_open)


def runtime_sequence(values: Iterable[float], initial_open: bool | None = None) -> list[bool]:
    last=initial_open; result=[]
    for value in values:
        last=runtime_gripper_open(value,last); result.append(last)
    return result


def runtime_oft_action(chunk: list[list[float]]) -> list[float]:
    """Copied from OpenVLAOFTInferenceNode: validate K=5 then return chunk[0]."""
    if len(chunk)!=5 or any(len(action)!=7 for action in chunk):
        raise ValueError("runtime requires a 5x7 action chunk")
    return [float(v) for v in chunk[0]]
