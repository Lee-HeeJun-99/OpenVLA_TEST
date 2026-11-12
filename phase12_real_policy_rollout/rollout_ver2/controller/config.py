from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path

import numpy as np
from .joint_limits import load_joint_limits

CONFIG_PATH = Path(__file__).with_name("controller_config.json")


@dataclass(frozen=True)
class ControllerConfig:
    rotation_frame: str
    endpoint_policy: str
    rate_hz: float
    initial_duration_s: float
    duration_scale: float
    max_duration_s: float
    joint_lower_rad: np.ndarray
    joint_upper_rad: np.ndarray
    joint_margin_rad: np.ndarray
    max_joint_velocity_rad_s: np.ndarray
    max_joint_acceleration_rad_s2: np.ndarray
    max_sample_joint_change_rad: np.ndarray
    translation_step_m: float
    rotation_step_rad: float
    workspace_min_m: np.ndarray
    workspace_max_m: np.ndarray
    endpoint_streak_limit: int
    historical_j5_lower_rad: float
    historical_j5_upper_rad: float
    ik_position_tolerance_m: float
    ik_rotation_tolerance_rad: float
    jacobian_min_singular_value: float
    sim_home_joint_rad: np.ndarray
    gripper_mount_offset_j6_rad: float
    real_home_joint_rad: np.ndarray
    start_home_tolerance_rad: np.ndarray
    sim_joint_envelope_min_deg: np.ndarray
    sim_joint_envelope_max_deg: np.ndarray
    gripper_full_travel_duration_s: float

    @property
    def effective_lower_rad(self):
        return self.joint_lower_rad + self.joint_margin_rad

    @property
    def effective_upper_rad(self):
        return self.joint_upper_rad - self.joint_margin_rad


def load_config(path: str | Path = CONFIG_PATH, rotation_frame: str | None = None):
    raw = json.loads(Path(path).read_text())
    limits = load_joint_limits()
    frame = rotation_frame or raw["rotation_frame"]
    if frame not in {"world_left", "tool_right"}:
        raise ValueError("rotation_frame must be world_left or tool_right")
    rad = np.deg2rad
    endpoint_policy = raw.get("endpoint_policy", "log_only")
    if endpoint_policy not in {"log_only", "stop"}:
        raise ValueError("endpoint_policy must be log_only or stop")
    return ControllerConfig(
        rotation_frame=frame, endpoint_policy=endpoint_policy,
        rate_hz=float(raw["rate_hz"]),
        initial_duration_s=float(raw["initial_duration_s"]),
        duration_scale=float(raw["duration_scale"]),
        max_duration_s=float(raw["max_duration_s"]),
        joint_lower_rad=rad(limits.pendant_lower_deg),
        joint_upper_rad=rad(limits.pendant_upper_deg),
        joint_margin_rad=rad(limits.host_margin_deg),
        max_joint_velocity_rad_s=np.asarray(raw["max_joint_velocity_rad_s_candidate"]),
        max_joint_acceleration_rad_s2=np.asarray(raw["max_joint_acceleration_rad_s2_candidate"]),
        max_sample_joint_change_rad=rad(raw["max_sample_joint_change_deg_candidate"]),
        translation_step_m=float(raw["translation_step_m_candidate"]),
        rotation_step_rad=rad(float(raw["rotation_step_deg_candidate"])),
        workspace_min_m=np.asarray(raw["workspace_m_candidate"]["min"]),
        workspace_max_m=np.asarray(raw["workspace_m_candidate"]["max"]),
        endpoint_streak_limit=int(raw["endpoint_streak_limit_candidate"]),
        historical_j5_lower_rad=float(rad(raw["historical_j5_lower_deg_candidate"])),
        historical_j5_upper_rad=float(rad(raw["historical_j5_upper_deg_candidate"])),
        ik_position_tolerance_m=float(raw["ik_position_tolerance_m_candidate"]),
        ik_rotation_tolerance_rad=float(rad(raw["ik_rotation_tolerance_deg_candidate"])),
        jacobian_min_singular_value=float(raw["jacobian_min_singular_value_candidate"]),
        sim_home_joint_rad=rad(raw["sim_home_joint_deg"]),
        gripper_mount_offset_j6_rad=float(rad(raw["gripper_mount_offset_J6_deg"])),
        real_home_joint_rad=rad(raw["real_home_joint_deg"]),
        start_home_tolerance_rad=rad(raw["start_home_tolerance_deg_candidate"]),
        sim_joint_envelope_min_deg=np.asarray(raw["sim_joint_envelope_real_basis_deg"]["min"]),
        sim_joint_envelope_max_deg=np.asarray(raw["sim_joint_envelope_real_basis_deg"]["max"]),
        gripper_full_travel_duration_s=float(raw["gripper_full_travel_duration_s"]),
    )
