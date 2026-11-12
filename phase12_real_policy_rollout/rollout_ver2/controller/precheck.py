from __future__ import annotations

from dataclasses import dataclass, field
import numpy as np

from .config import ControllerConfig
from .action import ParsedAction


@dataclass
class EndpointStreak:
    """Track endpoint-bin streaks for arm action dimensions only.

    Dimension 6 is an absolute binary gripper target and is deliberately not a
    distribution-departure signal.
    """
    counts: np.ndarray = field(default_factory=lambda: np.zeros((6, 2), dtype=int))

    def update(self, action: ParsedAction):
        self.counts[:, 0] = np.where(action.endpoint_min[:6], self.counts[:, 0] + 1, 0)
        self.counts[:, 1] = np.where(action.endpoint_max[:6], self.counts[:, 1] + 1, 0)
        return self.counts.copy()


@dataclass
class GripperStateTracker:
    previous: int | None = None

    def update(self, state: int) -> bool:
        state = int(state)
        changed = self.previous is not None and state != self.previous
        self.previous = state
        return changed


def action_reasons(action: ParsedAction, target_tool_position, config: ControllerConfig,
                   streak_counts):
    reasons = []
    if np.linalg.norm(action.delta_position_m) > config.translation_step_m:
        reasons.append("TRANSLATION_STEP_CANDIDATE_EXCEEDED")
    if np.linalg.norm(action.rotation_vector_rad) > config.rotation_step_rad:
        reasons.append("ROTATION_STEP_CANDIDATE_EXCEEDED")
    p = np.asarray(target_tool_position)
    if np.any(p < config.workspace_min_m) or np.any(p > config.workspace_max_m):
        reasons.append("WORKSPACE_CANDIDATE_EXCEEDED")
    if (config.endpoint_policy == "stop" and
            np.any(streak_counts >= config.endpoint_streak_limit)):
        dims = np.argwhere(streak_counts >= config.endpoint_streak_limit).tolist()
        reasons.append(f"ENDPOINT_TOKEN_STREAK:{dims}")
    return reasons


def singularity_reason(q, kinematics, config):
    singular_values = np.linalg.svd(kinematics.jacobian(q), compute_uv=False)
    minimum = float(singular_values[-1])
    if minimum < config.jacobian_min_singular_value:
        return "SINGULARITY_APPROACH", minimum
    return None, minimum


def singularity_family(q):
    """Diagnostic family label; it does not change or project the action."""
    q = np.asarray(q, dtype=float)
    if abs(q[2]) <= np.deg2rad(5.0):
        return "ELBOW_EXTENSION_J3"
    if abs(np.sin(q[4])) <= np.sin(np.deg2rad(5.0)):
        return "WRIST_J5_ZERO"
    return "SHOULDER_OR_OTHER"


def joint_margin_reasons(q, config):
    q = np.asarray(q)
    bad = np.flatnonzero((q < config.effective_lower_rad) | (q > config.effective_upper_rad))
    return [f"JOINT_LIMIT_MARGIN:J{i+1}" for i in bad]


def joint_envelope_status(q, config):
    """Log-only distance outside the observed Isaac real-basis envelope."""
    q_deg=np.rad2deg(np.asarray(q,dtype=float))
    below=np.maximum(config.sim_joint_envelope_min_deg-q_deg,0.0)
    above=np.maximum(q_deg-config.sim_joint_envelope_max_deg,0.0)
    excess=np.maximum(below,above)
    return {"outside":bool(np.any(excess>0)),"excess_deg":excess.tolist(),
            "joints":[int(i+1) for i in np.flatnonzero(excess>0)]}


def starting_home_reasons(q, config):
    """Exact joint-branch check; no angle wrapping or automatic motion."""
    q = np.asarray(q, dtype=float)
    error = np.abs(q - config.real_home_joint_rad)
    return [f"START_NOT_REAL_HOME:J{i+1}" for i in
            np.flatnonzero(error > config.start_home_tolerance_rad)]
