from __future__ import annotations

from dataclasses import dataclass
import time
import numpy as np
from scipy.spatial.transform import Rotation, Slerp

from kinematics import DLSIKSolver, IKConvergenceError, Pose, PyKDLKinematics
from .config import ControllerConfig
from .precheck import joint_margin_reasons,singularity_reason


@dataclass(frozen=True)
class Segment:
    duration_s: float
    extension_count: int
    times_s: np.ndarray
    link_poses: tuple[Pose, ...]
    joints_rad: np.ndarray
    max_velocity_rad_s: np.ndarray
    max_acceleration_rad_s2: np.ndarray
    ik_sample_times_s: np.ndarray
    precompute_wall_s: float


class SegmentIKError(IKConvergenceError):
    def __init__(self, sample_index, target_link_pose, seed_rad, cause):
        super().__init__(f"segment sample {sample_index}: {cause}")
        self.sample_index = int(sample_index)
        self.target_link_pose = target_link_pose
        self.seed_rad = np.asarray(seed_rad, dtype=float).copy()
        self.cause = cause


class SegmentPreflightError(RuntimeError):
    def __init__(self,sample_index,reasons):
        super().__init__(f"segment preflight sample {sample_index}: {reasons}")
        self.sample_index=int(sample_index);self.reasons=tuple(reasons)


def minimum_jerk(u):
    return 10 * u**3 - 15 * u**4 + 6 * u**5


class SegmentGenerator:
    def __init__(self, kin: PyKDLKinematics, config: ControllerConfig, tool_transform=None):
        self.kin, self.config, self.tool_transform = kin, config, tool_transform
        # Aim at numerical precision for smooth seed continuity.  The looser
        # configured candidate is only a fallback acceptance criterion; using
        # it as the first exit criterion causes sample-to-sample joint chatter.
        self.ik = DLSIKSolver(kin, damping=1e-3, max_iterations=250,
                              position_tolerance_m=1e-8,
                              rotation_tolerance_rad=1e-8)
        self.fallback_ik = DLSIKSolver(
            kin, damping=1e-3, max_iterations=250,
            position_tolerance_m=config.ik_position_tolerance_m,
            rotation_tolerance_rad=config.ik_rotation_tolerance_rad)

    def _poses(self, start: Pose, target: Pose, duration):
        count = int(round(duration * self.config.rate_hz)) + 1
        times = np.linspace(0.0, duration, count)
        s = minimum_jerk(times / duration)
        rots = Slerp([0.0, 1.0], Rotation.from_matrix(
            np.stack([start.rotation, target.rotation])))(s).as_matrix()
        positions = start.position_m + s[:, None] * (target.position_m - start.position_m)
        return times, tuple(Pose(p, r) for p, r in zip(positions, rots))

    def _solve(self, poses, seed, lower, upper):
        q = np.asarray(seed, dtype=float).copy(); values = [];sample_times=[]
        for sample_index, pose in enumerate(poses):
            started=time.perf_counter()
            seed_at_sample = q.copy()
            try:
                q = self.ik.solve(pose, q, joint_lower_rad=lower, joint_upper_rad=upper,
                                  max_joint_change_rad=self.config.max_sample_joint_change_rad)
            except Exception as error:
                try:
                    q = self.fallback_ik.solve(
                        pose, q, joint_lower_rad=lower, joint_upper_rad=upper,
                        max_joint_change_rad=self.config.max_sample_joint_change_rad)
                except Exception:
                    raise SegmentIKError(sample_index, pose, seed_at_sample, error) from error
            values.append(q.copy())
            sample_times.append(time.perf_counter()-started)
        return np.asarray(values),np.asarray(sample_times)

    def generate(self, start: Pose, target: Pose, seed_rad, *, use_candidate_limits=True,
                 enforce_full_preflight=True):
        precompute_started=time.perf_counter()
        lower = self.config.effective_lower_rad if use_candidate_limits else self.kin.urdf_lower_rad
        upper = self.config.effective_upper_rad if use_candidate_limits else self.kin.urdf_upper_rad
        duration, extensions = self.config.initial_duration_s, 0
        while duration <= self.config.max_duration_s + 1e-12:
            times, command_poses = self._poses(start, target, duration)
            poses = (tuple(self.tool_transform.tool_to_link(x) for x in command_poses)
                     if self.tool_transform is not None else command_poses)
            joints,sample_times = self._solve(poses, seed_rad, lower, upper)
            dt = np.diff(times)
            velocity = np.diff(joints, axis=0) / dt[:, None]
            acceleration = (np.diff(velocity, axis=0) /
                            ((dt[1:] + dt[:-1]) / 2.0)[:, None])
            max_v = np.max(np.abs(velocity), axis=0)
            max_a = np.max(np.abs(acceleration), axis=0) if len(acceleration) else np.zeros(6)
            if (np.all(max_v <= self.config.max_joint_velocity_rad_s) and
                    np.all(max_a <= self.config.max_joint_acceleration_rad_s2)):
                if enforce_full_preflight:
                    for index,sample_q in enumerate(joints):
                        reasons=joint_margin_reasons(sample_q,self.config)
                        singular,_=singularity_reason(sample_q,self.kin,self.config)
                        if singular:reasons.append(singular)
                        if reasons:raise SegmentPreflightError(index,reasons)
                return Segment(duration, extensions, times, poses, joints, max_v, max_a,
                               sample_times,time.perf_counter()-precompute_started)
            duration *= self.config.duration_scale
            extensions += 1
        raise IKConvergenceError("DURATION_LIMIT_EXHAUSTED_WITHOUT_CANDIDATE_DYNAMICS_PASS")
