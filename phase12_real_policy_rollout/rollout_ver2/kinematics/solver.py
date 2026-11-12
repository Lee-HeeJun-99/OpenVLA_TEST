"""PyKDL FK and local damped-least-squares IK for the A0509.

All internal quantities are SI: radians and metres.  The module is deliberately
offline-only: it has no rclpy dependency and contains no command publisher.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Sequence
import xml.etree.ElementTree as ET

import numpy as np
import PyKDL
from scipy.spatial.transform import Rotation

DEFAULT_URDF_PATH = Path(
    "/home/ubuntu/robot_ws/src/doosan-robot2/dsr_description2/urdf/a0509.urdf"
)


class KinematicsError(RuntimeError):
    pass


class IKConvergenceError(KinematicsError):
    def __init__(self, message, *, iterations=None, best_residual=None,
                 best_joint_rad=None):
        super().__init__(message)
        self.iterations = iterations
        self.best_residual = best_residual
        self.best_joint_rad = best_joint_rad


class IKJointChangeError(KinematicsError):
    pass


class JointLimitError(KinematicsError):
    pass


def deg_to_rad(value: Sequence[float] | np.ndarray) -> np.ndarray:
    return np.deg2rad(np.asarray(value, dtype=float))


def rad_to_deg(value: Sequence[float] | np.ndarray) -> np.ndarray:
    return np.rad2deg(np.asarray(value, dtype=float))


@dataclass(frozen=True)
class Pose:
    position_m: np.ndarray
    rotation: np.ndarray

    def __post_init__(self) -> None:
        p = np.asarray(self.position_m, dtype=float)
        r = np.asarray(self.rotation, dtype=float)
        if p.shape != (3,) or r.shape != (3, 3):
            raise ValueError("Pose requires position (3,) and rotation (3,3)")
        object.__setattr__(self, "position_m", p.copy())
        object.__setattr__(self, "rotation", r.copy())

    @property
    def matrix(self) -> np.ndarray:
        out = np.eye(4)
        out[:3, :3] = self.rotation
        out[:3, 3] = self.position_m
        return out


def _xyz(text: str | None) -> np.ndarray:
    return np.fromstring(text or "0 0 0", sep=" ", dtype=float)


def _frame(xyz: np.ndarray, rpy: np.ndarray) -> PyKDL.Frame:
    return PyKDL.Frame(
        PyKDL.Rotation.RPY(float(rpy[0]), float(rpy[1]), float(rpy[2])),
        PyKDL.Vector(*map(float, xyz)),
    )


class PyKDLKinematics:
    """A serial URDF chain represented by PyKDL."""

    def __init__(
        self,
        urdf_path: str | Path = DEFAULT_URDF_PATH,
        base_link: str = "base_link",
        tip_link: str = "link_6",
    ) -> None:
        self.urdf_path = Path(urdf_path)
        root = ET.parse(self.urdf_path).getroot()
        joints = {}
        child_to_joint = {}
        for node in root.findall("joint"):
            parent = node.find("parent").get("link")
            child = node.find("child").get("link")
            joints[node.get("name")] = node
            child_to_joint[child] = (parent, node)

        ordered = []
        cursor = tip_link
        while cursor != base_link:
            if cursor not in child_to_joint:
                raise ValueError(f"No URDF chain from {base_link!r} to {tip_link!r}")
            parent, joint = child_to_joint[cursor]
            ordered.append((cursor, joint))
            cursor = parent
        ordered.reverse()

        self.chain = PyKDL.Chain()
        names, lower, upper = [], [], []
        for child, node in ordered:
            origin = node.find("origin")
            xyz = _xyz(origin.get("xyz") if origin is not None else None)
            rpy = _xyz(origin.get("rpy") if origin is not None else None)
            fixed = node.get("type") == "fixed"
            if fixed:
                joint = PyKDL.Joint(node.get("name"), PyKDL.Joint.Fixed)
            else:
                axis_local = _xyz(node.find("axis").get("xyz"))
                origin_rot = Rotation.from_euler("xyz", rpy).as_matrix()
                axis_parent = origin_rot @ axis_local
                joint = PyKDL.Joint(
                    node.get("name"), PyKDL.Vector(*map(float, xyz)),
                    PyKDL.Vector(*map(float, axis_parent)), PyKDL.Joint.RotAxis,
                )
                limit = node.find("limit")
                names.append(node.get("name"))
                lower.append(float(limit.get("lower")))
                upper.append(float(limit.get("upper")))
            self.chain.addSegment(PyKDL.Segment(child, joint, _frame(xyz, rpy)))

        self.base_link = base_link
        self.tip_link = tip_link
        self.joint_names = tuple(names)
        self.urdf_lower_rad = np.asarray(lower)
        self.urdf_upper_rad = np.asarray(upper)
        self._fk = PyKDL.ChainFkSolverPos_recursive(self.chain)
        self._jac = PyKDL.ChainJntToJacSolver(self.chain)

    @property
    def dof(self) -> int:
        return self.chain.getNrOfJoints()

    def _joints(self, q_rad: Sequence[float]) -> PyKDL.JntArray:
        q = np.asarray(q_rad, dtype=float)
        if q.shape != (self.dof,) or not np.all(np.isfinite(q)):
            raise ValueError(f"Expected {self.dof} finite joint values in radians")
        out = PyKDL.JntArray(self.dof)
        for i, value in enumerate(q):
            out[i] = float(value)
        return out

    def fk(self, q_rad: Sequence[float]) -> Pose:
        frame = PyKDL.Frame()
        rc = self._fk.JntToCart(self._joints(q_rad), frame)
        if rc < 0:
            raise KinematicsError(f"PyKDL FK failed with code {rc}")
        rot = np.array([[frame.M[i, j] for j in range(3)] for i in range(3)])
        return Pose(np.array([frame.p[i] for i in range(3)]), rot)

    def jacobian(self, q_rad: Sequence[float]) -> np.ndarray:
        jac = PyKDL.Jacobian(self.dof)
        rc = self._jac.JntToJac(self._joints(q_rad), jac)
        if rc < 0:
            raise KinematicsError(f"PyKDL Jacobian failed with code {rc}")
        return np.array([[jac[i, j] for j in range(self.dof)] for i in range(6)])


class DLSIKSolver:
    """Local DLS IK; the seed is the previous commanded joint position.

    No safety margin is invented here.  Callers must provide the verified joint
    limits and the maximum accepted seed-to-solution change for every solve.
    A rejected solution is never clamped or modified.
    """

    def __init__(
        self,
        kinematics: PyKDLKinematics,
        damping: float = 1e-3,
        max_iterations: int = 200,
        position_tolerance_m: float = 1e-6,
        rotation_tolerance_rad: float = 1e-6,
    ) -> None:
        if damping <= 0 or max_iterations <= 0:
            raise ValueError("damping and max_iterations must be positive")
        self.kin = kinematics
        self.damping = float(damping)
        self.max_iterations = int(max_iterations)
        self.position_tolerance_m = float(position_tolerance_m)
        self.rotation_tolerance_rad = float(rotation_tolerance_rad)

    @staticmethod
    def _error(current: Pose, target: Pose) -> np.ndarray:
        # Geometric Jacobian angular rows are expressed in the world/base frame.
        rot_world = Rotation.from_matrix(target.rotation @ current.rotation.T).as_rotvec()
        return np.r_[target.position_m - current.position_m, rot_world]

    def solve(
        self,
        target: Pose,
        seed_rad: Sequence[float],
        *,
        joint_lower_rad: Sequence[float],
        joint_upper_rad: Sequence[float],
        max_joint_change_rad: float | Sequence[float],
    ) -> np.ndarray:
        seed = np.asarray(seed_rad, dtype=float)
        lo = np.asarray(joint_lower_rad, dtype=float)
        hi = np.asarray(joint_upper_rad, dtype=float)
        max_change = np.broadcast_to(np.asarray(max_joint_change_rad, dtype=float), seed.shape)
        if seed.shape != (self.kin.dof,) or lo.shape != seed.shape or hi.shape != seed.shape:
            raise ValueError("seed and joint limits must match chain DOF")
        if np.any(lo >= hi) or np.any(max_change <= 0):
            raise ValueError("Invalid joint limits or max_joint_change_rad")
        if np.any(seed < lo) or np.any(seed > hi):
            raise JointLimitError("IK seed is outside supplied verified limits")

        q = seed.copy()
        best_q, best_norm = q.copy(), np.inf
        iterations_run = 0
        for iteration in range(self.max_iterations):
            iterations_run = iteration + 1
            error = self._error(self.kin.fk(q), target)
            score = float(np.linalg.norm(error))
            if score < best_norm:
                best_q, best_norm = q.copy(), score
            if (np.linalg.norm(error[:3]) <= self.position_tolerance_m and
                    np.linalg.norm(error[3:]) <= self.rotation_tolerance_rad):
                change = np.abs(q - seed)
                if np.any(change > max_change):
                    raise IKJointChangeError(
                        f"IK change exceeds caller limit: {change.tolist()} rad"
                    )
                return q
            jac = self.kin.jacobian(q)
            dq = jac.T @ np.linalg.solve(
                jac @ jac.T + (self.damping ** 2) * np.eye(6), error
            )
            # Backtracking is a numerical convergence device, not action clamping.
            accepted = False
            for scale in (1.0, 0.5, 0.25, 0.125, 0.0625, 0.03125):
                candidate = q + scale * dq
                if np.any(candidate < lo) or np.any(candidate > hi):
                    continue
                candidate_norm = np.linalg.norm(self._error(self.kin.fk(candidate), target))
                if candidate_norm < score:
                    q, accepted = candidate, True
                    break
            if not accepted:
                break
        raise IKConvergenceError(
            f"DLS IK did not converge in {self.max_iterations} iterations; "
            f"best combined residual={best_norm:.6g} (mixed m/rad norm)",
            iterations=iterations_run, best_residual=best_norm,
            best_joint_rad=best_q.copy(),
        )
