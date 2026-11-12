from __future__ import annotations

from dataclasses import dataclass
import numpy as np
from scipy.spatial.transform import Rotation

from kinematics import Pose


@dataclass(frozen=True)
class ToolTransform:
    position_link_m: np.ndarray
    rotation_link_tool: Rotation

    @classmethod
    def tool0_sim(cls):
        return cls(np.array([0.0, 0.0, 0.01]), Rotation.from_euler("z", -90, degrees=True))

    @classmethod
    def tool0_real(cls):
        # Confirmed physical mount: Rz(180 deg) * tool0_sim.
        return cls(np.array([0.0, 0.0, 0.01]), Rotation.from_euler("z", 90, degrees=True))

    def link_to_tool(self, link_pose: Pose) -> Pose:
        lr = Rotation.from_matrix(link_pose.rotation)
        return Pose(link_pose.position_m + lr.apply(self.position_link_m),
                    (lr * self.rotation_link_tool).as_matrix())

    def tool_to_link(self, tool_pose: Pose) -> Pose:
        tr = Rotation.from_matrix(tool_pose.rotation)
        lr = tr * self.rotation_link_tool.inv()
        return Pose(tool_pose.position_m - lr.apply(self.position_link_m), lr.as_matrix())


def apply_action(tool_pose: Pose, delta_p, omega, rotation_frame: str) -> Pose:
    r = Rotation.from_matrix(tool_pose.rotation)
    delta = Rotation.from_rotvec(np.asarray(omega, dtype=float))
    if rotation_frame == "world_left":
        target_r = delta * r
    elif rotation_frame == "tool_right":
        target_r = r * delta
    else:
        raise ValueError(f"unsupported rotation frame: {rotation_frame}")
    return Pose(tool_pose.position_m + np.asarray(delta_p, dtype=float), target_r.as_matrix())
