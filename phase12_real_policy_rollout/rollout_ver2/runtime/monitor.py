from __future__ import annotations
import numpy as np
from scipy.spatial.transform import Rotation
from controller.precheck import (joint_margin_reasons, singularity_reason,
                                 starting_home_reasons,joint_envelope_status)
from runtime import robot_state as rs


class RuntimeMonitor:
    def __init__(self, feedback, kinematics, controller_config, runtime_config):
        self.feedback=feedback; self.kin=kinematics
        self.cc=controller_config; self.rc=runtime_config

    def check(self, commanded_tool_pose, now=None):
        reasons=self.feedback.stale_reasons(now)
        try: snapshot=self.feedback.snapshot(now)
        except RuntimeError as error: return reasons+[str(error)], None
        if snapshot.robot_state not in rs.ALLOWED_RUNNING_STATES:
            reasons.append(f"ROBOT_STATE_NOT_RUNNING:{snapshot.robot_state}")
        reasons.extend(joint_margin_reasons(snapshot.joint_rad, self.cc))
        singular, sigma=singularity_reason(snapshot.joint_rad, self.kin, self.cc)
        if singular: reasons.append(singular)
        if commanded_tool_pose is not None:
            position=np.linalg.norm(snapshot.tool0_real_pose.position_m-commanded_tool_pose.position_m)
            angle=Rotation.from_matrix(snapshot.tool0_real_pose.rotation @ commanded_tool_pose.rotation.T).magnitude()
            if position > float(self.rc["tracking_position_m_candidate"]):
                reasons.append("TRACKING_POSITION_CANDIDATE_EXCEEDED")
            if angle > np.deg2rad(float(self.rc["tracking_rotation_deg_candidate"])):
                reasons.append("TRACKING_ROTATION_CANDIDATE_EXCEEDED")
        return reasons, {"snapshot":snapshot, "minimum_singular_value":sigma,
                         "sim_joint_envelope_log_only":joint_envelope_status(
                             snapshot.joint_rad,self.cc)}

    def check_start(self, now=None):
        """One-shot gate before any command; it never moves the robot."""
        reasons,snapshot=self.feedback.preflight_reasons(now)
        if snapshot is None:return reasons,None
        reasons.extend(starting_home_reasons(snapshot.joint_rad,self.cc))
        return reasons,snapshot
