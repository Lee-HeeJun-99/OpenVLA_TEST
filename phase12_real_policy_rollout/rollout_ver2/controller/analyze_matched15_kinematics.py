#!/usr/bin/python3
"""Reconstruct matched15 arm joints from logged tool0 poses (joints were not logged)."""
from __future__ import annotations
import json
from pathlib import Path
import sys
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from controller.geometry import ToolTransform
from kinematics import DLSIKSolver,Pose,PyKDLKinematics

BUNDLE=Path("/home/ubuntu/a0509_vla_linux_field_bundle_20260903")
EVIDENCE=BUNDLE/"validation/evidence/vanilla/success_profile_s1_balanced_step8130_matched15_k1_seed9406_15ep.json"
HOME=BUNDLE/"isaac_sim/config/a0509_home_pose_openvla_legacy_repro.json"


def quat_wxyz_matrix(q):
    q=np.asarray(q,float);return Rotation.from_quat([q[1],q[2],q[3],q[0]]).as_matrix()


def main():
    kin=PyKDLKinematics(); tool=ToolTransform.tool0_sim()
    home=np.asarray(json.loads(HOME.read_text())["joint_positions_rad"],float)
    home_tool=tool.link_to_tool(kin.fk(home)); evidence=json.loads(EVIDENCE.read_text())
    joints=[]; sigmas=[]; residuals=[]; episode_rows=[]
    for episode in [x for x in evidence["results"] if x["success"]]:
        q=home.copy(); first=np.asarray(episode["actions"][0]["current_position_m"],float)
        world_base_translation=first-home_tool.position_m
        failures=0; local=[]
        for action in episode["actions"]:
            world_p=np.asarray(action["current_position_m"],float)
            target_tool=Pose(world_p-world_base_translation,
                             quat_wxyz_matrix(action["current_quaternion_wxyz"]))
            target_link=tool.tool_to_link(target_tool)
            fit=least_squares(lambda x:DLSIKSolver._error(kin.fk(x),target_link),q,
                bounds=(kin.urdf_lower_rad,kin.urdf_upper_rad),max_nfev=1000,
                xtol=1e-11,ftol=1e-11,gtol=1e-11)
            error=DLSIKSolver._error(kin.fk(fit.x),target_link)
            if np.linalg.norm(error[:3])>1e-4 or np.linalg.norm(error[3:])>1e-4: failures+=1
            q=fit.x; sigma=float(np.linalg.svd(kin.jacobian(q),compute_uv=False)[-1])
            joints.append(q.copy());sigmas.append(sigma);residuals.append(
                [float(np.linalg.norm(error[:3])),float(np.linalg.norm(error[3:]))])
            local.append(sigma)
        episode_rows.append({"episode_number":episode["episode_number"],
            "samples":len(local),"ik_residual_over_1e-4_count":failures,
            "world_base_translation_estimate_m":world_base_translation.tolist(),
            "sigma_min":float(np.min(local))})
    joints=np.asarray(joints);sigmas=np.asarray(sigmas);residuals=np.asarray(residuals)
    result={"status":"RECONSTRUCTED_NOT_DIRECTLY_LOGGED",
      "warning":"matched15 evidence omits joint targets; values are bounded IK reconstructions from logged current tool0 poses",
      "source":str(EVIDENCE),"home_seed":str(HOME),"samples":len(joints),
      "jacobian_min_singular_value":{"p0":float(np.percentile(sigmas,0)),
          "p1":float(np.percentile(sigmas,1)),"p5":float(np.percentile(sigmas,5)),
          "p50":float(np.percentile(sigmas,50))},
      "candidate_threshold_proposal":float(np.percentile(sigmas,0)*0.8),
      "candidate_basis":"80% of reconstructed successful-sample minimum; candidate only",
      "joint_min_deg":np.rad2deg(np.min(joints,axis=0)).tolist(),
      "joint_max_deg":np.rad2deg(np.max(joints,axis=0)).tolist(),
      "reconstruction_residual_m_max":float(np.max(residuals[:,0])),
      "reconstruction_residual_deg_max":float(np.rad2deg(np.max(residuals[:,1]))),
      "episodes":episode_rows}
    path=ROOT/"reports/matched15_kinematics_reconstructed.json"
    path.write_text(json.dumps(result,indent=2)+"\n");print(path)


if __name__=="__main__":main()
