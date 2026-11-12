#!/usr/bin/python3
"""Offline-only comparison of actual Isaac joints and recorded real starts."""
import json,glob,sys
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path: sys.path.insert(0,str(ROOT))
from kinematics import PyKDLKinematics
from controller.geometry import ToolTransform

BUNDLE=Path("/home/ubuntu/a0509_vla_linux_field_bundle_20260903")
SIM= BUNDLE/"lhj/phase11_sim_condition_replication/dataset/episodes/baseline"
REAL=BUNDLE/"lhj/phase12_real_policy_rollout/14_real_rollout_integration/real_trials"
HOME=BUNDLE/"isaac_sim/config/a0509_home_pose.json"

def ordered(sample):
    by=dict(zip(sample["names"],sample["position"]))
    return np.array([by[f"joint_{i}"] for i in range(1,7)])

def main():
    home=np.array(json.loads(HOME.read_text())["joint_positions_rad"])
    sim=[]; sim_files=[]
    for path in sorted(SIM.glob("baseline_episode_*.json")):
        rows=json.loads(path.read_text())["records"]
        values=[r["actual_joint_position_rad"][:6] for r in rows]
        sim.extend(values);sim_files.append({"path":str(path),"samples":len(values)})
    sim=np.asarray(sim); kin=PyKDLKinematics()
    tool_sim=ToolTransform.tool0_sim(); tool_real=ToolTransform.tool0_real()
    sigma=np.array([np.linalg.svd(kin.jacobian(q),compute_uv=False)[-1] for q in sim])
    real_home=home.copy();real_home[5]+=np.pi
    home_pose=tool_sim.link_to_tool(kin.fk(home)); runs=[]
    for path in sorted(REAL.glob("20261112_*_arm_only_servo_episode/jointstate.json")):
        samples=[x for x in json.loads(path.read_text())["samples"]
                 if x.get("valid") and len(x.get("position",[]))>=6]
        if not samples: continue
        q=ordered(samples[0]); delta=np.rad2deg(q-real_home)
        pose=tool_real.link_to_tool(kin.fk(q))
        position=float(np.linalg.norm(pose.position_m-home_pose.position_m)*1000)
        rotation=float(Rotation.from_matrix(pose.rotation@home_pose.rotation.T).magnitude()*180/np.pi)
        passed=bool(np.all(np.abs(delta)<=2))
        classification="PASS_REAL_HOME_2DEG" if passed else "REJECT_REAL_HOME_2DEG"
        runs.append({"run":path.parent.name,"start_joint_deg":np.rad2deg(q).tolist(),
                     "delta_from_real_home_deg":delta.tolist(),"classification":classification,
                     "tool0_real_vs_tool0_sim":{"position_mm":position,"rotation_deg":rotation}})
    out={"status":"OFFLINE_NO_ROBOT_IO","home_source":str(HOME),
         "sim_home_joint_deg":np.rad2deg(home).tolist(),"sim_replay_sources":sim_files,
         "gripper_mount_offset_J6_deg":180.0,
         "real_home_joint_deg":np.rad2deg(real_home).tolist(),
         "sim_actual_sample_count":len(sim),
         "sim_actual_jacobian_sigma_min":{"p0":float(np.percentile(sigma,0)),
            "p1":float(np.percentile(sigma,1)),"p5":float(np.percentile(sigma,5))},
         "sim_actual_joint_range_deg":{"min":np.rad2deg(sim.min(0)).tolist(),
                                       "max":np.rad2deg(sim.max(0)).tolist()},
         "real_reference_joint_range_deg":{"min":np.rad2deg(sim.min(0)+[0,0,0,0,0,np.pi]).tolist(),
             "max":np.rad2deg(sim.max(0)+[0,0,0,0,0,np.pi]).tolist()},
         "wrist_range_deg":{"J4":[float(np.rad2deg(sim[:,3].min())),float(np.rad2deg(sim[:,3].max()))],
                            "J5":[float(np.rad2deg(sim[:,4].min())),float(np.rad2deg(sim[:,4].max()))],
                            "J6":[float(np.rad2deg(sim[:,5].min())),float(np.rad2deg(sim[:,5].max()))]},
         "real_v1_starts":runs,"start_tolerance_candidate_deg":[2.0]*6,
         "automatic_home_motion":False}
    (ROOT/"reports/home_alignment.json").write_text(json.dumps(out,indent=2)+"\n")

if __name__=="__main__":main()
