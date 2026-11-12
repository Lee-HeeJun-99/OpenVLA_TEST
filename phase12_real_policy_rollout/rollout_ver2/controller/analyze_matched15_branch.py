#!/usr/bin/python3
"""Continuous, sim-home-seeded matched15 IK reconstruction; offline only."""
import json,sys,time
from pathlib import Path
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from controller.geometry import ToolTransform
from kinematics import DLSIKSolver,Pose,PyKDLKinematics

BUNDLE=Path('/home/ubuntu/a0509_vla_linux_field_bundle_20260903')
EVIDENCE=BUNDLE/'validation/evidence/vanilla/success_profile_s1_balanced_step8130_matched15_k1_seed9406_15ep.json'
HOME=BUNDLE/'isaac_sim/config/a0509_home_pose.json'
SIM=BUNDLE/'lhj/phase11_sim_condition_replication/dataset/episodes/baseline'
WORLD_BASE_P=np.array([-0.000389495411757726,-0.0002573428604871621,0.8235262353883922])
WORLD_BASE_R=Rotation.from_euler('xyz',[0.004445488457065276,0.016366501042746482,-0.0038822342347884717],degrees=True)

def rot(q):q=np.asarray(q);return Rotation.from_quat(q[[1,2,3,0]])
def summary(a):
    a=np.asarray(a);return {f'p{x}':float(np.percentile(a,x)) for x in (0,1,5,50,99,100)}

def main():
    kin=PyKDLKinematics();tool=ToolTransform.tool0_sim();home=np.array(json.loads(HOME.read_text())['joint_positions_rad'])
    evidence=json.loads(EVIDENCE.read_text());all_q=[];all_world=[];times=[];episodes=[];residual=[]
    for ep in evidence['results']:
        q=home.copy();local=[];failed=0
        for i,a in enumerate(ep['actions']):
            rw=rot(a['current_quaternion_wxyz']);pw=np.array(a['current_position_m'])
            base_tool=Pose(WORLD_BASE_R.inv().apply(pw-WORLD_BASE_P),(WORLD_BASE_R.inv()*rw).as_matrix())
            target=tool.tool_to_link(base_tool);started=time.perf_counter()
            fit=least_squares(lambda x:DLSIKSolver._error(kin.fk(x),target),q,
                bounds=(kin.urdf_lower_rad,kin.urdf_upper_rad),max_nfev=1500,
                xtol=1e-11,ftol=1e-11,gtol=1e-11)
            times.append(time.perf_counter()-started);err=DLSIKSolver._error(kin.fk(fit.x),target)
            if np.linalg.norm(err[:3])>5e-5 or np.linalg.norm(err[3:])>np.deg2rad(.005):failed+=1
            q=fit.x;local.append(q.copy());all_q.append(q.copy());all_world.append((pw,rw));residual.append(err)
        a=np.array(local);episodes.append({'episode_number':ep['episode_number'],'samples':len(a),
          'failures':failed,'first_joint_deg':np.rad2deg(a[0]).tolist(),
          'j5_range_deg':np.rad2deg([a[:,4].min(),a[:,4].max()]).tolist(),
          'j6_range_deg':np.rad2deg([a[:,5].min(),a[:,5].max()]).tolist()})
    q=np.array(all_q);residual=np.array(residual);sigma=np.array([np.linalg.svd(kin.jacobian(x),compute_uv=False)[-1] for x in q])
    actual=[]
    for p in sorted(SIM.glob('baseline_episode_*.json')):
        for row in json.loads(p.read_text())['records']:
            pose=row['sim_end_effector_pose'];actual.append((np.array(row['actual_joint_position_rad'][:6]),
                np.array(pose['position_m']),rot(pose['quaternion_wxyz'])))
    overlaps=[];nearest=None
    for i,(pw,rw) in enumerate(all_world):
        pd=np.array([np.linalg.norm(pw-x[1]) for x in actual]);candidates=np.argsort(pd)[:8]
        for j in candidates:
            rd=(rw*actual[j][2].inv()).magnitude()*180/np.pi
            item={'matched_index':i,'isaac225_index':int(j),'position_mm':float(pd[j]*1000),
                  'rotation_deg':float(rd),'joint_error_deg':np.rad2deg(q[i]-actual[j][0]).tolist()}
            score=item['position_mm']+10*item['rotation_deg']
            if nearest is None or score<nearest[0]:nearest=(score,item)
            if item['position_mm']<=1 and item['rotation_deg']<=.1:overlaps.append(item)
    j6deg=np.rad2deg(q[:,5]);isaac_j6=(-214.440606,-178.500246)
    branch={'all_j5_negative':bool(np.all(q[:,4]<0)),'j5_min_deg':float(np.rad2deg(q[:,4].min())),
            'j5_max_deg':float(np.rad2deg(q[:,4].max())),
            'j6_min_deg':float(j6deg.min()),'j6_max_deg':float(j6deg.max()),
            'isaac225_j6_range_deg':list(isaac_j6),
            'j6_in_isaac225_range_fraction':float(np.mean((j6deg>=isaac_j6[0])&(j6deg<=isaac_j6[1]))),
            'same_continuous_negative_j6_branch':bool(np.all(j6deg<0) and j6deg.max()>=isaac_j6[0]),
            'isaac225_branch_criterion':'J5 negative and continuous negative-J6 branch; envelope equality is reported separately'}
    passed=branch['all_j5_negative'] and branch['j6_max_deg']<0 and sum(x['failures'] for x in episodes)==0
    out={'status':'PASS_BRANCH_CONTINUITY' if passed else 'FAIL_BRANCH_OR_RESIDUAL',
      'source':str(EVIDENCE),'home_seed':str(HOME),'world_base_source':'kinematics_validation_2 wxyz fit',
      'method':'episode first pose seeded with sim_home; every later pose seeded with previous solution',
      'samples':len(q),'branch_check':branch,'episodes':episodes,
      'joint_min_deg':np.rad2deg(q.min(0)).tolist(),'joint_max_deg':np.rad2deg(q.max(0)).tolist(),
      'jacobian_sigma_min':{f'p{x}':float(np.percentile(sigma,x)) for x in (0,1,5,50)},
      'candidate_80pct_p0':float(.8*sigma.min()),
      'ik_sample_time_s':{'p50':float(np.percentile(times,50)),'p99':float(np.percentile(times,99)),
                          'max':float(np.max(times))},
      'residual_position_m_max':float(np.max(np.linalg.norm(residual[:,:3],axis=1))),
      'residual_rotation_deg_max':float(np.rad2deg(np.max(np.linalg.norm(residual[:,3:],axis=1)))),
      'overlap_definition':'position<=1mm and rotation<=0.1deg in world tool0_sim pose',
      'overlap_count':len(overlaps),'overlaps':overlaps,'nearest_pose_pair':nearest[1]}
    (ROOT/'reports/matched15_branch_reconstruction.json').write_text(json.dumps(out,indent=2)+'\n')

if __name__=='__main__':main()
