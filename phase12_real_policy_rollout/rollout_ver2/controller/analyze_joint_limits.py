#!/usr/bin/python3
"""Offline pendant-limit and envelope audit. No ROS or command interfaces."""
import json,sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:sys.path.insert(0,str(ROOT))
from controller.config import load_config
from controller.joint_limits import load_joint_limits
from controller.precheck import joint_margin_reasons,joint_envelope_status
from kinematics import PyKDLKinematics

BUNDLE=Path('/home/ubuntu/a0509_vla_linux_field_bundle_20260903')
SIM=BUNDLE/'lhj/phase11_sim_condition_replication/dataset/episodes/baseline'
REAL=BUNDLE/'lhj/phase12_real_policy_rollout/14_real_rollout_integration/real_trials'

def ordered(sample,key='position'):
    by=dict(zip(sample['names'],sample[key]));return np.array([by[f'joint_{i}'] for i in range(1,7)])

def first_events(samples,config):
    t0=float(samples[0]['receive']);default=None;old=None;envelope=None
    for i,s in enumerate(samples):
        q=ordered(s);reasons=joint_margin_reasons(q,config);env=joint_envelope_status(q,config)
        base={'sample_index':i,'receive':float(s['receive']),'elapsed_s':float(s['receive'])-t0,
              'joints_deg':np.rad2deg(q).tolist()}
        if default is None and reasons:default={**base,'reasons':reasons}
        if old is None and np.rad2deg(q[4]) <= -133:old={**base,'reason':'OLD_J5_LOWER_CANDIDATE_-133'}
        if envelope is None and env['outside']:envelope={**base,**env}
        if default and old and envelope:break
    return {'default_host_stop_first':default,'old_j5_minus133_first':old,
            'sim_envelope_first_exit_log_only':envelope}

def main():
    config=load_config();limits=load_joint_limits();sim=[]
    for p in sorted(SIM.glob('baseline_episode_*.json')):
        sim.extend([r['actual_joint_position_rad'][:6] for r in json.loads(p.read_text())['records']])
    sim=np.asarray(sim);sim[:,5]+=np.pi
    sigma=np.array([np.linalg.svd(PyKDLKinematics().jacobian(q),compute_uv=False)[-1] for q in sim])
    host_lo,host_hi=limits.host_lower_deg,limits.host_upper_deg
    sim_deg=np.rad2deg(sim);sim_lo,sim_hi=sim_deg.min(0),sim_deg.max(0)
    runs={}
    for p in sorted(REAL.glob('20261112_*_arm_only_servo_episode/jointstate.json')):
        samples=[s for s in json.loads(p.read_text())['samples'] if s.get('valid') and len(s.get('position',[]))>=6]
        if samples:runs[p.parent.name]=first_events(samples,config)
    replay=[]
    p=ROOT/'reports/controller_dryrun_world_left.jsonl'
    if p.exists():
        for line in p.open():
            row=json.loads(line)
            if 'joint_position_deg' not in row:continue
            q=np.deg2rad(row['joint_position_deg']);reasons=joint_margin_reasons(q,config)
            if reasons:
                replay.append({'segment':row['segment'],'time_s':row['time_s'],
                               'joints_deg':row['joint_position_deg'],'reasons':reasons});break
    branch=json.loads((ROOT/'reports/matched15_branch_reconstruction.json').read_text())
    matched_lo=np.asarray(branch['joint_min_deg']);matched_hi=np.asarray(branch['joint_max_deg'])
    matched_lo[5]+=180;matched_hi[5]+=180
    union_lo=np.minimum(sim_lo,matched_lo);union_hi=np.maximum(sim_hi,matched_hi)
    configured_lo=config.sim_joint_envelope_min_deg;configured_hi=config.sim_joint_envelope_max_deg
    if not (np.allclose(union_lo,configured_lo,atol=1e-5) and np.allclose(union_hi,configured_hi,atol=1e-5)):
        raise RuntimeError('CONFIGURED_SIM_ENVELOPE_DOES_NOT_MATCH_SOURCE_UNION')
    out={'status':'OFFLINE_NO_ROBOT_IO','pendant':{
          'profile':limits.profile,'source_date':limits.source_date,'evidence_path':limits.evidence_path,
          'evidence_exists':(ROOT/limits.evidence_path).is_file(),
          'lower_deg':limits.pendant_lower_deg.tolist(),'upper_deg':limits.pendant_upper_deg.tolist(),
          'host_margin_deg':limits.host_margin_deg.tolist(),
          'host_stop_lower_deg':host_lo.tolist(),'host_stop_upper_deg':host_hi.tolist(),
          'protective_expected_lower_deg':limits.protective_expected_lower_deg.tolist(),
          'protective_expected_upper_deg':limits.protective_expected_upper_deg.tolist()},
         'isaac225_real_basis':{'samples':len(sim),'all_inside_default_host_stop':bool(np.all(sim_deg>=host_lo)&np.all(sim_deg<=host_hi)),
          'range_min_deg':sim_lo.tolist(),'range_max_deg':sim_hi.tolist(),
          'clearance_to_lower_deg':(sim_lo-host_lo).tolist(),'clearance_to_upper_deg':(host_hi-sim_hi).tolist(),
          'minimum_clearance_deg':np.minimum(sim_lo-host_lo,host_hi-sim_hi).tolist(),
          'jacobian_sigma_min':{f'p{x}':float(np.percentile(sigma,x)) for x in (0,1,5,50)},
          'isaac_only_historical_candidate_superseded':0.0875,
          'latest_candidate_from_matched15_branch_and_isaac225':0.0803},
         'sim_joint_envelope_log_only':{
           'isaac225_real_basis':{'min':sim_lo.tolist(),'max':sim_hi.tolist()},
           'matched15_continuous_branch_real_basis':{'min':matched_lo.tolist(),'max':matched_hi.tolist()},
           'union':{'min':union_lo.tolist(),'max':union_hi.tolist()},'policy':'log_only'},
         'v1_recorded_jointstate':runs,'world_left_replay_first_default_stop':replay[0] if replay else None}
    (ROOT/'reports/joint_limits_analysis.json').write_text(json.dumps(out,indent=2)+'\n')

if __name__=='__main__':main()
