#!/usr/bin/python3
"""Offline matched15 joint-step and cube/home-frame reference export."""
import json,sys
from pathlib import Path
import numpy as np
from PIL import Image
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from controller.geometry import ToolTransform
from kinematics import DLSIKSolver,Pose,PyKDLKinematics

BUNDLE=ROOT.parents[2]
EVIDENCE=BUNDLE/'validation/evidence/vanilla/success_profile_s1_balanced_step8130_matched15_k1_seed9406_15ep.json'
HOME=BUNDLE/'isaac_sim/config/a0509_home_pose.json'
OUT=ROOT/'reports/sim_reference'
WORLD_BASE_P=np.array([-0.000389495411757726,-0.0002573428604871621,0.8235262353883922])
WORLD_BASE_R=Rotation.from_euler('xyz',[0.004445488457065276,0.016366501042746482,-0.0038822342347884717],degrees=True)

def rotation_wxyz(q):
    q=np.asarray(q,float);return Rotation.from_quat(q[[1,2,3,0]])

def main():
    OUT.mkdir(parents=True,exist_ok=True)
    evidence=json.loads(EVIDENCE.read_text());home=np.array(json.loads(HOME.read_text())['joint_positions_rad'])
    kin=PyKDLKinematics();tool=ToolTransform.tool0_sim();changes=[];initialization_changes=[]
    episodes=[];cube_rows=[]
    for episode in evidence['results']:
        q=home.copy();episode_changes=[]
        first=episode['actions'][0];home_world_p=np.array(first['current_position_m'])
        home_world_r=rotation_wxyz(first['current_quaternion_wxyz'])
        for color,position in episode['initial_cube_positions_m'].items():
            relative_base=(np.array(position)-home_world_p)*1000
            relative_tool=home_world_r.inv().apply(np.array(position)-home_world_p)*1000
            cube_rows.append({'episode':episode['episode_number'],'target_color':episode['target_color'],
              'cube_color':color,'relative_to_home_tool0_origin_base_axes_xyz_mm':relative_base.tolist(),
              'expressed_in_home_tool0_axes_xyz_mm':relative_tool.tolist()})
        for action_index,action in enumerate(episode['actions']):
            rw=rotation_wxyz(action['current_quaternion_wxyz']);pw=np.array(action['current_position_m'])
            base_tool=Pose(WORLD_BASE_R.inv().apply(pw-WORLD_BASE_P),(WORLD_BASE_R.inv()*rw).as_matrix())
            target=tool.tool_to_link(base_tool)
            fit=least_squares(lambda x:DLSIKSolver._error(kin.fk(x),target),q,
                bounds=(kin.urdf_lower_rad,kin.urdf_upper_rad),max_nfev=1500,
                xtol=1e-11,ftol=1e-11,gtol=1e-11)
            delta=np.abs(np.rad2deg(fit.x-q))
            if action_index==0:initialization_changes.append(delta)
            else:changes.append(delta);episode_changes.append(delta)
            q=fit.x
        local=np.asarray(episode_changes)
        episodes.append({'episode':episode['episode_number'],'target_color':episode['target_color'],
          'trajectory_transitions':len(local),'max_abs_joint_step_deg':float(local.max()),
          'max_joint':int(np.unravel_index(np.argmax(local),local.shape)[1]+1)})
    changes=np.asarray(changes);initialization_changes=np.asarray(initialization_changes)
    per_joint=[]
    for index in range(6):
        values=changes[:,index];per_joint.append({'joint':f'J{index+1}',
          'p99_abs_step_deg':float(np.percentile(values,99)),
          'max_abs_step_deg':float(values.max())})
    flattened=changes.ravel();overall={'trajectory_transitions':int(len(changes)),
      'joint_components':int(flattened.size),'p99_abs_step_deg':float(np.percentile(flattened,99)),
      'max_abs_step_deg':float(flattened.max()),'max_joint_delta_candidate_deg':10.0,
      'candidate_10deg_below_observed_max':bool(10.0<float(flattened.max())),
      'candidate_10deg_above_or_equal_observed_max':bool(10.0>=float(flattened.max())),
      'excluded_home_seed_to_first_observation':{
        'reason':'initial pose reconstruction alignment, not an action transition',
        'p99_abs_deg':float(np.percentile(initialization_changes,99)),
        'max_abs_deg':float(initialization_changes.max())}}

    # Exact matched15 frames are absent: export clearly named related-sim proxies only.
    proxy_sources=[
      BUNDLE/'lhj/phase9_sim_policy_shadow_replay/01_sim_policy_rollout_episode4_sim_baseline/episode4_sim_baseline_redprompt_k5_seed9406_1ep_frames/episode_001_red_initial.jpg',
      BUNDLE/'lhj/phase9_sim_policy_shadow_replay/01_sim_policy_rollout/phase9_fixed_layout_sim_policy_k1_seed20260922_1ep_frames/episode_001_red_initial.jpg',
      BUNDLE/'lhj/phase9_sim_policy_shadow_replay/01_sim_policy_rollout_episode4/episode4_fixed_layout_oft_k5_k5_seed20260923_1ep_frames/episode_001_red_initial.jpg']
    image_manifest=[]
    for index,source in enumerate(proxy_sources,1):
        if not source.is_file():continue
        destination=OUT/f'sim_proxy_initial_{index:02d}_224.jpg'
        Image.open(source).convert('RGB').resize((224,224),Image.Resampling.BICUBIC).save(destination,quality=92)
        image_manifest.append({'file':str(destination.relative_to(ROOT)),'source':str(source),
          'exact_matched15':False,'processing':'resize-naive 224x224 bicubic, JPEG q92'})
    exact_paths=[row['initial_frame'] for row in evidence['results'][:3]]
    result={'source':str(EVIDENCE),'method':'sim_home seed, then previous solution seed; absolute joint deltas',
      'includes_home_to_first_pose':False,'overall':overall,'per_joint':per_joint,'episodes':episodes,
      'cube_positions':cube_rows,'cube_frame':'origin is each episode first current tool0_sim. Primary '
        'relative field uses sim world/base-aligned axes; tool-axis expression is also included. '
        'tool0_sim is equivalent to tool0_real after confirmed J6+180 mount compensation',
      'exact_matched15_images_available':False,'exact_image_paths_from_evidence':exact_paths,
      'image_note':'Evidence paths are Windows D:\\codex_staging paths absent from this bundle. '
        'Saved images are related-sim proxies and must not be represented as exact matched15 frames.',
      'proxy_images':image_manifest}
    (OUT/'matched15_reference.json').write_text(json.dumps(result,indent=2)+'\n')
    lines=['# matched15 sim reference','',f'- source: `{EVIDENCE}`',
      '- IK: sim_home seed 후 직전 해 seed의 연속 branch. 통계는 연속 pose 전이만 사용하며 home seed→첫 관측 정렬은 제외.',
      f'- 전체 관절 성분 |Δq| p99/max: `{overall["p99_abs_step_deg"]:.4f}° / {overall["max_abs_step_deg"]:.4f}°`.',
      f'- 10° 후보가 관측 최대보다 작은가: `{overall["candidate_10deg_below_observed_max"]}`.',
      f'- 제외한 home seed→첫 pose 정렬 max: `{overall["excluded_home_seed_to_first_observation"]["max_abs_deg"]:.4f}°`.','',
      '| joint | p99 | max |','|---|---:|---:|']
    lines += [f'| {r["joint"]} | {r["p99_abs_step_deg"]:.4f}° | {r["max_abs_step_deg"]:.4f}° |' for r in per_joint]
    lines += ['','## Cube positions relative to episode home tool0 origin','','단위 mm. 표는 sim world≈robot base 축이며 원점만 첫 tool0 pose로 옮겼다. tool0 자체 축 표현도 JSON에 저장했다.','','| episode | target | cube | base-axis relative xyz mm |','|---:|---|---|---|']
    for row in cube_rows:
        xyz=', '.join(f'{x:.2f}' for x in row['relative_to_home_tool0_origin_base_axes_xyz_mm'])
        lines.append(f'| {row["episode"]} | {row["target_color"]} | {row["cube_color"]} | [{xyz}] |')
    lines += ['','## Images','',result['image_note'],'']
    lines += [f'- `{x["file"]}` ← `{x["source"]}` (proxy)' for x in image_manifest]
    (OUT/'matched15_reference.md').write_text('\n'.join(lines)+'\n')
    print(json.dumps(overall,indent=2))

if __name__=='__main__':main()
