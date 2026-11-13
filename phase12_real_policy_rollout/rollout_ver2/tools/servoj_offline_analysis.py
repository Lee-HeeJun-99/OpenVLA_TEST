#!/usr/bin/env /usr/bin/python3
"""Offline-only fixed-duration ServoJ replay analysis; never imports ROS."""
from __future__ import annotations
import argparse,json,time,sys
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import matplotlib.pyplot as plt

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))

from controller.config import load_config
from controller.geometry import ToolTransform
from kinematics import PyKDLKinematics
from runtime.servoj import FixedSegmentPlanner,ServoJPrecheckError

DEFAULT_RUN=ROOT/"reports/live/20261113_091859/policy_movej/20261113_140038"

def pose_error(a,b):
    return 1e3*float(np.linalg.norm(a.position_m-b.position_m)),float(np.degrees(
        Rotation.from_matrix(a.rotation@b.rotation.T).magnitude()))

def classify(reasons):
    labels=[]
    for needle,label in (("IK_FAILED","ik_failure"),("JOINT_LIMIT_MARGIN","joint_margin"),
                         ("JOINT_VELOCITY","joint_velocity"),("JOINT_ACCELERATION","joint_acceleration"),
                         ("SINGULARITY","singularity"),("WORKSPACE","workspace")):
        if any(needle in x for x in reasons):labels.append(label)
    return labels or ["other"]

def main():
    ap=argparse.ArgumentParser();ap.add_argument("--run",type=Path,default=DEFAULT_RUN)
    ap.add_argument("--output",type=Path,default=ROOT/"reports/servoj_offline")
    args=ap.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    summary=json.loads((args.run/"summary.json").read_text());steps=summary["steps"]
    kin=PyKDLKinematics();tool=ToolTransform.tool0_real();cfg=load_config()
    durations=(.4,.6,1.0);results={};examples={}
    actual_poses=[]
    for st in steps:
        q_target=np.deg2rad(st["target_joint_deg_j1_to_j6"])
        q_actual=q_target+np.deg2rad(st["arrival_joint_error_deg"])
        actual_poses.append(tool.link_to_tool(kin.fk(q_actual)))
    previous_q=cfg.real_home_joint_rad.copy()
    starts=[]
    for st in steps:
        starts.append(previous_q.copy());previous_q=np.deg2rad(st["target_joint_deg_j1_to_j6"])
    for T in durations:
        planner=FixedSegmentPlanner(kin,tool,cfg,rate_hz=100,sigma_min=cfg.jacobian_min_singular_value)
        failures=[];pre=[];ik_samples=[];max_v=np.zeros(6);max_a=np.zeros(6);endpoint_action=[];endpoint_recorded=[];endpoint_actual=[]
        for i,(st,q0) in enumerate(zip(steps,starts)):
            start=tool.link_to_tool(kin.fk(q0));dp=st["interpreted_delta_position_m"];rv=st["interpreted_rotation_vector_rad"]
            try:
                seg=planner.plan_action(start,dp,rv,q0,T);pre.append(seg.precompute_s);ik_samples.extend(seg.ik_sample_s)
                max_v=np.maximum(max_v,np.max(np.abs(seg.velocity_rad_s),axis=0));max_a=np.maximum(max_a,np.max(np.abs(seg.acceleration_rad_s2),axis=0))
                recorded=tool.link_to_tool(kin.fk(np.deg2rad(st["target_joint_deg_j1_to_j6"])))
                endpoint_action.append(pose_error(seg.tool_poses[-1],planner.target(start,dp,rv)))
                endpoint_recorded.append(pose_error(seg.tool_poses[-1],recorded));endpoint_actual.append(pose_error(seg.tool_poses[-1],actual_poses[i]))
                if T not in examples and i==0:examples[T]=seg
            except ServoJPrecheckError as e:
                failures.append({"step":i,"reasons":list(e.reasons),"classes":classify(e.reasons),"diagnostics":e.diagnostics})
                d=e.diagnostics
                if "max_velocity_rad_s" in d:max_v=np.maximum(max_v,d["max_velocity_rad_s"])
                if "max_acceleration_rad_s2" in d:max_a=np.maximum(max_a,d["max_acceleration_rad_s2"])
        def pct(x):return {"p50":float(np.percentile(x,50)),"p99":float(np.percentile(x,99)),"max":float(np.max(x))} if x else None
        results[str(T)]={"segments":len(steps),"accepted":len(steps)-len(failures),"failures":failures,
          "failure_class_counts":{k:sum(k in f["classes"] for f in failures) for k in ["ik_failure","joint_margin","joint_velocity","joint_acceleration","singularity","workspace","other"]},
          "max_velocity_rad_s":max_v.tolist(),"velocity_limit_percent":(100*max_v/cfg.max_joint_velocity_rad_s).tolist(),
          "max_acceleration_rad_s2":max_a.tolist(),"acceleration_limit_percent":(100*max_a/cfg.max_joint_acceleration_rad_s2).tolist(),
          "segment_precompute_s":pct(pre),"ik_sample_s":pct(ik_samples),
          "endpoint_vs_action_target":{"position_mm":pct([x[0] for x in endpoint_action]),"rotation_deg":pct([x[1] for x in endpoint_action])},
          "endpoint_vs_movej_command_fk":{"position_mm":pct([x[0] for x in endpoint_recorded]),"rotation_deg":pct([x[1] for x in endpoint_recorded])},
          "endpoint_vs_movej_actual":{"position_mm":pct([x[0] for x in endpoint_actual]),"rotation_deg":pct([x[1] for x in endpoint_actual])}}
    payload={"source":str(args.run),"executed_actions":len(steps),"rate_hz":100,"sigma_min":cfg.jacobian_min_singular_value,
             "limits":{"velocity_rad_s":cfg.max_joint_velocity_rad_s.tolist(),"acceleration_rad_s2":cfg.max_joint_acceleration_rad_s2.tolist()},"durations":results}
    (args.output/"analysis.json").write_text(json.dumps(payload,indent=2)+"\n")
    fig,axs=plt.subplots(1,len(examples),figsize=(15,4),sharey=True)
    for ax,(T,seg) in zip(axs,examples.items()):
        for j in range(6):ax.plot(seg.times_s[1:],np.degrees(seg.velocity_rad_s[:,j]),label=f"J{j+1}")
        ax.set(title=f"step 0, T={T}s",xlabel="time (s)");ax.grid();
    axs[0].set_ylabel("joint velocity (deg/s)");axs[-1].legend(ncol=2);fig.tight_layout();fig.savefig(args.output/"joint_velocity_example.png",dpi=160);plt.close(fig)
    fig,axs=plt.subplots(1,2,figsize=(10,4))
    for ax,key,title in ((axs[0],"velocity_limit_percent","velocity"),(axs[1],"acceleration_limit_percent","acceleration")):
        x=np.arange(6);width=.25
        for n,T in enumerate(durations):ax.bar(x+(n-1)*width,results[str(T)][key],width,label=f"T={T}s")
        ax.axhline(100,color="r",ls="--");ax.set_xticks(x,[f"J{i}" for i in range(1,7)]);ax.set_ylabel("limit (%)");ax.set_title(title);ax.legend();ax.grid(axis="y")
    fig.tight_layout();fig.savefig(args.output/"duration_limit_margin.png",dpi=160);plt.close(fig)
    lines=["# ServoJ offline replay","",f"- Source: `{args.run}`",f"- Actions: {len(steps)}; 100 Hz; sigma-min candidate {cfg.jacobian_min_singular_value}","- Offline only; no ROS import or command publication.","", "| T (s) | accepted / 222 | IK | margin | sigma | velocity | acceleration | precompute p50/p99/max (s) |", "|---:|---:|---:|---:|---:|---:|---:|---|"]
    for T in durations:
        r=results[str(T)];c=r["failure_class_counts"];p=r["segment_precompute_s"]
        lines.append(f"| {T} | {r['accepted']} / {len(steps)} | {c['ik_failure']} | {c['joint_margin']} | {c['singularity']} | {c['joint_velocity']} | {c['joint_acceleration']} | {p} |")
    lines += ["","## Maximum dynamics", "", "Percentages use the configured candidate limits; a value over 100% is a whole-segment rejection, never clamped.",""]
    for T in durations:
        r=results[str(T)];lines += [f"- T={T}: velocity % `{np.round(r['velocity_limit_percent'],2).tolist()}`, acceleration % `{np.round(r['acceleration_limit_percent'],2).tolist()}`"]
    lines += ["","## Endpoint comparison","","The action-target comparison is the required numerical identity. The recorded-command comparison uses FK of the MoveJ target joints and therefore exposes its earlier IK tolerance. Actual arrival is reconstructed per step as recorded target joint plus `arrival_joint_error_deg` (not by accumulating deltas).",""]
    for T in durations:
        r=results[str(T)];lines += [f"- T={T}: action target `{r['endpoint_vs_action_target']}`; recorded MoveJ command FK `{r['endpoint_vs_movej_command_fk']}`; MoveJ actual `{r['endpoint_vs_movej_actual']}`"]
    lines += ["","## Failure locations","", "Full diagnostics are in `analysis.json`.",""]
    for T in durations:lines += [f"- T={T}: `{[f['step'] for f in results[str(T)]['failures']]}`"]
    lines += ["", "![joint velocity](joint_velocity_example.png)","", "![limit margin](duration_limit_margin.png)",""]
    (args.output/"servoj_offline.md").write_text("\n".join(lines))
    print(json.dumps({T:{"accepted":results[str(T)]["accepted"],"failed":len(results[str(T)]["failures"])} for T in durations},indent=2))
if __name__=="__main__":main()
