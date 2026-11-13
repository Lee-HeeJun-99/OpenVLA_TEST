#!/usr/bin/python3
"""Offline rollout uploader.  It has no ROS or control-runtime imports."""
from __future__ import annotations
import argparse,json,math,statistics,subprocess,time
from pathlib import Path
import numpy as np


def rows(path):
    if not path.is_file():return []
    result=[]
    for line in path.read_text().splitlines():
        try:result.append(json.loads(line))
        except (ValueError,TypeError):pass
    return result


def load_summary(folder):
    for name in ("summary.json","dryrun_live_summary.json"):
        path=folder/name
        if path.is_file():return json.loads(path.read_text())
    return {}


def numeric(values):return [float(v) for v in values if v is not None and np.isfinite(v)]
def mean(values):
    values=numeric(values);return None if not values else float(np.mean(values))
def p95(values):
    values=numeric(values);return None if not values else float(np.percentile(values,95))
def maximum(values):
    values=numeric(values);return None if not values else float(max(values))


def command_actual_error(step):
    command=step.get("command_delta") or {};actual=step.get("actual_delta") or {}
    cp=np.asarray(command.get("position_m",[]),dtype=float);ap=np.asarray(actual.get("position_m",[]),dtype=float)
    cr=np.asarray(command.get("rotation_vector_rad",[]),dtype=float);ar=np.asarray(actual.get("rotation_vector_rad",[]),dtype=float)
    pos=None if cp.shape!=(3,) or ap.shape!=(3,) else float(np.linalg.norm(cp-ap)*1000)
    rot=None if cr.shape!=(3,) or ar.shape!=(3,) else float(np.rad2deg(np.linalg.norm(cr-ar)))
    return pos,rot


def build_upload(folder,scene=None):
    summary=load_summary(folder);steps=rows(folder/"steps.jsonl");predictions=rows(folder/"predictions.jsonl")
    systems=rows(folder/"system_metrics.jsonl");pred_by_step={int(r.get("step",i)):r for i,r in enumerate(predictions)}
    logs=[];cumulative=np.zeros(3);first_close=None
    for index,step in enumerate(steps):
        i=int(step.get("step",index));pred=pred_by_step.get(i,{})
        pt=pred.get("processing_timing",{});ot=pred.get("observation_timing",{});st=pred.get("server_timing",{})
        poserr,roterr=command_actual_error(step);raw=step.get("action_raw") or [None]*7
        actual=step.get("actual_delta",{}).get("position_m")
        if actual is not None and len(actual)==3:cumulative+=np.asarray(actual,dtype=float)*1000
        gripper=step.get("gripper_binary",step.get("gripper_target"))
        if first_close is None and gripper is not None and float(gripper)>=.5:first_close=i
        endpoint_count=sum(bool(v) for v in step.get("endpoint_arm_min",[]))+sum(
          bool(v) for v in step.get("endpoint_arm_max",[]))
        row={"step":i,"infer/frame_wait_s":ot.get("frame_wait_s"),
          "infer/preprocess_s":pt.get("preprocess_s"),"infer/http_s":pt.get("http_s"),
          "ctrl/ik_precheck_s":step.get("ik_precheck_s"),"ctrl/move_s":step.get("move_s"),
          "ctrl/settle_s":step.get("settle_s"),"ctrl/step_total_s":step.get("step_total_s"),
          "ctrl/cmd_actual_pos_mm":poserr,"ctrl/cmd_actual_rot_deg":roterr,
          "ctrl/arrival_err_deg_max":maximum(np.abs(step.get("arrival_joint_error_deg",[]))),
          "ctrl/sigma_min":step.get("path_sigma_min"),"act/gripper":gripper,
          "act/endpoint_count":endpoint_count,"pose/x_mm":float(cumulative[0]),
          "pose/y_mm":float(cumulative[1]),"pose/z_mm":float(cumulative[2])}
        for key,value in st.items():row["infer/server_"+key]=value
        for n,name in enumerate(("dx","dy","dz")):row["act/"+name+"_mm"]=None if raw[n] is None else raw[n]*1000
        for n,name in enumerate(("rx","ry","rz"),3):row["act/"+name+"_deg"]=None if raw[n] is None else math.degrees(raw[n])
        for n,value in enumerate(step.get("target_joint_deg_j1_to_j6",[])):row[f"pose/j{n+1}_deg"]=value
        logs.append({k:v for k,v in row.items() if v is not None})
    syslogs=[]
    for row in systems:
        mapped={"t_s":row.get("t_s")}
        for key,value in row.items():
            if key in ("t_s","t_monotonic","t_wall","cpu_per_core") or isinstance(value,dict):continue
            if isinstance(value,(int,float)) or value is None:mapped["sys/"+key]=value
        for proc in ("proc_ctrl","proc_obs","proc_server"):
            for key,value in (row.get(proc) or {}).items():
                if key!="pid":mapped[f"sys/{proc}_{key}"]=value
        syslogs.append({k:v for k,v in mapped.items() if v is not None})
    forward=[r.get("server_timing",{}).get("forward_s") for r in predictions]
    http=[r.get("processing_timing",{}).get("http_s") for r in predictions]
    poserrs=[command_actual_error(s)[0] for s in steps];roterrs=[command_actual_error(s)[1] for s in steps]
    upload_summary={"end_reason":summary.get("end_reason"),"executed_steps":summary.get("executed_steps",len(steps)),
      "elapsed_s":summary.get("elapsed_s",summary.get("duration_s")),
      "cumulative_displacement":summary.get("cumulative_displacement"),
      "minimum_height_relative_home_mm":summary.get("minimum_height_relative_home_mm"),
      "first_close_step":first_close,"forward_s_mean":mean(forward),"forward_s_p95":p95(forward),
      "http_s_mean":mean(http),"http_s_p95":p95(http),
      "step_total_s_mean":mean([s.get("step_total_s") for s in steps]),
      "gpu_util_max":maximum([s.get("gpu_util") for s in systems]),
      "vram_used_gb_max":maximum([s.get("vram_used_gb") for s in systems]),
      "cpu_core_max":maximum([s.get("cpu_core_max") for s in systems]),
      "ram_used_gb_max":maximum([s.get("ram_used_gb") for s in systems]),
      "cmd_actual_pos_mm_max":maximum(poserrs),"cmd_actual_rot_deg_max":maximum(roterrs),
      "arrival_err_deg_max":maximum([maximum(np.abs(s.get("arrival_joint_error_deg",[]))) for s in steps]),
      "joint_states_age_max_ms":maximum([s.get("joint_states_age_max_ms") for s in systems])}
    settings={k:v for k,v in summary.items() if k not in ("steps","inference_results",
      "observation_records","feedback_stale_events") and not isinstance(v,list)}
    first_prediction=predictions[0] if predictions else {}
    config={"instruction":summary.get("instruction"),"scene":scene or summary.get("operator_note") or
      summary.get("scene_cube_color"),"model":summary.get("model") or first_prediction.get("server_model"),
      "checkpoint":first_prediction.get("server_checkpoint"),"control_mode":summary.get("control_mode"),
      "segment_T_s":summary.get("segment_T_s"),"servoj_time_s":summary.get("servoj_time_s"),"settings":settings}
    try:config["git_commit"]=subprocess.check_output(["git","rev-parse","HEAD"],cwd=folder,
      text=True,stderr=subprocess.DEVNULL).strip()
    except Exception:config["git_commit"]=None
    return {"config":config,"step_logs":logs,"system_logs":syslogs,"summary":upload_summary,
      "files_found":{"steps":bool(steps),"predictions":bool(predictions),"system_metrics":bool(systems)}}


def folders_from_args(args):
    found=[p.resolve() for p in args.folders]
    if args.all:
        candidates={p.parent for name in ("summary.json","dryrun_live_summary.json")
          for p in args.all.rglob(name)}
        found.extend(p for p in sorted(candidates) if not (p/"wandb_uploaded.json").exists())
    return list(dict.fromkeys(found))


def main():
    p=argparse.ArgumentParser();p.add_argument("folders",nargs="*",type=Path);p.add_argument("--all",type=Path)
    p.add_argument("--project",default="a0509-vla-rollout");p.add_argument("--scene");p.add_argument("--dry-run",action="store_true")
    a=p.parse_args();folders=folders_from_args(a)
    if not folders:raise ValueError("no rollout folders selected")
    for folder in folders:
        payload=build_upload(folder,a.scene)
        if a.dry_run:print(json.dumps({"folder":str(folder),**payload},indent=2,default=str));continue
        import wandb
        run=wandb.init(project=a.project,name=folder.name,id=folder.name,resume="allow",config=payload["config"])
        wandb.define_metric("step");wandb.define_metric("infer/*",step_metric="step")
        wandb.define_metric("ctrl/*",step_metric="step");wandb.define_metric("act/*",step_metric="step")
        wandb.define_metric("pose/*",step_metric="step");wandb.define_metric("t_s")
        wandb.define_metric("sys/*",step_metric="t_s")
        for row in payload["step_logs"]:wandb.log(row)
        for row in payload["system_logs"]:wandb.log(row)
        for key,value in payload["summary"].items():
            if value is not None:run.summary[key]=value
        artifact=wandb.Artifact(folder.name,type="rollout");artifact.add_dir(str(folder));run.log_artifact(artifact)
        url=run.url;run.finish()
        (folder/"wandb_uploaded.json").write_text(json.dumps({"run_url":url,
          "uploaded_at_wall":time.time(),"project":a.project},indent=2)+"\n")

if __name__=="__main__":main()
