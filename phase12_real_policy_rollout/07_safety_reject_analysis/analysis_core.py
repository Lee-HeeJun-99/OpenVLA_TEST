#!/usr/bin/env python3
"""Offline-only Episode 4 safety-reject analysis.

This module reads recorded predictions and never imports ROS or creates command
capabilities.  Production configuration files are inputs, never outputs.
"""
from __future__ import annotations

import csv
import itertools
import json
import math
import sys
from collections import Counter, defaultdict
from pathlib import Path

import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
P12 = HERE.parent
LHJ = P12.parent
SAFETY = P12 / "02_safety"
SHADOW = P12 / "03_shadow_mode"
sys.path[:0] = [str(SAFETY),str(SHADOW)]

from action_rate_limiter import CanonicalActionRateLimiter  # noqa: E402
from open_loop_gripper_supervisor import OpenLoopGripperSupervisor  # noqa: E402
from runtime_safety_supervisor import RuntimeSafetySupervisor  # noqa: E402
from oft_timing_contract import (inference_time_sec,legacy_recorded_time_sec,
                                 target_time_sec)  # noqa: E402

RESULTS = HERE / "results"
PLOTS = HERE / "plots"
OPEN_OUT = P12 / "03_shadow_mode/recorded_shadow_episode4_openvla_20261006.jsonl"
OFT_OUT = P12 / "03_shadow_mode/recorded_shadow_episode4_oft_20261006.jsonl"
P10 = LHJ / "phase10_planner_based_shadow_mode/06_shadow_collection/offline_recorded_episode4"
OPEN_SRC = P10 / "openvla_step8130_retry/samples.jsonl"
OFT_SRC = P10 / "oft_vision_step28560_local/samples.jsonl"
CONFIG = P12 / "01_configs/safety_limits.yaml"

WORKSPACE_MIN = np.array([0.275, -0.355, 0.267], dtype=float)
WORKSPACE_MAX = np.array([0.554, 0.386, 0.754], dtype=float)
DT = 0.2
GRASP_PHASE = "grasp_close"

CANONICAL = {
    "invalid_action": "nonfinite_action",
    "clock_domain_or_time_regression": "timestamp_invalid",
    "inference_timeout": "model_timeout",
    "gripper_stale_or_timeout": "gripper_state_invalidated",
    "gripper_unknown_state_blocks_command": "gripper_state_invalidated",
}


def read_jsonl(path: Path):
    with path.open() as f:
        return [json.loads(line) for line in f if line.strip()]


def write_csv(path: Path, rows, fields=None):
    path.parent.mkdir(parents=True, exist_ok=True)
    rows = list(rows)
    if fields is None:
        fields = list(rows[0]) if rows else []
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def vector(row):
    a = row["canonical_action"]
    return np.array(a["translation_m"] + a["rotation_rotvec_rad"] + [a["gripper_closedness"]], dtype=float)


def source_position_map(path: Path):
    return {int(r["frame_id"]): np.array(r["raw_ee_position"], dtype=float)
            for r in read_jsonl(path) if r.get("raw_ee_position") is not None}


def production_config():
    with CONFIG.open() as f:
        return yaml.safe_load(f)


def now_and_source(model, row):
    frame = int(row["frame_id"])
    idx = int(row["canonical_action"].get("chunk_index", 0))
    if model == "openvla":
        return frame * DT, frame * DT
    # This analysis reproduces the immutable pre-fix artifact intentionally.
    # New runtime decisions use target_time_sec() in prediction_shadow_runtime.
    return legacy_recorded_time_sec(frame,idx), legacy_recorded_time_sec(frame,0)


def replay(model, rows, positions, *, t_step=.004, r_step_deg=4.0,
           tv_mm_s=20.0, ta_mm_s2=20.0, rv_deg_s=20.0, ra_deg_s2=20.0):
    """Faithfully replay current command-disabled safety semantics.

    Returns all limited actions and blockers.  This is analysis-only and has no
    command sink, publisher, client or ROS dependency.
    """
    limiter = CanonicalActionRateLimiter(
        control_rate_hz=5.0,
        max_translation_velocity_m_s=tv_mm_s / 1000.0,
        max_rotation_velocity_rad_s=math.radians(rv_deg_s),
        max_translation_acceleration_m_s2=ta_mm_s2 / 1000.0,
        max_rotation_acceleration_rad_s2=math.radians(ra_deg_s2))
    safety = RuntimeSafetySupervisor(
        max_action_age_sec=1.2, min_command_period_sec=.19,
        max_translation_m=t_step, max_rotation_rad=math.radians(r_step_deg),
        max_translation_velocity_m_s=tv_mm_s / 1000.0,
        max_rotation_velocity_rad_s=math.radians(rv_deg_s),
        max_translation_acceleration_m_s2=ta_mm_s2 / 1000.0,
        max_rotation_acceleration_rad_s2=math.radians(ra_deg_s2))
    grip = OpenLoopGripperSupervisor(.3, .7)
    grip.confirm_initial_open(True)
    output = []
    for row in rows:
        raw = vector(row)
        limited = np.array(limiter.limit(raw).limited_action)
        now, source = now_and_source(model, row)
        sid = row["canonical_action"]["sequence_id"]
        sr = safety.inspect(action_id=sid, action=limited, source_monotonic=source,
                            now_monotonic=now)
        pos = positions[int(row["frame_id"])]
        target = pos + limited[:3]
        workspace_bad = bool(np.any(target < WORKSPACE_MIN) or np.any(target > WORKSPACE_MAX))
        gd = grip.resolve(raw[6], phase=row["phase"], fresh=sr.accepted)
        premature = gd.reason == "close_forbidden_outside_grasp_close"
        blockers = []
        if np.linalg.norm(raw[:3]) > t_step + 1e-12:
            blockers.append("raw_translation_step_limit")
        if np.linalg.norm(raw[3:6]) > math.radians(r_step_deg) + 1e-12:
            blockers.append("raw_rotation_step_limit")
        if not sr.accepted:
            blockers.append(sr.hold_reason)
        if workspace_bad:
            blockers.append("workspace_violation")
        if premature:
            blockers.append("premature_gripper_close")
        if not gd.accepted and not premature:
            blockers.append("gripper_" + gd.reason)
        output.append({"blockers": blockers, "limited": limited, "target": target,
                       "gripper_reason": gd.reason})
    return output


def diagnostic_metrics(rows):
    raw = np.stack([vector(r) for r in rows])
    tv = np.linalg.norm(raw[:, :3], axis=1) / DT
    rv = np.degrees(np.linalg.norm(raw[:, 3:6], axis=1)) / DT
    ta = np.full(len(rows), np.nan)
    ra = np.full(len(rows), np.nan)
    if len(rows) > 1:
        tvel = raw[:, :3] / DT
        rvel = raw[:, 3:6] / DT
        ta[1:] = np.linalg.norm(np.diff(tvel, axis=0), axis=1) / DT
        ra[1:] = np.degrees(np.linalg.norm(np.diff(rvel, axis=0), axis=1)) / DT
    return raw, tv, rv, ta, ra


def enrich_model(model, rows, source_path):
    positions = source_position_map(source_path)
    rep = replay(model, rows, positions)
    raw, tv, rv, ta, ra = diagnostic_metrics(rows)
    out = []
    for i, (row, sim) in enumerate(zip(rows, rep)):
        actual = list(row["safety_decision"]["reason"])
        # Independent diagnostics expose conditions hidden by supervisor early-return.
        diag = []
        if np.linalg.norm(raw[i, :3]) > .004 + 1e-12: diag.append("raw_translation_step_limit")
        if np.linalg.norm(raw[i, 3:6]) > math.radians(4) + 1e-12: diag.append("raw_rotation_step_limit")
        if tv[i] > .020 + 1e-12: diag.append("raw_translation_velocity_limit")
        if rv[i] > 20 + 1e-9: diag.append("raw_rotation_velocity_limit")
        if i and ta[i] > .020 + 1e-12: diag.append("raw_translation_acceleration_limit")
        if i and ra[i] > 20 + 1e-9: diag.append("raw_rotation_acceleration_limit")
        pos = positions[int(row["frame_id"])]
        raw_target = pos + raw[i, :3]
        if np.any(raw_target < WORKSPACE_MIN) or np.any(raw_target > WORKSPACE_MAX):
            diag.append("raw_workspace_violation")
        if raw[i, 6] >= .7 and row["phase"] != GRASP_PHASE:
            diag.append("premature_gripper_close")
        target = sim["target"]
        violated = []
        outside = []
        for ax, value, lo, hi in zip("xyz", target, WORKSPACE_MIN, WORKSPACE_MAX):
            if value < lo: violated.append(ax); outside.append(lo-value)
            elif value > hi: violated.append(ax); outside.append(value-hi)
        margin = float(np.min(np.minimum(pos-WORKSPACE_MIN, WORKSPACE_MAX-pos)))
        ca = row["canonical_action"]
        out.append({
            "model": model, "frame_id": int(row["frame_id"]),
            "sequence_id": ca["sequence_id"],
            "chunk_id": f"episode4-frame{row['frame_id']}" if model == "oft" else "",
            "chunk_index": int(ca.get("chunk_index", 0)), "chunk_size": int(ca.get("chunk_size", 1)),
            "phase": row["phase"], "raw_action": json.dumps(raw[i].tolist()),
            "limited_action": json.dumps(sim["limited"].tolist()),
            "accepted": bool(row["safety_decision"]["accepted"]),
            "blockers": json.dumps(actual), "diagnostic_blockers": json.dumps(sorted(set(diag))),
            "primary_blocker": actual[0] if actual else "",
            "translation_dx_m": raw[i,0], "translation_dy_m": raw[i,1], "translation_dz_m": raw[i,2],
            "translation_norm_m": float(np.linalg.norm(raw[i,:3])),
            "rotation_norm_deg": float(np.degrees(np.linalg.norm(raw[i,3:6]))),
            "translation_velocity_mm_s": float(tv[i]*1000),
            "rotation_velocity_deg_s": float(rv[i]),
            "translation_acceleration_mm_s2": "" if np.isnan(ta[i]) else float(ta[i]*1000),
            "rotation_acceleration_deg_s2": "" if np.isnan(ra[i]) else float(ra[i]),
            "current_position_m": json.dumps(pos.tolist()), "target_position_m": json.dumps(target.tolist()),
            "raw_target_position_m": json.dumps(raw_target.tolist()),
            "workspace_near_boundary_4mm": margin <= .004,
            "workspace_boundary_margin_m": margin,
            "workspace_violated_axis": ",".join(violated),
            "workspace_distance_outside_m": max(outside) if outside else 0.0,
            "gripper_closedness": float(raw[i,6]),
            "gripper_binary": "CLOSED" if raw[i,6] >= .7 else ("OPEN" if raw[i,6] <= .3 else "HOLD"),
            "action_age_sec": now_and_source(model,row)[0]-now_and_source(model,row)[1],
            "command_issued": False,
        })
    # The replay must reproduce both emitted blockers and decisions exactly.
    mismatches = [i for i,(r,s) in enumerate(zip(rows,rep))
                  if list(r["safety_decision"]["reason"]) != s["blockers"]]
    return out, mismatches


def describe(values):
    a = np.asarray(values, dtype=float)
    return {"mean":float(np.mean(a)), "std":float(np.std(a)), "median":float(np.median(a)),
            "p90":float(np.percentile(a,90)), "p95":float(np.percentile(a,95)),
            "p99":float(np.percentile(a,99)), "max":float(np.max(a))}


def blocker_tables(all_rows):
    summary=[]; overlap=[]; primary=[]
    for model in ("openvla","oft"):
        rows=[r for r in all_rows if r["model"]==model]
        rejected=[r for r in rows if not r["accepted"]]
        counts=Counter(b for r in rejected for b in json.loads(r["blockers"]))
        pcounts=Counter(r["primary_blocker"] for r in rejected)
        for b,c in sorted(counts.items(),key=lambda x:(-x[1],x[0])):
            summary.append({"model":model,"blocker":b,"canonical_category":CANONICAL.get(b,b),
                            "action_count":c,"percent_total":100*c/len(rows),
                            "percent_rejected":100*c/len(rejected)})
        for b,c in sorted(pcounts.items(),key=lambda x:(-x[1],x[0])):
            primary.append({"model":model,"primary_blocker":b,"count":c,
                            "percent_rejected":100*c/len(rejected)})
        names=sorted(counts)
        for a in names:
            for b in names:
                c=sum(a in json.loads(r["blockers"]) and b in json.loads(r["blockers"]) for r in rejected)
                overlap.append({"model":model,"blocker_a":a,"blocker_b":b,"cooccurrence_count":c})
    return summary,primary,overlap


def top_combinations(all_rows):
    answer={}
    for model in ("openvla","oft"):
        c=Counter(" + ".join(sorted(json.loads(r["blockers"]))) for r in all_rows
                  if r["model"]==model and not r["accepted"])
        answer[model]=[{"combination":k,"count":v} for k,v in c.most_common(10)]
    return answer


def sensitivity(model, rows, source_path):
    pos=source_position_map(source_path)
    base=replay(model,rows,pos)
    result=[]
    specs={
      "translation_step_m":[.002,.003,.004,.005,.006,.008,.010],
      "rotation_step_deg":[2,3,4,5,6,8,10],
      "translation_velocity_mm_s":[10,20,30,40,50],
      "translation_acceleration_mm_s2":[20,40,60,80,100],
    }
    baseline={"translation_step_m":.004,"rotation_step_deg":4.0,
              "translation_velocity_mm_s":20.0,"translation_acceleration_mm_s2":20.0}
    base_n=sum(not x["blockers"] for x in base)
    for parameter,values in specs.items():
        for value in values:
            kw={"t_step":baseline["translation_step_m"],"r_step_deg":baseline["rotation_step_deg"],
                "tv_mm_s":baseline["translation_velocity_mm_s"],"ta_mm_s2":baseline["translation_acceleration_mm_s2"]}
            {"translation_step_m":"t_step","rotation_step_deg":"r_step_deg",
             "translation_velocity_mm_s":"tv_mm_s","translation_acceleration_mm_s2":"ta_mm_s2"}
            key={"translation_step_m":"t_step","rotation_step_deg":"r_step_deg",
                 "translation_velocity_mm_s":"tv_mm_s","translation_acceleration_mm_s2":"ta_mm_s2"}[parameter]
            kw[key]=value
            sim=replay(model,rows,pos,**kw)
            accepted=sum(not x["blockers"] for x in sim)
            result.append({"model":model,"parameter":parameter,"value":value,
                           "production_baseline_value":baseline[parameter],
                           "accepted":accepted,"total":len(rows),"accepted_delta":accepted-base_n,
                           "analysis_only":True})
    return result


def leave_one_out(all_rows):
    result=[]
    for model in ("openvla","oft"):
        rows=[r for r in all_rows if r["model"]==model]
        baseline=sum(r["accepted"] for r in rows)
        blockers=sorted({b for r in rows for b in json.loads(r["blockers"])})
        for ignored in blockers:
            accepted=sum(not [b for b in json.loads(r["blockers"]) if b != ignored] for r in rows)
            result.append({"model":model,"ignored_blocker":ignored,"baseline_accepted":baseline,
                           "hypothetical_accepted":accepted,"accepted_gain":accepted-baseline,
                           "note":"posthoc analysis only; stateful downstream effects are not replayed"})
    return result


def chunk_summary(rows):
    out=[]
    for idx in range(5):
        part=[r for r in rows if int(r["chunk_index"])==idx]
        out.append({"chunk_index":idx,"total":len(part),"accepted":sum(r["accepted"] for r in part),
                    "rejected":sum(not r["accepted"] for r in part),
                    "reject_rate":sum(not r["accepted"] for r in part)/len(part) if part else None,
                    "mean_translation_norm_m":float(np.mean([r["translation_norm_m"] for r in part])),
                    "mean_gripper_closedness":float(np.mean([r["gripper_closedness"] for r in part]))})
    return out


def metrics_summary(all_rows, mismatches, sensitivity_rows, loo_rows, combos):
    result={"classification":"EPISODE4_OFFLINE_SAFETY_REJECT_ANALYSIS",
            "production_safety_config_changed":False,"real_robot_commands":0,
            "reproduction":{"expected":{"openvla":[45,16,29],"oft":[45,3,42]},"replay_blocker_mismatches":mismatches},
            "models":{},"top_blocker_combinations":combos,
            "limitations":["accepted/rejected is SafetyPipeline gating, not task success",
                           "TCP source is planned_actual_duration, not measured feedback",
                           "one recorded Episode 4 only","sensitivity is analysis-only"]}
    for model in ("openvla","oft"):
        rows=[r for r in all_rows if r["model"]==model]
        t=[r["translation_norm_m"] for r in rows]; q=[r["rotation_norm_deg"] for r in rows]
        accepted=sum(r["accepted"] for r in rows)
        closes=[r for r in rows if r["gripper_closedness"]>=.7]
        grasp=[r for r in rows if r["phase"]==GRASP_PHASE]
        model_sens=[r for r in sensitivity_rows if r["model"]==model]
        max_gain=max((r["accepted_delta"] for r in model_sens),default=0)
        semantic=sum("premature_gripper_close" in json.loads(r["diagnostic_blockers"]) for r in rows)
        p95_ratio=describe(t)["p95"]/.004
        if max_gain >= math.ceil(.2*len(rows)) and semantic <= .1*len(rows) and p95_ratio <= 1.5:
            cause="SAFETY_THRESHOLD_DOMINANT"
        elif max_gain < math.ceil(.1*len(rows)) and (semantic > .2*len(rows) or p95_ratio > 2):
            cause="MODEL_ACTION_DOMINANT"
        else: cause="MIXED"
        result["models"][model]={
          "total":len(rows),"accepted":accepted,"rejected":len(rows)-accepted,
          "translation_norm_m":describe(t),"rotation_norm_deg":describe(q),
          "translation_exceedance":{"1mm":sum(x>.001 for x in t),"2mm":sum(x>.002 for x in t),
             "3mm":sum(x>.003 for x in t),"4mm":sum(x>.004 for x in t)},
          "rotation_exceedance":{"1deg":sum(x>1 for x in q),"2deg":sum(x>2 for x in q),
             "3deg":sum(x>3 for x in q),"4deg":sum(x>4 for x in q),"5deg":sum(x>5 for x in q)},
          "implied_translation_velocity_mm_s":{"max":max(r["translation_velocity_mm_s"] for r in rows),
             "exceed_20_count":sum(r["translation_velocity_mm_s"]>20 for r in rows)},
          "implied_translation_acceleration_mm_s2":{"max":max(float(r["translation_acceleration_mm_s2"]) for r in rows[1:]),
             "exceed_20_count":sum(r["translation_acceleration_mm_s2"]!="" and float(r["translation_acceleration_mm_s2"])>20 for r in rows)},
          "gripper":{"first_close_candidate_frame":closes[0]["frame_id"] if closes else None,
             "first_grasp_close_frame":grasp[0]["frame_id"] if grasp else None,
             "close_lead_frames":(grasp[0]["frame_id"]-closes[0]["frame_id"]) if closes and grasp else None,
             "premature_close_candidates":semantic},
          "largest_single_threshold_gain":max_gain,"dominant_cause":cause}
    return result


def run_analysis():
    RESULTS.mkdir(parents=True,exist_ok=True);PLOTS.mkdir(parents=True,exist_ok=True)
    open_raw=read_jsonl(OPEN_OUT);oft_raw=read_jsonl(OFT_OUT)
    open_rows,om=enrich_model("openvla",open_raw,OPEN_SRC)
    oft_rows,fm=enrich_model("oft",oft_raw,OFT_SRC)
    all_rows=open_rows+oft_rows
    write_csv(RESULTS/"openvla_action_analysis.csv",open_rows)
    write_csv(RESULTS/"oft_action_analysis.csv",oft_rows)
    b,p,o=blocker_tables(all_rows)
    write_csv(RESULTS/"blocker_summary.csv",b)
    write_csv(RESULTS/"primary_blocker_summary.csv",p)
    write_csv(RESULTS/"blocker_overlap.csv",o)
    write_csv(RESULTS/"openvla_blocker_cooccurrence.csv",[x for x in o if x["model"]=="openvla"])
    write_csv(RESULTS/"oft_blocker_cooccurrence.csv",[x for x in o if x["model"]=="oft"])
    sens=sensitivity("openvla",open_raw,OPEN_SRC)+sensitivity("oft",oft_raw,OFT_SRC)
    write_csv(RESULTS/"threshold_sensitivity.csv",sens)
    loo=leave_one_out(all_rows);write_csv(RESULTS/"leave_one_blocker_out.csv",loo)
    chunks=chunk_summary(oft_rows);write_csv(RESULTS/"oft_chunk_index_summary.csv",chunks)
    combos=top_combinations(all_rows)
    summary=metrics_summary(all_rows,{"openvla":om,"oft":fm},sens,loo,combos)
    summary["production_config_snapshot"]=production_config()
    (RESULTS/"summary.json").write_text(json.dumps(summary,indent=2,ensure_ascii=False)+"\n")
    return all_rows,b,sens,chunks,summary
