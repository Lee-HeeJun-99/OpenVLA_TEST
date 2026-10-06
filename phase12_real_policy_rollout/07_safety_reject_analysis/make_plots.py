#!/usr/bin/env python3
"""Deterministic plots for the offline reject analysis."""
import json
from collections import defaultdict

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from analysis_core import PLOTS

COLORS={"openvla":"#377eb8","oft":"#e41a1c"}

def save(fig,name):
    fig.tight_layout(); fig.savefig(PLOTS/name,dpi=180,bbox_inches="tight"); plt.close(fig)

def make_plots(rows, blockers, sensitivity, chunks):
    PLOTS.mkdir(parents=True,exist_ok=True)
    for model,num in (("openvla","01"),("oft","02")):
        data=[x for x in blockers if x["model"]==model]
        fig,ax=plt.subplots(figsize=(9,4.8)); names=[x["blocker"] for x in data]; vals=[x["action_count"] for x in data]
        ax.barh(names[::-1],vals[::-1],color=COLORS[model]); ax.set_xlabel("Actions containing blocker (multi-count)")
        ax.set_title(f"Episode 4 {model.upper()} safety blockers")
        save(fig,f"{num}_blocker_count_{model}.png")

    fig,ax=plt.subplots(figsize=(8,4.8))
    for model in COLORS:
        x=[r["translation_norm_m"]*1000 for r in rows if r["model"]==model]
        ax.hist(x,bins=16,alpha=.55,label=model,color=COLORS[model])
    ax.axvline(4,color="black",ls="--",label="production 4 mm");ax.set_xlabel("Raw translation norm (mm)");ax.set_ylabel("Actions");ax.legend();ax.set_title("Raw translation magnitude")
    save(fig,"03_translation_norm_distribution.png")

    fig,ax=plt.subplots(figsize=(8,4.8))
    for model in COLORS:
        x=[r["rotation_norm_deg"] for r in rows if r["model"]==model]
        ax.hist(x,bins=16,alpha=.55,label=model,color=COLORS[model])
    ax.axvline(4,color="black",ls="--",label="production 4 deg");ax.set_xlabel("Raw rotvec magnitude (deg)");ax.set_ylabel("Actions");ax.legend();ax.set_title("Raw rotation magnitude")
    save(fig,"04_rotation_norm_distribution.png")

    fig,ax=plt.subplots(figsize=(8,4.8))
    for model in COLORS:
        x=[r["translation_velocity_mm_s"] for r in rows if r["model"]==model]
        ax.plot(x,label=model,color=COLORS[model],alpha=.85)
    ax.axhline(20,color="black",ls="--",label="production 20 mm/s");ax.set_xlabel("5 Hz action index");ax.set_ylabel("Implied raw translation speed (mm/s)");ax.legend();ax.set_title("Implied translation velocity")
    save(fig,"05_velocity_distribution.png")

    fig,ax=plt.subplots(figsize=(8,4.8))
    for model in COLORS:
        x=[np.nan if r["translation_acceleration_mm_s2"]=="" else float(r["translation_acceleration_mm_s2"]) for r in rows if r["model"]==model]
        ax.plot(x,label=model,color=COLORS[model],alpha=.85)
    ax.axhline(20,color="black",ls="--",label="production 20 mm/s²");ax.set_xlabel("5 Hz action index");ax.set_ylabel("Implied raw translation acceleration (mm/s²)");ax.legend();ax.set_title("Implied translation acceleration")
    save(fig,"06_acceleration_distribution.png")

    fig,ax=plt.subplots(figsize=(9,4.8))
    for model in COLORS:
        part=[r for r in rows if r["model"]==model]
        ax.plot(range(len(part)),[r["gripper_closedness"] for r in part],label=model,color=COLORS[model])
    ax.axhline(.3,color="gray",ls=":",label="open threshold");ax.axhline(.7,color="black",ls="--",label="close threshold")
    ax.set_xlabel("Action index");ax.set_ylabel("Canonical closedness");ax.set_ylim(-.05,1.05);ax.legend(ncol=2);ax.set_title("Gripper closedness timeline")
    save(fig,"07_gripper_closedness_timeline.png")

    fig,axes=plt.subplots(2,1,figsize=(10,4.8),sharex=True)
    for ax,model in zip(axes,COLORS):
        part=[r for r in rows if r["model"]==model]
        y=[1 if r["accepted"] else 0 for r in part]
        ax.scatter(range(len(part)),y,c=["#4daf4a" if v else "#e41a1c" for v in y],s=24)
        ax.set_yticks([0,1],["reject","accept"]);ax.set_title(model.upper())
    axes[-1].set_xlabel("Action index");fig.suptitle("SafetyPipeline accept/reject (not task success)")
    save(fig,"08_accept_reject_timeline.png")

    fig,ax=plt.subplots(figsize=(7,4.5));ax.bar([r["chunk_index"] for r in chunks],[100*r["reject_rate"] for r in chunks],color="#e41a1c")
    ax.set_xticks(range(5));ax.set_ylim(0,105);ax.set_xlabel("OFT chunk index");ax.set_ylabel("Reject rate (%)");ax.set_title("OFT K=5 reject rate by chunk index")
    save(fig,"09_oft_chunk_index_reject_rate.png")

    params=sorted(set(r["parameter"] for r in sensitivity))
    fig,axes=plt.subplots(2,2,figsize=(10,7.5));
    for ax,param in zip(axes.flat,params):
        for model in COLORS:
            part=[r for r in sensitivity if r["model"]==model and r["parameter"]==param]
            ax.plot([r["value"] for r in part],[r["accepted"] for r in part],marker="o",label=model,color=COLORS[model])
        ax.set_title(param);ax.set_ylabel("Accepted / 45");ax.grid(alpha=.25)
    axes[0,0].legend();fig.suptitle("Analysis-only marginal threshold sensitivity (production unchanged)")
    save(fig,"10_threshold_sensitivity.png")

