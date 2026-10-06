#!/usr/bin/env python3
"""Recorded prediction → canonical → safety → NullCommandSink pipeline."""
from __future__ import annotations
import argparse,json,sys
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation

ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT/'02_safety'),str(ROOT/'03_shadow_mode')]
from canonical_action import CanonicalAction
from command_sinks import NullCommandSink
from safety_pipeline import RuntimeState,SafetyPipeline
from integrated_logger import FsyncJsonlLogger

def abc(rotation_xyzw):
    if not rotation_xyzw:return (0.,0.,0.)
    return tuple(Rotation.from_quat(rotation_xyzw).as_euler('ZYZ',degrees=True))

def actions_for(row,model):
    if model=='openvla':
        value=row.get('openvla_denormalized_action') or row.get('openvla_raw_action')
        if isinstance(value,dict):value=value.get('action')
        return [value] if value else []
    values=row.get('oft_denormalized_action_chunk')
    if not values and isinstance(row.get('oft_raw_action_chunk'),dict):values=row['oft_raw_action_chunk'].get('actions')
    return values or []

def main():
    p=argparse.ArgumentParser();p.add_argument('--model',choices=('openvla','oft'),required=True)
    p.add_argument('--input',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--command-mode',choices=('disabled',),default='disabled');p.add_argument('--limit',type=int,default=0)
    a=p.parse_args();pipeline=SafetyPipeline(a.model,operator_confirmed_initial_open=True);sink=NullCommandSink();n=accepted=0
    with FsyncJsonlLogger(a.output,minimum_free_bytes=0) as log:
     for line in a.input.open():
      row=json.loads(line);acts=actions_for(row,a.model)
      if not acts:continue
      base=float(row.get('frame_id',n))*(1.0 if a.model=='oft' else .2)
      pos=tuple(float(x) for x in (row.get('raw_ee_position') or (.4,0,.5)))
      for k,vector in enumerate(acts):
       action=CanonicalAction.from_vector(vector,timestamp_monotonic=base,
        sequence_id=f"{row.get('episode_id','recorded')}-{row.get('frame_id',n)}-k{k}",source_model=a.model,
        chunk_index=k,chunk_size=len(acts))
       state=RuntimeState(base+.2*k,pos,abc(row.get('raw_ee_rotation')),row.get('phase') or 'unknown')
       decision=pipeline.inspect(action,state);receipt=sink.submit(action.as_dict(),decision.as_dict())
       out={"classification":"RECORDED_PREDICTION_COMMAND_DISABLED_SHADOW","episode_id":row.get('episode_id'),
        "frame_id":row.get('frame_id'),"phase":state.phase,"image_sha256":row.get('raw_image_sha256'),
        "tcp_source":row.get('pose_source','RECORDED_TCP'),"canonical_action":action.as_dict(),
        "filtered_action":decision.as_dict()['filtered_action'],"safety_decision":decision.as_dict(),
        "sink_receipt":receipt,"latency":row.get('inference_latency_openvla') if a.model=='openvla' else row.get('inference_latency_oft'),
        "command_issued":False,"executed_action":None,"robot_delivered_command":None}
       log.append(out);n+=1;accepted+=int(decision.accepted)
       if a.limit and n>=a.limit:break
      if a.limit and n>=a.limit:break
    print(json.dumps({'status':'COMPLETE_COMMAND_DISABLED','model':a.model,'records':n,'accepted':accepted,
                      'rejected':n-accepted,'command_issued':False,'output':str(a.output)}))
if __name__=='__main__':main()

