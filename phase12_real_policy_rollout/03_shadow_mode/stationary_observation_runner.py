#!/usr/bin/env python3
"""Recorded/mock stationary observation runner with no command capability."""
from __future__ import annotations
import argparse, hashlib, json, math
from pathlib import Path
from integrated_logger import FsyncJsonlLogger

def main():
    p=argparse.ArgumentParser();p.add_argument('--input',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--limit',type=int,default=30)
    a=p.parse_args();count=0;valid=0
    with FsyncJsonlLogger(a.output,minimum_free_bytes=0) as log:
      with a.input.open() as source:
       for line in source:
        if count>=a.limit:break
        raw=json.loads(line);image_path=raw.get('raw_image_path') or raw.get('model_input_image_path')
        image_hash=raw.get('raw_image_sha256')
        if image_path and not image_hash and Path(image_path).is_file():image_hash=hashlib.sha256(Path(image_path).read_bytes()).hexdigest()
        position=raw.get('raw_joint_position') or []
        state_ok=len(position)>=6 and all(math.isfinite(float(v)) for v in position[:6])
        record={"classification":"STATIONARY_OBSERVATION_RECORDED_DATA_TEST","frame_id":raw.get('frame_id',count),
         "image_path":image_path,"image_sha256":image_hash,"camera_source_timestamp":raw.get('timestamp_camera'),
         "joint_state":{"position":position[:6],"source_timestamp":raw.get('timestamp_state')},
         "tcp_source":"RECORDED_TCP" if raw.get('raw_ee_position') else "UNAVAILABLE_TCP",
         "tcp_pose":{"position_m":raw.get('raw_ee_position'),"rotation_xyzw":raw.get('raw_ee_rotation')},
         "valid":bool(image_hash and state_ok),"command_issued":False,"executed_action":None,
         "robot_delivered_command":None}
        log.append(record);count+=1;valid+=int(record['valid'])
    print(json.dumps({'status':'COMPLETE_RECORDED_ONLY','records':count,'valid':valid,
                      'command_issued':False,'output':str(a.output)}))
if __name__=='__main__':main()

