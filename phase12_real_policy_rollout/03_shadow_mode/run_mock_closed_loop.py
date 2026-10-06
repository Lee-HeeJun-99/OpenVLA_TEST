#!/usr/bin/env python3
"""Deterministic software-only pipeline smoke test."""
from __future__ import annotations
import argparse,json,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT/'02_safety'),str(ROOT/'03_shadow_mode')]
from canonical_action import CanonicalAction
from mock_closed_loop import MockClosedLoop,MockRobotState
from integrated_logger import FsyncJsonlLogger
def main():
 p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args()
 actions=[CanonicalAction.from_vector([.0005,0,0,0,0,0,0],timestamp_monotonic=.2*(i+1),sequence_id=f'mock-{i}',source_model='mock') for i in range(5)]
 rows=MockClosedLoop().run(MockRobotState([.4,0,.5],[10,20,30]),actions)
 with FsyncJsonlLogger(a.output,minimum_free_bytes=0) as log:
  for row in rows:log.append(row)
 print(json.dumps({'status':'MOCK_CLOSED_LOOP_COMPLETE','records':len(rows),'accepted':sum(r['decision']['accepted'] for r in rows),'command_issued':False}))
if __name__=='__main__':main()
