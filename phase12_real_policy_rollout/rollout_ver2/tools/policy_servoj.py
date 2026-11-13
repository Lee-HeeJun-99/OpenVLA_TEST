#!/usr/bin/env /usr/bin/python3
"""ServoJ policy Mode-A field entry-point draft.

Reviewed architecture contract:
  process A owns feedback/safety, the dedicated 100 Hz ServoJ stream thread,
  stream inhibition and MoveStop; it never handles images.
  process B owns camera/new-frame wait, JPEG and HTTP inference.
Each action is fully planned and checked before process A sends sample zero.
The current offline revision intentionally creates neither process nor ROS
command object because ServoJ field gates remain false.
"""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from runtime.servoj.session_contract import field_draft_main
if __name__=="__main__":field_draft_main("policy","Settled-observation policy using fixed-duration ServoJ segments")
