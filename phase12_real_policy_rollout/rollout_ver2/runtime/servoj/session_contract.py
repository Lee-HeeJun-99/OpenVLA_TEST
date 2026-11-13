"""Fail-closed contracts shared by field-tool drafts; no ROS imports."""
from __future__ import annotations
import json,time
from pathlib import Path

def require_confirmation(mode,text):
    if text != "I_CONFIRM_V2_MODE:"+mode:raise PermissionError("FRESH_HUMAN_CONFIRMATION_REQUIRED")

class OperatorAbort:
    def __init__(self,stop_callback):self.stop_callback=stop_callback;self.requested=False;self.response_s=None
    def request(self):
        if self.requested:return
        self.requested=True;started=time.monotonic();self.stop_callback();self.response_s=time.monotonic()-started

def write_summary(run_dir,payload):
    path=Path(run_dir);path.mkdir(parents=True,exist_ok=True)
    (path/"summary.json").write_text(json.dumps(payload,indent=2)+"\n")

def field_draft_main(mode,description):
    import argparse
    ap=argparse.ArgumentParser(description=description)
    ap.add_argument("--enable-physical",action="store_true");ap.add_argument("--confirmation")
    ap.add_argument("--output",type=Path,required=True)
    ap.add_argument("--duration",type=float);ap.add_argument("--segment-T",type=float,choices=(.4,.6,1.0))
    ap.add_argument("--send-rate-hz",type=float,default=100.0);ap.add_argument("--time-param",type=float)
    args=ap.parse_args()
    if not args.enable_physical:raise PermissionError("PHYSICAL_EXECUTION_DISABLED")
    confirmation=args.confirmation
    if confirmation is None:confirmation=input("Type I_CONFIRM_V2_MODE:"+mode+" exactly: ")
    require_confirmation(mode,confirmation)
    # This offline deliverable must not construct ROS command objects.  Session
    # implementation is unlocked only after review of its still-false gates.
    write_summary(args.output,{"mode":mode,"pass":False,"end_reason":"FIELD_ADAPTER_REVIEW_REQUIRED",
      "commands_sent":0,"telemetry_required":True,"enter_abort_required":True,
      "move_stop_on_abort_required":True})
    raise RuntimeError("FIELD_ADAPTER_REVIEW_REQUIRED")
