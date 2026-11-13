#!/usr/bin/env /usr/bin/python3
"""FIELD DRAFT: stream the measured pose unchanged for N seconds.

Required reviewed behavior: 100 Hz telemetry, Enter abort, immediate stream
inhibit + MoveStop, and summary.json.  This offline revision is fail-closed.
"""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from runtime.servoj.session_contract import field_draft_main
if __name__=="__main__":field_draft_main("servoj_hold_test","Hold current joints with ServoJ; arm_stationary evidence")
