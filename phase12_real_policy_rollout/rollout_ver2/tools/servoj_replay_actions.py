#!/usr/bin/env /usr/bin/python3
"""FIELD DRAFT: replay recorded actions as settled Mode-A segments."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from runtime.servoj.session_contract import field_draft_main
if __name__=="__main__":field_draft_main("servoj_replay_actions","Replay actions and compare each settled pose to MoveJ")
