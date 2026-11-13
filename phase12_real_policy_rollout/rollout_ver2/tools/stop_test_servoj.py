#!/usr/bin/env /usr/bin/python3
"""FIELD DRAFT: inhibit stream then DR_QSTOP=1 during ServoJ motion."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from runtime.servoj.session_contract import field_draft_main
def main():field_draft_main("stop_test_servoj","Measure ServoJ stop time and post-request travel")
if __name__=="__main__":main()
