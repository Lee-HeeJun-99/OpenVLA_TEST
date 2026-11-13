#!/usr/bin/env python3
"""Run with Isaac Sim python.sh; delegates to the immutable reference renderer."""
import runpy, sys
from pathlib import Path
here = Path(__file__).resolve().parent
sys.argv = [str(here.parent / "render_matched15_layout.py"), "--pose-json", str(here / "pose.json")]
runpy.run_path(sys.argv[0], run_name="__main__")
