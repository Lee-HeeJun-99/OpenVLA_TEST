#!/usr/bin/env python3
"""Generate all offline Episode 4 reject tables, plots, and summary."""
from analysis_core import run_analysis
from make_plots import make_plots

if __name__ == "__main__":
    rows, blockers, sensitivity, chunks, summary = run_analysis()
    make_plots(rows, blockers, sensitivity, chunks)
    for model in ("openvla", "oft"):
        m=summary["models"][model]
        print(f"{model}: total={m['total']} accepted={m['accepted']} rejected={m['rejected']} cause={m['dominant_cause']}")

