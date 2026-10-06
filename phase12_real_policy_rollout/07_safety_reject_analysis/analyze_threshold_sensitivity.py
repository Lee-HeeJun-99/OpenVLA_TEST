#!/usr/bin/env python3
"""Regenerate the analysis-only marginal threshold sweep."""
from analysis_core import OPEN_OUT,OFT_OUT,OPEN_SRC,OFT_SRC,RESULTS,read_jsonl,sensitivity,write_csv
if __name__ == "__main__":
    rows=sensitivity("openvla",read_jsonl(OPEN_OUT),OPEN_SRC)+sensitivity("oft",read_jsonl(OFT_OUT),OFT_SRC)
    write_csv(RESULTS/"threshold_sensitivity.csv",rows)
    print(f"wrote {len(rows)} analysis-only sensitivity rows")

