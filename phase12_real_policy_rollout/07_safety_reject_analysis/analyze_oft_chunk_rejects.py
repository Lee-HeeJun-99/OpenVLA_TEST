#!/usr/bin/env python3
"""Regenerate OFT K=5 chunk-index reject statistics."""
from analysis_core import OFT_OUT,OFT_SRC,RESULTS,chunk_summary,enrich_model,read_jsonl,write_csv
if __name__ == "__main__":
    rows,_=enrich_model("oft",read_jsonl(OFT_OUT),OFT_SRC)
    out=chunk_summary(rows);write_csv(RESULTS/"oft_chunk_index_summary.csv",out)
    for row in out: print(row)
