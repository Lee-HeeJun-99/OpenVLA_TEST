#!/usr/bin/env python3
"""Fail closed when a prediction-only episode contains missing/failed inference."""
import argparse
import json
from pathlib import Path

p = argparse.ArgumentParser()
p.add_argument("--model", choices=("openvla", "oft"), required=True)
p.add_argument("--samples", type=Path, required=True)
a = p.parse_args()
rows = [json.loads(line) for line in a.samples.read_text().splitlines() if line.strip()]
key = "openvla_canonical_action" if a.model == "openvla" else "oft_canonical_action_chunk"
expected = len(rows) if a.model == "openvla" else (len(rows) + 4) // 5
actual = sum(row.get(key) is not None for row in rows)
errors = [row for row in rows if row.get("inference_errors")]
fixtures = sum(bool(row.get("fixture_smoke_test_only")) for row in rows)
if not rows or actual != expected or errors or fixtures:
    raise SystemExit(
        f"INVALID prediction output: rows={len(rows)} expected={expected} "
        f"actual={actual} errors={len(errors)} fixtures={fixtures}"
    )
print(json.dumps({"status":"PASS","model":a.model,"rows":len(rows),"predictions":actual}))
