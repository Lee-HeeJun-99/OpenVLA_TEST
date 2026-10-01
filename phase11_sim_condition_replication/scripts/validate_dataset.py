#!/usr/bin/env python3
import json
from collections import Counter
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
manifest = json.loads((ROOT / "dataset" / "dataset_manifest.json").read_text(encoding="utf-8"))
counts = Counter()
errors = []
for row in manifest["records"]:
    report = Path(row["report_json"])
    episode = Path(row["episode_dir"])
    if row["condition_id"] == "baseline":
        valid = report.is_file() and (episode / "frames.jsonl").is_file()
    else:
        valid = False
        if report.is_file() and (episode / "frames.jsonl").is_file():
            try:
                valid = json.loads(report.read_text(encoding="utf-8")).get("status") == "complete"
            except Exception:
                pass
    counts[(row["condition_id"], "complete" if valid else "pending_or_invalid")] += 1
    if not row["target_orange_position_invariant"] or not row["trajectory_invariant"]:
        errors.append(f"pairing invariant false: {row['condition_id']} {row['base_episode_id']}")
print(json.dumps({"counts": {"|".join(k): v for k, v in counts.items()}, "errors": errors}, indent=2))
raise SystemExit(1 if errors else 0)
