#!/usr/bin/env python3
"""Align integrated samples without assuming equal indices are synchronized."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--samples", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--max-camera-state-delta-sec", type=float, default=0.05)
    args = parser.parse_args()
    records = [json.loads(line) for line in args.samples.read_text(encoding="utf-8").splitlines() if line.strip()]
    fields = [
        "trial_id", "episode_id", "frame_id", "timestamp_camera", "timestamp_state",
        "timestamp_planner", "camera_state_delta_sec", "clock_domain_comparable",
        "alignment_method", "within_tolerance", "phase", "valid",
    ]
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for record in records:
            camera = record.get("timestamp_camera")
            state = record.get("timestamp_state")
            camera_domain = record.get("camera_clock_domain")
            state_domain = record.get("state_clock_domain")
            comparable = (
                camera is not None
                and state is not None
                and camera_domain not in (None, "unknown_source_clock")
                and camera_domain == state_domain
            )
            delta = abs(float(camera) - float(state)) if comparable else None
            writer.writerow({
                "trial_id": record["trial_id"], "episode_id": record["episode_id"],
                "frame_id": record["frame_id"], "timestamp_camera": camera,
                "timestamp_state": state, "timestamp_planner": record.get("timestamp_planner"),
                "camera_state_delta_sec": delta,
                "clock_domain_comparable": comparable,
                "alignment_method": "timestamp" if comparable else "frame_id_with_recorded_callback_ages",
                "within_tolerance": delta is not None and delta <= args.max_camera_state_delta_sec,
                "phase": record.get("phase"), "valid": record.get("valid"),
            })
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
