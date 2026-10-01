#!/usr/bin/env python3
"""Resume-safe launcher for the 15 new Sim replay episodes.

Dry-run is the default. Pass --execute to launch Isaac Sim. Existing complete
outputs are skipped, and existing incomplete outputs are never overwritten.
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
from datetime import datetime, timezone
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "config" / "experiment.json"
MANIFEST = ROOT / "dataset" / "dataset_manifest.json"
LOG = ROOT / "work_log.md"


def append_log(message: str) -> None:
    stamp = datetime.now(timezone.utc).astimezone().isoformat(timespec="seconds")
    with LOG.open("a", encoding="utf-8") as handle:
        handle.write(f"\n- `{stamp}` {message}\n")
        handle.flush()
        os.fsync(handle.fileno())


def free_gb(path: Path) -> float:
    return shutil.disk_usage(path).free / (1024 ** 3)


def complete(row: dict) -> bool:
    report = Path(row["report_json"])
    if not report.is_file():
        return False
    try:
        payload = json.loads(report.read_text(encoding="utf-8"))
    except Exception:
        return False
    episode_dir = Path(row["episode_dir"])
    frames = episode_dir / "frames.jsonl"
    return payload.get("status") == "complete" and frames.is_file()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--execute", action="store_true", help="Actually launch Isaac Sim")
    parser.add_argument("--condition", choices=["lighting_low", "extra_object", "distractor_swap"])
    parser.add_argument("--episode", type=int, choices=range(1, 6))
    args = parser.parse_args()
    cfg = json.loads(CONFIG.read_text(encoding="utf-8"))
    manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))
    minimum = float(cfg["storage"]["minimum_free_gb"])
    rows = [r for r in manifest["records"] if r["condition_id"] != "baseline"]
    if args.condition:
        rows = [r for r in rows if r["condition_id"] == args.condition]
    if args.episode:
        rows = [r for r in rows if r["base_episode_id"] == f"episode_{args.episode:06d}"]

    for row in rows:
        report = Path(row["report_json"])
        if complete(row):
            print(f"SKIP complete: {row['condition_id']} {row['base_episode_id']}")
            continue
        if report.exists() or Path(row["episode_dir"]).exists():
            raise SystemExit(f"Refusing to overwrite incomplete output: {report}")
        available = free_gb(Path(cfg["dataset_root"]))
        if available < minimum:
            append_log(f"BLOCKED_LOW_DISK free_gb={available:.2f} threshold_gb={minimum:.2f}")
            raise SystemExit(f"Free disk {available:.2f} GB is below {minimum:.2f} GB")
        report.parent.mkdir(parents=True, exist_ok=True)
        cmd = [
            cfg["isaac_python"], cfg["control_app"], "--headless",
            "--scene", row["scene"],
            "--manual-layout-json", row["layout_json"],
            "--joint-replay-steps-jsonl", row["source_steps_jsonl"],
            "--joint-replay-output", row["report_json"],
            "--joint-replay-rate-hz", "5.0", "--joint-replay-home-relative",
            "--joint-replay-snap-before-capture", "--joint-replay-attach-cube-on-close",
            "--disable-secondary-camera", "--steps", "1",
        ]
        print(" ".join(cmd))
        if not args.execute:
            continue
        env = os.environ.copy()
        env["A0509_PROJECT_ROOT"] = "/home/ubuntu/a0509_vla_linux_field_bundle_20260903/validation/a0509_project"
        log_path = report.with_suffix(".log")
        append_log(f"START {row['condition_id']} {row['base_episode_id']} free_gb={available:.2f}")
        with log_path.open("w", encoding="utf-8") as log_handle:
            result = subprocess.run(cmd, env=env, stdout=log_handle, stderr=subprocess.STDOUT)
        if result.returncode != 0 or not complete(row):
            append_log(f"FAILED {row['condition_id']} {row['base_episode_id']} returncode={result.returncode}")
            raise SystemExit(f"Collection failed; inspect {log_path}")
        append_log(
            f"COMPLETE {row['condition_id']} {row['base_episode_id']} "
            f"free_gb_after={free_gb(Path(cfg['dataset_root'])):.2f}"
        )


if __name__ == "__main__":
    main()
