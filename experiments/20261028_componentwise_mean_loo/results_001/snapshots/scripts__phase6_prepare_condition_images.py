#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import shutil
from pathlib import Path

from phase6_preprocessing_audit import ANALYSIS, CONDITIONS, PHASE6, load_rgb, transform_pair, write_json


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", type=Path, default=PHASE6 / "01_preprocessing" / "condition_inputs")
    parser.add_argument("--quality", type=int, default=95)
    parser.add_argument("--limit", type=int, default=0, help="0 means all pairs.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    out = args.output_dir.expanduser().resolve()
    if out.exists():
        shutil.rmtree(out)
    out.mkdir(parents=True, exist_ok=True)

    manifest = json.loads((ANALYSIS / "paired_inputs" / "paired_manifest.json").read_text(encoding="utf-8"))
    pairs = manifest["pairs"]
    if args.limit and args.limit > 0:
        pairs = pairs[: args.limit]

    condition_records = {}
    for condition in CONDITIONS:
        real_dir = out / condition.name / "real"
        sim_dir = out / condition.name / "sim"
        real_dir.mkdir(parents=True, exist_ok=True)
        sim_dir.mkdir(parents=True, exist_ok=True)
        real_list = []
        sim_list = []
        for pair in pairs:
            real_base = load_rgb(pair["real"]["paired_image"])
            sim_base = load_rgb(pair["sim"]["paired_image"])
            real_img, sim_img = transform_pair(condition.name, real_base, sim_base)
            filename = f"{pair['pair_id']}.jpg"
            real_path = real_dir / filename
            sim_path = sim_dir / filename
            real_img.save(real_path, quality=args.quality)
            sim_img.save(sim_path, quality=args.quality)
            real_list.append(str(real_path))
            sim_list.append(str(sim_path))
        (out / condition.name / "real_images.txt").write_text("\n".join(real_list) + "\n", encoding="utf-8")
        (out / condition.name / "sim_images.txt").write_text("\n".join(sim_list) + "\n", encoding="utf-8")
        condition_records[condition.name] = {
            "family": condition.family,
            "description": condition.description,
            "real_images_txt": str(out / condition.name / "real_images.txt"),
            "sim_images_txt": str(out / condition.name / "sim_images.txt"),
            "count": len(real_list),
        }

    payload = {
        "analysis_root": str(ANALYSIS),
        "output_dir": str(out),
        "pair_count": len(pairs),
        "conditions": condition_records,
    }
    write_json(out / "condition_image_manifest.json", payload)
    print(json.dumps(payload, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
