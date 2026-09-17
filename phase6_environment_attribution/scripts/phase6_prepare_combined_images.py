#!/usr/bin/env python3
from __future__ import annotations

import json
import shutil
from pathlib import Path

from PIL import Image, ImageChops


ROOT = Path("/home/ubuntu/a0509_vla_linux_field_bundle_20260903")
PHASE6 = ROOT / "lhj" / "phase6_environment_attribution"
PREPROCESS_INPUT = PHASE6 / "01_preprocessing" / "condition_inputs"
OUT = PHASE6 / "05_combined" / "condition_inputs"


def write_json(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def shift_image(image: Image.Image, dx: int, dy: int) -> Image.Image:
    shifted = ImageChops.offset(image, dx, dy)
    w, h = image.size
    fill = Image.new("RGB", image.size, (0, 0, 0))
    if dx > 0:
        shifted.paste(fill.crop((0, 0, dx, h)), (0, 0))
    elif dx < 0:
        shifted.paste(fill.crop((0, 0, -dx, h)), (w + dx, 0))
    if dy > 0:
        shifted.paste(fill.crop((0, 0, w, dy)), (0, 0))
    elif dy < 0:
        shifted.paste(fill.crop((0, 0, w, -dy)), (0, h + dy))
    return shifted


def main() -> int:
    if OUT.exists():
        shutil.rmtree(OUT)
    OUT.mkdir(parents=True, exist_ok=True)

    conditions = {
        "K0_P4_letterbox": {
            "description": "P4 letterbox condition copied as combined baseline.",
            "base": "P4_letterbox_224",
            "sim_shift_x_px": 0,
        },
        "K1_P4_letterbox_C2_shift_right_24px": {
            "description": "P4 letterbox plus C2-like Sim right shift by 24 pixels.",
            "base": "P4_letterbox_224",
            "sim_shift_x_px": 24,
        },
    }

    manifest: dict[str, dict] = {}
    for name, cfg in conditions.items():
        base = PREPROCESS_INPUT / cfg["base"]
        real_dir = OUT / name / "real"
        sim_dir = OUT / name / "sim"
        real_dir.mkdir(parents=True, exist_ok=True)
        sim_dir.mkdir(parents=True, exist_ok=True)
        real_images = []
        sim_images = []
        for real_path in sorted((base / "real").glob("*.jpg")):
            dst = real_dir / real_path.name
            shutil.copy2(real_path, dst)
            real_images.append(str(dst))
        for sim_path in sorted((base / "sim").glob("*.jpg")):
            dst = sim_dir / sim_path.name
            image = Image.open(sim_path).convert("RGB")
            if cfg["sim_shift_x_px"]:
                image = shift_image(image, int(cfg["sim_shift_x_px"]), 0)
            image.save(dst, quality=95)
            sim_images.append(str(dst))
        (OUT / name / "real_images.txt").write_text("\n".join(real_images) + "\n", encoding="utf-8")
        (OUT / name / "sim_images.txt").write_text("\n".join(sim_images) + "\n", encoding="utf-8")
        manifest[name] = {
            **cfg,
            "real_images_txt": str(OUT / name / "real_images.txt"),
            "sim_images_txt": str(OUT / name / "sim_images.txt"),
            "count": len(real_images),
        }

    write_json(
        OUT / "combined_condition_manifest.json",
        {
            "conditions": manifest,
            "status": "COMBINED_P4_C2_INPUTS_READY",
            "interpretation": "K1 is a controlled image-space combined sensitivity condition, not calibrated camera alignment.",
        },
    )
    print(json.dumps({"conditions": list(manifest), "count": next(iter(manifest.values()))["count"]}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
