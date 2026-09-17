#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import shutil
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from PIL import Image, ImageChops


ROOT = Path("/home/ubuntu/a0509_vla_linux_field_bundle_20260903")
PHASE6 = ROOT / "lhj" / "phase6_environment_attribution"
ANALYSIS = ROOT / "outputs" / "token_distribution_analysis" / "5_episodes"


@dataclass(frozen=True)
class CameraCondition:
    name: str
    description: str
    transform: str
    value: float


CONDITIONS = [
    CameraCondition("C0_original", "No image-space camera perturbation.", "identity", 0.0),
    CameraCondition("C1_sim_shift_left_24px", "Shift Sim image left by 24 px.", "shift_x", -24.0),
    CameraCondition("C2_sim_shift_right_24px", "Shift Sim image right by 24 px.", "shift_x", 24.0),
    CameraCondition("C3_sim_shift_up_24px", "Shift Sim image up by 24 px.", "shift_y", -24.0),
    CameraCondition("C4_sim_shift_down_24px", "Shift Sim image down by 24 px.", "shift_y", 24.0),
    CameraCondition("C5_sim_zoom_in_1p05", "Zoom Sim image in around center by 1.05x.", "scale", 1.05),
    CameraCondition("C6_sim_zoom_out_0p95", "Zoom Sim image out around center by 0.95x.", "scale", 0.95),
    CameraCondition("C7_sim_rotate_ccw_2deg", "Rotate Sim image counter-clockwise by 2 degrees.", "rotate", 2.0),
    CameraCondition("C8_sim_rotate_cw_2deg", "Rotate Sim image clockwise by 2 degrees.", "rotate", -2.0),
]


def read_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def load_rgb(path: str | Path) -> Image.Image:
    return Image.open(path).convert("RGB")


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


def scale_center(image: Image.Image, scale: float) -> Image.Image:
    w, h = image.size
    nw = max(1, int(round(w * scale)))
    nh = max(1, int(round(h * scale)))
    resized = image.resize((nw, nh), Image.Resampling.BICUBIC)
    canvas = Image.new("RGB", (w, h), (0, 0, 0))
    if scale >= 1.0:
        left = (nw - w) // 2
        top = (nh - h) // 2
        return resized.crop((left, top, left + w, top + h))
    left = (w - nw) // 2
    top = (h - nh) // 2
    canvas.paste(resized, (left, top))
    return canvas


def transform_sim(image: Image.Image, condition: CameraCondition) -> Image.Image:
    if condition.transform == "identity":
        return image.copy()
    if condition.transform == "shift_x":
        return shift_image(image, int(condition.value), 0)
    if condition.transform == "shift_y":
        return shift_image(image, 0, int(condition.value))
    if condition.transform == "scale":
        return scale_center(image, float(condition.value))
    if condition.transform == "rotate":
        return image.rotate(float(condition.value), resample=Image.Resampling.BICUBIC, expand=False, fillcolor=(0, 0, 0))
    raise KeyError(condition.transform)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", type=Path, default=PHASE6 / "03_camera" / "condition_inputs")
    parser.add_argument("--quality", type=int, default=95)
    parser.add_argument("--limit", type=int, default=0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    out = args.output_dir.expanduser().resolve()
    if out.exists():
        shutil.rmtree(out)
    out.mkdir(parents=True, exist_ok=True)

    pairs = read_json(ANALYSIS / "paired_inputs" / "paired_manifest.json")["pairs"]
    if args.limit and args.limit > 0:
        pairs = pairs[: args.limit]

    records = {}
    for condition in CONDITIONS:
        real_dir = out / condition.name / "real"
        sim_dir = out / condition.name / "sim"
        real_dir.mkdir(parents=True, exist_ok=True)
        sim_dir.mkdir(parents=True, exist_ok=True)
        real_list = []
        sim_list = []
        for pair in pairs:
            real = load_rgb(pair["real"]["paired_image"])
            sim = transform_sim(load_rgb(pair["sim"]["paired_image"]), condition)
            filename = f"{pair['pair_id']}.jpg"
            real_path = real_dir / filename
            sim_path = sim_dir / filename
            real.save(real_path, quality=args.quality)
            sim.save(sim_path, quality=args.quality)
            real_list.append(str(real_path))
            sim_list.append(str(sim_path))
        (out / condition.name / "real_images.txt").write_text("\n".join(real_list) + "\n", encoding="utf-8")
        (out / condition.name / "sim_images.txt").write_text("\n".join(sim_list) + "\n", encoding="utf-8")
        records[condition.name] = {
            "description": condition.description,
            "transform": condition.transform,
            "value": condition.value,
            "real_images_txt": str(out / condition.name / "real_images.txt"),
            "sim_images_txt": str(out / condition.name / "sim_images.txt"),
            "count": len(real_list),
        }

    payload = {
        "analysis_root": str(ANALYSIS),
        "output_dir": str(out),
        "pair_count": len(pairs),
        "conditions": records,
        "status": "IMAGE_SPACE_CAMERA_SENSITIVITY_INPUTS_READY",
        "interpretation": "These are image-space camera-like perturbations, not calibrated Real camera alignment.",
    }
    write_json(out / "camera_condition_manifest.json", payload)
    print(json.dumps(payload, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
