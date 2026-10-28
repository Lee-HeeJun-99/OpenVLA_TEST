#!/usr/bin/env python3
from __future__ import annotations

import argparse
import importlib.util
import json
import os
import sys
import time
from pathlib import Path
from types import MethodType
from typing import Any

import numpy as np

from sim2real_analysis.common.feature_capture import (
    ModuleFeatureCapture,
    save_feature_npz,
)
from sim2real_analysis.common.image_stats import load_rgb
from sim2real_analysis.common.samples import (
    records_from_dataset,
    records_from_images,
    write_json,
)


ROOT = Path(__file__).resolve().parents[2]
OFT_REPO = ROOT / "runtime" / "openvla-oft"
VANILLA_SUPPORT = ROOT / "runtime" / "vanilla-support"
DATASET_KEY = "a0509_sim_cube_pick"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Extract VLA internal features offline.")
    parser.add_argument("--model", choices=("vanilla", "oft"), required=True)
    input_group = parser.add_mutually_exclusive_group(required=True)
    input_group.add_argument("--dataset-root", type=Path)
    input_group.add_argument("--image-input", type=Path, nargs="+")
    parser.add_argument("--domain", choices=("sim", "real"), required=True)
    parser.add_argument("--label", default="standalone")
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--split", default=None)
    parser.add_argument("--max-samples", type=int, default=32)
    parser.add_argument(
        "--first-per-episode",
        action="store_true",
        help="Use only the first available frame from each episode.",
    )
    parser.add_argument("--full", action="store_true", help="Store full tensors instead of pooled features.")
    parser.add_argument("--instruction", default=None)
    parser.add_argument("--checkpoint", type=Path, default=None)
    parser.add_argument("--base-model", type=Path, default=ROOT / "models" / "base_openvla_7b")
    parser.add_argument("--variant", default="oftplus_h5_vision")
    parser.add_argument("--local-files-only", action="store_true")
    parser.add_argument("--extra-module", action="append", default=[])
    return parser.parse_args()


def _import_file(module_name: str, path: Path):
    spec = importlib.util.spec_from_file_location(module_name, path)
    if spec is None or spec.loader is None:
        raise ImportError(f"Could not import {module_name} from {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    spec.loader.exec_module(module)
    return module


def _load_oft_runtime(args: argparse.Namespace) -> Any:
    os.environ.setdefault("A0509_ACTION_CHUNK_SIZE", "5")
    if str(OFT_REPO) not in sys.path:
        sys.path.insert(0, str(OFT_REPO))
    module = _import_file("a0509_oft_server_runtime", OFT_REPO / "vla-scripts" / "serve_a0509_oft.py")
    checkpoint = args.checkpoint or ROOT / "runtime_state" / "oft_mixed480_step28560_merged"
    variant = module.get_a0509_oft_variant(args.variant)
    return module.OFTRuntime(
        checkpoint,
        variant,
        center_crop=True,
        num_diffusion_steps_inference=50,
        seed=7,
        load_in_8bit=False,
        load_in_4bit=False,
    )


def _load_vanilla_runtime(args: argparse.Namespace) -> Any:
    if str(VANILLA_SUPPORT) not in sys.path:
        sys.path.insert(0, str(VANILLA_SUPPORT))
    module = _import_file("a0509_vanilla_server_runtime", VANILLA_SUPPORT / "serve_adapter.py")
    bundle = args.checkpoint or ROOT / "models" / "vanilla_s1_balanced_step8130"
    return module.AdapterRuntime(
        bundle=bundle,
        model_id=str(args.base_model),
        local_files_only=True if args.local_files_only else True,
    )


def _resolve_modules(model_root: Any, names: list[str]) -> dict[str, Any]:
    modules = {}
    module_map = dict(model_root.named_modules())
    for name in names:
        if name not in module_map:
            matches = [key for key in module_map if key.endswith(name)]
            if len(matches) == 1:
                modules[name] = module_map[matches[0]]
                continue
            raise KeyError(f"Module {name!r} not found; matches={matches[:20]}")
        modules[name] = module_map[name]
    return modules


def _patch_action_head(features: dict[str, Any], runtime: Any):
    action_head = getattr(runtime, "action_head", None)
    if action_head is None or not hasattr(action_head, "predict_action"):
        return None
    original = action_head.predict_action

    def wrapped(self, actions_hidden_states):
        features["action_hidden_states.input"] = actions_hidden_states
        output = original(actions_hidden_states)
        features["action_head.output"] = output
        return output

    action_head.predict_action = MethodType(wrapped, action_head)
    return original


def _restore_action_head(runtime: Any, original: Any) -> None:
    action_head = getattr(runtime, "action_head", None)
    if action_head is not None and original is not None:
        action_head.predict_action = original


def _predict(runtime: Any, model: str, image, instruction: str) -> dict[str, Any]:
    if model == "oft":
        return runtime.predict(image, instruction)
    return runtime.predict(image, instruction)


def main() -> None:
    args = parse_args()
    output_dir = args.output_dir.expanduser().resolve()
    feature_dir = output_dir / "features"
    if args.dataset_root is not None:
        records = records_from_dataset(
            args.dataset_root,
            domain=args.domain,
            split=args.split,
            limit=args.max_samples,
            first_per_episode=args.first_per_episode,
        )
        source = {"dataset_root": str(args.dataset_root.expanduser().resolve())}
    else:
        records = records_from_images(
            args.image_input,
            domain=args.domain,
            label=args.label,
            instruction=args.instruction or "",
            limit=args.max_samples,
        )
        source = {
            "image_inputs": [
                str(path.expanduser().resolve()) for path in args.image_input
            ]
        }
    runtime = _load_oft_runtime(args) if args.model == "oft" else _load_vanilla_runtime(args)
    model_root = runtime.vla if args.model == "oft" else runtime.model

    default_modules = ["vision_backbone", "projector"]
    modules = _resolve_modules(model_root, [*default_modules, *args.extra_module])
    manifest = []

    for record in records:
        image = load_rgb(record.image_path)
        instruction = args.instruction or record.instruction or "pick up the cube"
        feature_path = feature_dir / f"{record.sample_id.replace(':', '_')}.npz"
        action_features: dict[str, Any] = {}
        original_predict_action = _patch_action_head(action_features, runtime)
        try:
            with ModuleFeatureCapture(modules) as capture:
                started = time.perf_counter()
                response = _predict(runtime, args.model, image, instruction)
                latency = time.perf_counter() - started
                features = {**capture.features, **action_features}
        finally:
            _restore_action_head(runtime, original_predict_action)

        metadata = {
            "sample_id": record.sample_id,
            "domain": record.domain,
            "model": args.model,
            "instruction": instruction,
            "source_image": str(record.image_path),
            "response": response,
            "latency_seconds": latency,
        }
        summary = save_feature_npz(
            feature_path,
            features,
            full=args.full,
            metadata=metadata,
        )
        manifest.append(
            {
                **metadata,
                "feature_file": str(feature_path.relative_to(output_dir)),
                "features": summary,
            }
        )

    write_json(
        output_dir / "feature_manifest.json",
        {
            "model": args.model,
            "domain": args.domain,
            **source,
            "checkpoint": str(args.checkpoint) if args.checkpoint else None,
            "full_features": bool(args.full),
            "sample_count": len(manifest),
            "first_per_episode": bool(args.first_per_episode),
            "records": manifest,
        },
    )
    print(f"Wrote {len(manifest)} feature records to {output_dir / 'feature_manifest.json'}")


if __name__ == "__main__":
    main()
