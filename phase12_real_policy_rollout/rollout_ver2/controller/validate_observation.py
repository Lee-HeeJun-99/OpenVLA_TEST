#!/usr/bin/python3
from __future__ import annotations
import hashlib
import io
import json
from pathlib import Path
import sys
import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from controller.observation import preprocess_observation, processor_tensor

IMAGE = Path("/home/ubuntu/a0509_vla_linux_field_bundle_20260903/validation/preflight/input.jpg")


def main():
    rgb = np.asarray(Image.open(IMAGE).convert("RGB"), dtype=np.uint8)
    # Independent spelling of isaac_sim/a0509_control/vla_client.py.
    reference = io.BytesIO()
    Image.fromarray(rgb, mode="RGB").save(
        reference, format="JPEG", quality=92, optimize=False)
    jpeg_a = reference.getvalue()
    with Image.open(io.BytesIO(jpeg_a)) as source:
        tensor_a = processor_tensor(source.convert("RGB"))
    jpeg_b, tensor_b = preprocess_observation(rgb.copy())
    result = {
        "input": str(IMAGE), "rgb_shape": list(rgb.shape),
        "jpeg_identical": jpeg_a == jpeg_b,
        "jpeg_sha256": hashlib.sha256(jpeg_a).hexdigest(),
        "tensor_identical": bool((tensor_a == tensor_b).all()),
        "tensor_shape": list(tensor_a.shape),
        "tensor_dtype": str(tensor_a.dtype),
        "tensor_max_abs_difference": float((tensor_a-tensor_b).abs().max()),
        "contract": "RGB uint8 -> JPEG q92 optimize=False -> RGB decode -> resize-naive 224 dual fused tensor",
    }
    path = ROOT/"reports/observation_preprocessing_validation.json"
    path.write_text(json.dumps(result, indent=2)+"\n")
    print(path)


if __name__ == "__main__":
    main()
