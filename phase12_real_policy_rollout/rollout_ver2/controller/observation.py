"""Offline observation contract matching the matched15 vanilla sim client."""

from __future__ import annotations

import io
import json
from pathlib import Path
import numpy as np
import torch
from PIL import Image
from torchvision.transforms import functional as TVF
from torchvision.transforms import InterpolationMode

PROCESSOR_CONFIG = Path(
    "/home/ubuntu/a0509_vla_linux_field_bundle_20260903/models/"
    "vanilla_s1_balanced_step8130/processor/preprocessor_config.json"
)


def encode_sim_evaluation_jpeg(rgb: np.ndarray) -> bytes:
    image = np.asarray(rgb)
    if image.ndim != 3 or image.shape[2] != 3 or image.dtype != np.uint8:
        raise ValueError("expected HxWx3 uint8 RGB")
    buffer = io.BytesIO()
    Image.fromarray(image, mode="RGB").save(
        buffer, format="JPEG", quality=92, optimize=False)
    return buffer.getvalue()


def decode_server_rgb(jpeg: bytes) -> Image.Image:
    with Image.open(io.BytesIO(jpeg)) as source:
        return source.convert("RGB")


def processor_tensor(image: Image.Image, config_path=PROCESSOR_CONFIG) -> torch.Tensor:
    """Exact saved Prismatic resize-naive image transform, without text tokens."""
    config = json.loads(Path(config_path).read_text())
    if config["image_resize_strategy"] != "resize-naive":
        raise ValueError("processor is not resize-naive")
    tensors = []
    for resize, crop, normalize in zip(
            config["tvf_resize_params"], config["tvf_crop_params"],
            config["tvf_normalize_params"]):
        transformed = TVF.resize(
            image, size=resize["size"], interpolation=InterpolationMode.BICUBIC,
            antialias=bool(resize["antialias"]))
        transformed = TVF.center_crop(transformed, crop["output_size"])
        tensor = TVF.to_tensor(transformed)
        tensors.append(TVF.normalize(tensor, mean=normalize["mean"],
                                     std=normalize["std"], inplace=False))
    return torch.vstack(tensors)


def preprocess_observation(rgb: np.ndarray) -> tuple[bytes, torch.Tensor]:
    jpeg = encode_sim_evaluation_jpeg(rgb)
    return jpeg, processor_tensor(decode_server_rgb(jpeg))
