"""A0509 portable-dataset adapter for OpenVLA-OFT training."""

from __future__ import annotations

import json
import random
from collections import defaultdict
from pathlib import Path
from typing import Any, Iterator

import numpy as np
import torch
from PIL import Image, ImageEnhance
from torch.utils.data import IterableDataset

from prismatic.vla.constants import IGNORE_INDEX, NUM_ACTIONS_CHUNK


DATASET_KEY = "a0509_sim_cube_pick"


def _read_json(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def _read_jsonl(path: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    with path.open("r", encoding="utf-8") as handle:
        for line in handle:
            if line.strip():
                rows.append(json.loads(line))
    return rows


def _normalize_actions(
    actions: np.ndarray,
    dataset_statistics: dict[str, Any],
) -> np.ndarray:
    stats = dataset_statistics[DATASET_KEY]["action"]
    low = np.asarray(stats["q01"], dtype=np.float32)
    high = np.asarray(stats["q99"], dtype=np.float32)
    minimum = np.asarray(stats["min"], dtype=np.float32)
    maximum = np.asarray(stats["max"], dtype=np.float32)
    mask = np.asarray(stats.get("mask", np.ones_like(low)), dtype=bool)
    normalized = np.where(
        mask,
        np.clip(2.0 * (actions - low) / (high - low + 1e-8) - 1.0, -1.0, 1.0),
        actions,
    )
    normalized = np.where(minimum == maximum, 0.0, normalized)
    return normalized.astype(np.float32)


class _ImageAugment:
    def __call__(self, image: Image.Image) -> Image.Image:
        width, height = image.size
        scale = random.uniform(0.9, 1.0)
        crop_width = max(1, round(width * scale))
        crop_height = max(1, round(height * scale))
        left = random.randint(0, width - crop_width)
        top = random.randint(0, height - crop_height)
        image = image.crop((left, top, left + crop_width, top + crop_height))
        image = image.resize((width, height), Image.Resampling.BICUBIC)
        image = ImageEnhance.Brightness(image).enhance(random.uniform(0.8, 1.2))
        image = ImageEnhance.Contrast(image).enhance(random.uniform(0.8, 1.2))
        image = ImageEnhance.Color(image).enhance(random.uniform(0.8, 1.2))
        return image


class A0509OFTDataset(IterableDataset):
    """Produces fixed-size future action chunks from the existing episode split."""

    def __init__(
        self,
        dataset_root: Path,
        split: str,
        action_tokenizer: Any,
        base_tokenizer: Any,
        image_transform: Any,
        prompt_builder_fn: Any,
        image_augmentation: bool,
        repeat: bool,
        seed: int = 7,
    ) -> None:
        super().__init__()
        self.dataset_root = Path(dataset_root).resolve()
        self.split = split
        self.action_tokenizer = action_tokenizer
        self.base_tokenizer = base_tokenizer
        self.image_transform = image_transform
        self.prompt_builder_fn = prompt_builder_fn
        self.augment = _ImageAugment() if image_augmentation else None
        self.repeat = bool(repeat)
        self.seed = int(seed)
        self.dataset_statistics = _read_json(
            self.dataset_root / "metadata" / "dataset_statistics.json"
        )

        raw_rows = _read_jsonl(self.dataset_root / "metadata" / f"{split}.jsonl")
        grouped: dict[str, list[dict[str, Any]]] = defaultdict(list)
        for row in raw_rows:
            grouped[str(row["episode_id"])].append(row)

        self.samples: list[tuple[dict[str, Any], np.ndarray]] = []
        for episode_id in sorted(grouped):
            episode_rows = sorted(
                grouped[episode_id],
                key=lambda value: int(value["step_index"]),
            )
            raw_actions = np.asarray(
                [row["action"] for row in episode_rows],
                dtype=np.float32,
            )
            terminal_hold = np.zeros((7,), dtype=np.float32)
            terminal_hold[-1] = float(raw_actions[-1, -1])
            for index, row in enumerate(episode_rows):
                future = raw_actions[index : index + NUM_ACTIONS_CHUNK]
                if len(future) < NUM_ACTIONS_CHUNK:
                    padding = np.repeat(
                        terminal_hold[None, :],
                        NUM_ACTIONS_CHUNK - len(future),
                        axis=0,
                    )
                    future = np.concatenate([future, padding], axis=0)
                self.samples.append(
                    (
                        row,
                        _normalize_actions(future, self.dataset_statistics),
                    )
                )

        if not self.samples:
            raise ValueError(f"A0509 split {split!r} is empty")

    def __len__(self) -> int:
        return len(self.samples)

    def _transform(
        self,
        row: dict[str, Any],
        actions: np.ndarray,
    ) -> dict[str, Any]:
        image_path = (self.dataset_root / row["image"]).resolve()
        if not image_path.is_relative_to(self.dataset_root):
            raise ValueError(f"Image escapes dataset root: {row['image']}")
        with Image.open(image_path) as source:
            image = source.convert("RGB")
        if self.augment is not None:
            image = self.augment(image)

        language = " ".join(str(row["instruction"]).strip().lower().split()).rstrip(
            " .?!"
        )
        action_strings = self.action_tokenizer(actions)
        if isinstance(action_strings, str):
            action_chunk_string = action_strings
        else:
            action_chunk_string = "".join(action_strings)

        prompt_builder = self.prompt_builder_fn("openvla")
        prompt_builder.add_turn(
            "human",
            f"What action should the robot take to {language}?",
        )
        prompt_builder.add_turn("gpt", action_chunk_string)
        input_ids = self.base_tokenizer(
            prompt_builder.get_prompt(),
            add_special_tokens=True,
        ).input_ids
        labels = list(input_ids)
        input_ids_tensor = torch.tensor(input_ids, dtype=torch.long)
        labels_tensor = torch.tensor(labels, dtype=torch.long)
        labels_tensor[: -(actions.size + 1)] = IGNORE_INDEX
        return {
            "pixel_values": self.image_transform(image),
            "input_ids": input_ids_tensor,
            "labels": labels_tensor,
            "dataset_name": DATASET_KEY,
            "actions": actions,
        }

    def __iter__(self) -> Iterator[dict[str, Any]]:
        worker = torch.utils.data.get_worker_info()
        worker_id = 0 if worker is None else worker.id
        rng = random.Random(self.seed + worker_id)
        indices = list(range(len(self.samples)))
        while True:
            if self.repeat:
                rng.shuffle(indices)
            for index in indices:
                row, actions = self.samples[index]
                yield self._transform(row, actions)
            if not self.repeat:
                return
