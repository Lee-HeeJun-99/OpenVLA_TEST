#!/usr/bin/env python3
"""Append-only integrated JSONL logger with atomic close metadata."""

from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
from typing import Any


REQUIRED_IDENTIFIERS = ("trial_id", "condition_id", "layout_id", "episode_id", "frame_id")


class IntegratedLogger:
    def __init__(self, output_dir: Path) -> None:
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=False)
        self.data_path = self.output_dir / "samples.jsonl"
        self.meta_path = self.output_dir / "logger_status.json"
        self._stream = self.data_path.open("x", encoding="utf-8")
        self.count = 0
        self.closed = False

    @staticmethod
    def image_sha256(path: Path) -> str:
        return hashlib.sha256(Path(path).read_bytes()).hexdigest()

    def append(self, record: dict[str, Any]) -> None:
        if self.closed:
            raise RuntimeError("logger is closed")
        missing = [key for key in REQUIRED_IDENTIFIERS if record.get(key) in (None, "")]
        if missing:
            raise ValueError(f"missing required identifiers: {missing}")
        # Shadow predictions must never masquerade as executed commands.
        for model in ("openvla", "oft"):
            executed = record.get(f"{model}_executed_action")
            if executed not in (None, "NOT_EXECUTED"):
                raise ValueError(f"{model}_executed_action must be null/NOT_EXECUTED")
        self._stream.write(json.dumps(record, ensure_ascii=False, separators=(",", ":")) + "\n")
        self._stream.flush()
        os.fsync(self._stream.fileno())
        self.count += 1

    def close(self, *, status: str = "complete") -> None:
        if self.closed:
            return
        self._stream.close()
        self.closed = True
        self.meta_path.write_text(
            json.dumps({"status": status, "record_count": self.count, "data_file": self.data_path.name}, indent=2) + "\n",
            encoding="utf-8",
        )

    def __enter__(self) -> "IntegratedLogger":
        return self

    def __exit__(self, exc_type, exc, traceback) -> None:
        self.close(status="failed" if exc_type else "complete")

