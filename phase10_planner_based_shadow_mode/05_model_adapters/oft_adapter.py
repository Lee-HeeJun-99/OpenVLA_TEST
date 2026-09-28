#!/usr/bin/env python3
"""Recorded-input OFT K=5 adapter with no robot publishing capability."""

from __future__ import annotations

import base64
import json
from pathlib import Path
import time
from typing import Any, Callable
from urllib.request import Request, urlopen

from action_canonicalizer import canonicalize_chunk, integrated_displacement


class OFTAdapter:
    expected_variant = "oftplus_h5_vision"
    horizon = 5

    def __init__(self, server_url: str | None = None, *, timeout_seconds: float = 30.0,
                 predictor: Callable[[dict[str, Any]], dict[str, Any]] | None = None) -> None:
        if server_url is None and predictor is None:
            raise ValueError("server_url or an offline predictor is required")
        self.server_url = None if server_url is None else server_url.rstrip("/")
        self.timeout_seconds = float(timeout_seconds)
        self.predictor = predictor

    def _request(self, payload: dict[str, Any]) -> dict[str, Any]:
        if self.predictor is not None:
            return self.predictor(payload)
        assert self.server_url is not None
        request = Request(
            f"{self.server_url}/predict",
            data=json.dumps(payload, separators=(",", ":")).encode("utf-8"),
            method="POST",
            headers={"Content-Type": "application/json"},
        )
        with urlopen(request, timeout=self.timeout_seconds) as response:
            return json.loads(response.read())

    def predict_recorded(self, image_path: Path, instruction: str) -> dict[str, Any]:
        started = time.perf_counter()
        response = self._request({
            "instruction": instruction,
            "image_jpeg_base64": base64.b64encode(image_path.read_bytes()).decode("ascii"),
            "variant": self.expected_variant,
        })
        latency = time.perf_counter() - started
        variant = response.get("variant", self.expected_variant)
        if variant != self.expected_variant:
            raise ValueError(f"expected {self.expected_variant}, got {variant}")
        actions = response.get("actions")
        if not isinstance(actions, list) or len(actions) != self.horizon:
            raise ValueError(f"OFT requires exactly K=5 actions, got {actions!r}")
        canonical = canonicalize_chunk(
            actions,
            source="oft_denormalized",
            time_horizon_seconds=0.2,
        )
        return {
            "model": "oft",
            "variant": variant,
            "raw_model_output": response,
            "denormalized_action_chunk": actions,
            "canonical_action_chunk": [item.to_dict() for item in canonical],
            "integrated_displacement": integrated_displacement(canonical),
            "inference_latency_seconds": latency,
            "executed_action": None,
            "execution_status": "NOT_EXECUTED",
        }

