#!/usr/bin/env python3
"""Recorded-input OpenVLA adapter with no robot publishing capability."""

from __future__ import annotations

import base64
import json
from pathlib import Path
import time
from typing import Any, Callable
from urllib.request import Request, urlopen

from action_canonicalizer import canonicalize_action


class OpenVLAAdapter:
    expected_variant = "openvla_token"
    horizon = 1

    def __init__(
        self,
        server_url: str | None = None,
        *,
        timeout_seconds: float = 30.0,
        predictor: Callable[[dict[str, Any]], dict[str, Any]] | None = None,
    ) -> None:
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
        payload = {
            "instruction": instruction,
            "image_jpeg_base64": base64.b64encode(image_path.read_bytes()).decode("ascii"),
            "variant": self.expected_variant,
        }
        response = self._request(payload)
        latency = time.perf_counter() - started
        variant = response.get("variant", self.expected_variant)
        if variant != self.expected_variant:
            raise ValueError(f"expected {self.expected_variant}, got {variant}")
        raw = response.get("action")
        if raw is None:
            actions = response.get("actions")
            raw = actions[0] if isinstance(actions, list) and actions else None
        if raw is None:
            raise ValueError("OpenVLA response has no action")
        canonical = canonicalize_action(
            raw,
            source="openvla_denormalized",
            time_horizon_seconds=0.2,
        )
        return {
            "model": "openvla",
            "variant": variant,
            "raw_model_output": response,
            "denormalized_action": list(raw),
            "canonical_action": canonical.to_dict(),
            "inference_latency_seconds": latency,
            "executed_action": None,
            "execution_status": "NOT_EXECUTED",
        }

