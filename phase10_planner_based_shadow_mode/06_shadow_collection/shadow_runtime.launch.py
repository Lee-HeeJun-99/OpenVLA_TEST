"""Non-executing Phase 10 launch draft.

It validates the offline-preparation safety file and intentionally launches no
node. A subscriber-only recorder Node may be added only after local review.
"""
from pathlib import Path
import sys

from launch import LaunchDescription

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))
from real_runtime_adapter import load_config  # noqa: E402


def generate_launch_description():
    safety, _ = load_config(ROOT / "shadow_runtime.yaml")
    safety.validate()
    return LaunchDescription([])
