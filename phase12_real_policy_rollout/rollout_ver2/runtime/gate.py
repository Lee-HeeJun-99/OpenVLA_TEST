from __future__ import annotations
from dataclasses import dataclass
from pathlib import Path

GATE_ITEMS = (
    "pendant_limits_recorded", "live_topics_verified",
    "servoj_time_param_verified", "tracking_thresholds_calibrated",
    "feedback_rate_verified", "gripper_wiring_recorded", "gripper_io_verified",
    "arm_stationary_verified", "stop_movej_verified", "stop_servoj_verified",
    "extended_range_clearance_checked")

MODE_REQUIREMENTS = {
    "dryrun_offline": (),
    "dryrun_live": (),
    "gripper_test": ("pendant_limits_recorded", "live_topics_verified",
                     "gripper_wiring_recorded", "arm_stationary_verified"),
    "stop_test_movej": ("pendant_limits_recorded", "live_topics_verified",
                        "feedback_rate_verified"),
    "stop_test_servoj": ("stop_movej_verified", "pendant_limits_recorded",
                         "live_topics_verified", "servoj_time_param_verified",
                         "tracking_thresholds_calibrated", "feedback_rate_verified"),
    "synthetic": ("stop_servoj_verified", "pendant_limits_recorded",
                  "live_topics_verified", "servoj_time_param_verified",
                  "tracking_thresholds_calibrated", "feedback_rate_verified"),
    "policy": ("stop_servoj_verified", "pendant_limits_recorded",
               "live_topics_verified", "servoj_time_param_verified",
               "tracking_thresholds_calibrated", "feedback_rate_verified"),
}


@dataclass(frozen=True)
class GateEntry:
    passed: bool = False
    evidence: str | None = None


class ExecutionGate:
    def __init__(self, entries=None):
        self.entries = {name: GateEntry() for name in GATE_ITEMS}
        self.entries.update(entries or {})

    def audit(self, mode):
        if mode not in MODE_REQUIREMENTS:
            raise ValueError("unknown execution mode")
        failures = []
        for name in MODE_REQUIREMENTS[mode]:
            entry = self.entries[name]
            if not entry.passed:
                failures.append(f"{name}:not_passed")
            elif not entry.evidence or not Path(entry.evidence).is_file():
                failures.append(f"{name}:evidence_missing")
        return failures

    def require(self, mode):
        failures = self.audit(mode)
        if failures:
            raise PermissionError("V2_GATE_BLOCKED:" + ",".join(failures))
