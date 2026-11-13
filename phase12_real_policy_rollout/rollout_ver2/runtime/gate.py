from __future__ import annotations
from dataclasses import dataclass
from pathlib import Path
import json

EVIDENCE_TYPES={"photo","operator_attestation","measurement"}

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
                         "tracking_thresholds_calibrated", "feedback_rate_verified",
                         "arm_stationary_verified"),
    "synthetic": ("stop_servoj_verified", "pendant_limits_recorded",
                  "live_topics_verified", "servoj_time_param_verified",
                  "tracking_thresholds_calibrated", "feedback_rate_verified"),
    "policy": ("stop_servoj_verified", "pendant_limits_recorded",
               "live_topics_verified", "servoj_time_param_verified",
               "tracking_thresholds_calibrated", "feedback_rate_verified",
               "arm_stationary_verified"),
    "policy_movej": ("pendant_limits_recorded", "live_topics_verified",
                     "feedback_rate_verified", "stop_movej_verified"),
}


@dataclass(frozen=True)
class GateEntry:
    passed: bool = False
    evidence: str | None = None
    evidence_type: str | None = None


class ExecutionGate:
    def __init__(self, entries=None):
        self.entries = {name: GateEntry() for name in GATE_ITEMS}
        self.entries.update(entries or {})

    def _entry_failures(self,name):
        entry=self.entries[name];failures=[]
        if not entry.passed:return [f"{name}:not_passed"]
        if not entry.evidence or not Path(entry.evidence).is_file():return [f"{name}:evidence_missing"]
        if entry.evidence_type not in EVIDENCE_TYPES:return [f"{name}:evidence_type_invalid"]
        if entry.evidence_type=="operator_attestation":
            try:
                payload=json.loads(Path(entry.evidence).read_text())
                required=("items","verification_method","date","operator","photo_omission_reason")
                if not all(payload.get(key) for key in required):failures.append(f"{name}:attestation_incomplete")
                elif payload["verification_method"]!="펜던트 화면 직접 확인":
                    failures.append(f"{name}:attestation_method_invalid")
            except (OSError,ValueError,TypeError):failures.append(f"{name}:attestation_invalid_json")
        return failures

    def audit(self, mode):
        if mode not in MODE_REQUIREMENTS:raise ValueError("unknown execution mode")
        return [failure for name in MODE_REQUIREMENTS[mode]
                for failure in self._entry_failures(name)]

    def require(self, mode):
        failures = self.audit(mode)
        if failures:
            raise PermissionError("V2_GATE_BLOCKED:" + ",".join(failures))

    def require_item(self,name):
        if name not in self.entries:raise ValueError("unknown gate item")
        failures=self._entry_failures(name)
        if failures:raise PermissionError("V2_GATE_BLOCKED:"+",".join(failures))
