from __future__ import annotations
from dataclasses import dataclass
import json
from pathlib import Path
import numpy as np

PATH=Path(__file__).with_name("pendant_limits.json")
DEFAULT_LO=np.array([-360,-95,-135,-360,-135,-360],dtype=float)
DEFAULT_HI=-DEFAULT_LO

@dataclass(frozen=True)
class JointLimitConfig:
    profile: str
    source_date: str
    evidence_path: str
    extension_reason: str | None
    clearance_record: dict | None
    pendant_lower_deg: np.ndarray
    pendant_upper_deg: np.ndarray
    maximum_abs_deg: np.ndarray
    tolerance_deg: np.ndarray
    host_margin_deg: np.ndarray

    @property
    def host_lower_deg(self): return self.pendant_lower_deg+self.host_margin_deg
    @property
    def host_upper_deg(self): return self.pendant_upper_deg-self.host_margin_deg
    @property
    def protective_expected_lower_deg(self): return self.pendant_lower_deg-self.tolerance_deg
    @property
    def protective_expected_upper_deg(self): return self.pendant_upper_deg+self.tolerance_deg
    @property
    def is_default(self):
        return np.array_equal(self.pendant_lower_deg,DEFAULT_LO) and np.array_equal(self.pendant_upper_deg,DEFAULT_HI)

    def require_extension_gates(self,gate):
        if self.is_default:return
        if not self.extension_reason:raise PermissionError("EXTENDED_LIMIT_REASON_REQUIRED")
        record=self.clearance_record
        if not isinstance(record,dict) or not all(record.get(k) for k in
                ("date","operator","joints_checked","angles_checked_deg")):
            raise PermissionError("EXTENDED_RANGE_CLEARANCE_RECORD_INCOMPLETE")
        failures=[]
        for name in ("pendant_limits_recorded","extended_range_clearance_checked"):
            entry=gate.entries[name]
            if not entry.passed:failures.append(name+":not_passed")
            elif not entry.evidence or not Path(entry.evidence).is_file():failures.append(name+":evidence_missing")
        if failures:raise PermissionError("EXTENDED_RANGE_GATE_BLOCKED:"+",".join(failures))

def load_joint_limits(path=PATH):
    raw=json.loads(Path(path).read_text())
    def six(name):
        a=np.asarray(raw[name],dtype=float)
        if a.shape!=(6,) or not np.all(np.isfinite(a)):raise ValueError(name+" must contain six finite values")
        return a
    lo,hi,maximum=six("pendant_lower_deg"),six("pendant_upper_deg"),six("doosan_configurable_max_abs_deg")
    tolerance,margin=six("controller_tolerance_deg"),six("host_margin_deg_candidate")
    if np.any(lo>=hi):raise ValueError("PENDANT_LIMIT_ORDER_INVALID")
    if np.any(np.abs(lo)>maximum) or np.any(np.abs(hi)>maximum):raise ValueError("PENDANT_EXCEEDS_DOOSAN_CONFIGURABLE_MAX")
    if np.any(margin<=0) or np.any(lo+margin>=hi-margin):raise ValueError("HOST_STOP_NOT_STRICTLY_INSIDE_PENDANT")
    evidence=str(raw.get("evidence_path") or "").strip();date=str(raw.get("source_date") or "").strip()
    if not evidence or not date:raise ValueError("PENDANT_SOURCE_EVIDENCE_AND_DATE_REQUIRED")
    return JointLimitConfig(str(raw.get("profile","")),date,evidence,raw.get("extension_reason"),
                            raw.get("extended_range_clearance_record"),
                            lo,hi,maximum,tolerance,margin)
