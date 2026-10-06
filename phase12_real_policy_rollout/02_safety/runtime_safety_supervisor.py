"""Command-free runtime safety supervisor for offline and Shadow validation."""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence


@dataclass(frozen=True)
class RuntimeSafetyResult:
    accepted: bool
    hold_required: bool
    hold_reason: str | None
    command_issued: bool = False


class RuntimeSafetySupervisor:
    """Validates timing and action dynamics without publishing or calling Hold."""

    def __init__(self, *, max_action_age_sec: float, min_command_period_sec: float,
                 max_translation_m: float, max_rotation_rad: float,
                 max_translation_velocity_m_s: float,
                 max_rotation_velocity_rad_s: float,
                 max_translation_acceleration_m_s2: float,
                 max_rotation_acceleration_rad_s2: float):
        limits = (max_action_age_sec, min_command_period_sec, max_translation_m,
                  max_rotation_rad, max_translation_velocity_m_s,
                  max_rotation_velocity_rad_s, max_translation_acceleration_m_s2,
                  max_rotation_acceleration_rad_s2)
        if not all(math.isfinite(float(x)) and float(x) > 0 for x in limits):
            raise ValueError("all safety limits must be positive finite values")
        self.max_action_age_sec=float(max_action_age_sec)
        self.min_command_period_sec=float(min_command_period_sec)
        self.max_translation_m=float(max_translation_m)
        self.max_rotation_rad=float(max_rotation_rad)
        self.max_translation_velocity_m_s=float(max_translation_velocity_m_s)
        self.max_rotation_velocity_rad_s=float(max_rotation_velocity_rad_s)
        self.max_translation_acceleration_m_s2=float(max_translation_acceleration_m_s2)
        self.max_rotation_acceleration_rad_s2=float(max_rotation_acceleration_rad_s2)
        self._last_id=None; self._last_time=None; self._last_velocity=None

    @staticmethod
    def _norm(values: Sequence[float]) -> float:
        return math.sqrt(sum(float(x) ** 2 for x in values))

    @staticmethod
    def _reject(reason: str) -> RuntimeSafetyResult:
        return RuntimeSafetyResult(False, True, reason, False)

    @staticmethod
    def _exceeds(value: float, limit: float) -> bool:
        # Numerical tolerance only: it prevents a vector clipped exactly to the
        # configured boundary from being rejected by floating-point roundoff.
        tolerance=max(1e-12,abs(float(limit))*1e-9)
        return float(value) > float(limit)+tolerance

    def inspect(self, *, action_id: str, action: Sequence[float],
                source_monotonic: float, now_monotonic: float,
                camera_ok: bool=True, state_ok: bool=True, tcp_ok: bool=True,
                inference_ok: bool=True, communication_ok: bool=True,
                logger_ok: bool=True) -> RuntimeSafetyResult:
        if not logger_ok:return self._reject("logger_failure")
        if not communication_ok:return self._reject("communication_failure")
        if not camera_ok:return self._reject("camera_failure")
        if not state_ok:return self._reject("joint_state_failure")
        if not tcp_ok:return self._reject("tcp_failure")
        if not inference_ok:return self._reject("inference_timeout")
        try: values=tuple(float(x) for x in action)
        except Exception:return self._reject("invalid_action")
        if len(values)!=7 or not all(math.isfinite(x) for x in values):return self._reject("invalid_action")
        now=float(now_monotonic);source=float(source_monotonic)
        age=now-source
        if not math.isfinite(age) or age < 0:return self._reject("clock_domain_or_time_regression")
        if age > self.max_action_age_sec:return self._reject("stale_action")
        if action_id == self._last_id:return self._reject("duplicate_action")
        if self._exceeds(self._norm(values[:3]),self.max_translation_m):return self._reject("translation_step_limit")
        if self._exceeds(self._norm(values[3:6]),self.max_rotation_rad):return self._reject("rotation_step_limit")
        if self._last_time is not None:
            dt=now-self._last_time
            if dt < self.min_command_period_sec:return self._reject("command_rate_limit")
            if dt <= 0:return self._reject("clock_domain_or_time_regression")
            velocity=tuple(x/dt for x in values[:6])
            if self._exceeds(self._norm(velocity[:3]),self.max_translation_velocity_m_s):return self._reject("translation_velocity_limit")
            if self._exceeds(self._norm(velocity[3:]),self.max_rotation_velocity_rad_s):return self._reject("rotation_velocity_limit")
            if self._last_velocity is not None:
                acceleration=tuple((v-p)/dt for v,p in zip(velocity,self._last_velocity))
                if self._exceeds(self._norm(acceleration[:3]),self.max_translation_acceleration_m_s2):return self._reject("translation_acceleration_limit")
                if self._exceeds(self._norm(acceleration[3:]),self.max_rotation_acceleration_rad_s2):return self._reject("rotation_acceleration_limit")
            self._last_velocity=velocity
        self._last_id=str(action_id);self._last_time=now
        return RuntimeSafetyResult(True,False,None,False)

    def watchdog(self, *, last_camera_monotonic: float | None,
                 last_state_monotonic: float | None,
                 last_tcp_monotonic: float | None, now_monotonic: float,
                 timeout_sec: float) -> RuntimeSafetyResult:
        now=float(now_monotonic)
        for name,value in (("camera",last_camera_monotonic),("joint_state",last_state_monotonic),("tcp",last_tcp_monotonic)):
            if value is None or now-float(value)>float(timeout_sec):
                return self._reject(f"{name}_watchdog_timeout")
        return RuntimeSafetyResult(True,False,None,False)
