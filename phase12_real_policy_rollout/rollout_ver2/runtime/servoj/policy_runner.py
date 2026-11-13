"""ROS-independent 100 Hz ServoJ policy segment execution.

The complete segment is supplied by the planner before this class can touch the
publisher.  After the trajectory it keeps sending the exact final target while
checking settlement.  Any fault permanently inhibits publication before the
stop callback is invoked.
"""
from __future__ import annotations
from dataclasses import dataclass
import threading,time
import numpy as np
from controller.precheck import joint_margin_reasons
from runtime import robot_state as rs
from runtime.policy_movej_core import SettlementTracker
from .tracking import tracking_threshold_deg


@dataclass(frozen=True)
class PolicyStreamResult:
    sent_count:int
    send_jitter_ms:dict
    missed_deadline_count:int
    max_lateness_ms:float
    tracking_err_deg_max:list
    tracking_threshold_deg:list
    trajectory_end_monotonic:float|None
    settled_monotonic:float|None
    fault:str|None
    stop_result:object|None


class PolicyServoJRunner:
    """Dedicated sender used only in control/safety process A."""
    def __init__(self,publisher,feedback,alarm,controller_config,runtime_config,
                 *,clock=time.monotonic,sleep=time.sleep):
        self.publisher=publisher;self.feedback=feedback;self.alarm=alarm
        self.cc=controller_config;self.rc=runtime_config;self.clock=clock;self.sleep=sleep
        self.abort=threading.Event();self.permanently_inhibited=False

    def inhibit(self):
        self.abort.set();self.permanently_inhibited=True;self.publisher.inhibit()

    def run(self,segment,stop_callback=None,operator_abort=None,episode_deadline=None):
        rate=float(self.rc.get("rate_hz",100.));period=1./rate
        time_param=float(self.rc["servoj_time"])
        max_velocity=np.max(np.abs(np.rad2deg(segment.velocity_rad_s)),axis=0)
        thresholds=np.asarray(tracking_threshold_deg(time_param,max_velocity),float)
        pc=self.rc["policy_movej"]
        tracker=SettlementTracker(pc["settle_velocity_rad_s_candidate"],
          pc["settle_hold_s_candidate"],pc["target_error_deg_candidate"])
        start=self.clock();index=0;sent=0;missed=0;max_late=0.;jitters=[]
        maximum_error=np.zeros(6);trajectory_end=None;settled=None;fault=None;stop_result=None
        target=np.asarray(segment.joints_rad[-1],float)
        motion_timeout=float(pc["motion_timeout_s_candidate"])
        try:
            while True:
                deadline=start+index*period;now=self.clock();late=max(0.,now-deadline)
                max_late=max(max_late,late*1000.)
                if late>=period:
                    skip=max(1,int(np.ceil(late/period)));missed+=skip;index+=skip;continue
                while self.clock()<deadline and not self.abort.is_set():
                    self.sleep(max(0.,min(.001,deadline-self.clock())))
                now=self.clock();late=max(0.,now-deadline);max_late=max(max_late,late*1000.)
                if late>=period:
                    skip=max(1,int(np.ceil(late/period)));missed+=skip;index+=skip;continue
                if self.abort.is_set() or (operator_abort is not None and operator_abort.is_set()):
                    raise RuntimeError("OPERATOR_ABORT")
                if episode_deadline is not None and now>=float(episode_deadline):
                    raise RuntimeError("EPISODE_TIME_LIMIT")
                trajectory_index=min(index,len(segment.joints_rad)-1)
                command=np.asarray(segment.joints_rad[trajectory_index],float)
                if index>=len(segment.joints_rad)-1 and trajectory_end is None:trajectory_end=now
                snap=self.feedback.snapshot(now);errors=np.abs(np.rad2deg(snap.joint_rad-command))
                maximum_error=np.maximum(maximum_error,errors)
                faults=list(self.feedback.stale_reasons(now))
                faults.extend(f"TRACKING_ERROR:J{j+1}:{errors[j]:.6f}>{thresholds[j]:.6f}"
                  for j in np.flatnonzero(errors>thresholds))
                if snap.robot_state not in rs.ALLOWED_RUNNING_STATES:
                    faults.append(f"ROBOT_STATE_NOT_RUNNING:{snap.robot_state}")
                faults.extend(joint_margin_reasons(snap.joint_rad,self.cc))
                if self.alarm.active():faults.append("ROBOT_ALARM_ACTIVE")
                if faults:raise RuntimeError(",".join(dict.fromkeys(faults)))
                self.publisher.publish(command,now);sent+=1;jitters.append((now-deadline)*1000.)
                if index>=len(segment.joints_rad)-1:
                    if tracker.update(snap.joint_rad,snap.joint_velocity_rad_s,target,now):
                        settled=now;break
                    if trajectory_end is not None and now-trajectory_end>motion_timeout:
                        raise RuntimeError("SETTLEMENT_TIMEOUT")
                index+=1
        except Exception as error:
            fault=f"{type(error).__name__}:{error}";self.inhibit()
            if stop_callback is not None:stop_result=stop_callback(fault)
        values=np.asarray(jitters or [0.])
        return PolicyStreamResult(sent,{"p50":float(np.percentile(values,50)),
          "p99":float(np.percentile(values,99)),"max":float(np.max(values))},missed,max_late,
          maximum_error.tolist(),thresholds.tolist(),trajectory_end,settled,fault,stop_result)

    def start(self,*args,**kwargs):
        holder={}
        thread=threading.Thread(target=lambda:holder.setdefault("result",self.run(*args,**kwargs)),
          name="policy-servoj-100hz-control",daemon=True)
        thread.start();return thread,holder


class ModeAObservationLatch:
    """Small testable contract: one observation response yields at most one action."""
    def __init__(self):self.waiting=False;self.actions=0
    def request(self):
        if self.waiting:raise RuntimeError("OBSERVATION_ALREADY_PENDING")
        self.waiting=True
    def accept(self,message):
        if not self.waiting:raise RuntimeError("UNSOLICITED_OBSERVATION_RESULT")
        self.waiting=False
        if message.get("type")=="RESULT" and message.get("ok"):
            self.actions+=1;return message["action_raw"]
        return None
