"""Mockable fixed-joint ServoJ hold state machine."""
from __future__ import annotations
import json,time
from pathlib import Path
import numpy as np
from runtime import robot_state as rs

class HoldFault(RuntimeError):pass

class HoldRunner:
    def __init__(self,publisher,feedback,stop_transport,alarm,config,*,clock=time.monotonic,
                 sleep=time.sleep,abort_requested=lambda:False):
        self.publisher=publisher;self.feedback=feedback;self.stop=stop_transport;self.alarm=alarm
        self.config=config;self.clock=clock;self.sleep=sleep;self.abort_requested=abort_requested
        self.rows=[];self.jitters=[];self.sent=0;self.inhibited=False;self.stop_calls=[]
        self.deadlines={};self.stop_confirmation=None
    def inhibit_and_stop(self,reason):
        if not self.inhibited:
            self.inhibited=True;self.publisher.inhibit()
        if not self.stop_calls:
            requested=self.clock()
            try:ack=self.stop.move_stop(stop_mode=int(self.config["move_stop_mode_candidate_unverified"]))
            except Exception as error:ack={"success":False,"error":str(error)}
            self.stop_calls.append({"reason":reason,"requested_monotonic":requested,
              "completed_monotonic":self.clock(),"ack":ack})
    def _sample(self,stage,target,send_time=None):
        now=self.clock();snap=self.feedback.snapshot(now);delta=np.rad2deg(snap.joint_rad-target)
        speed=np.abs(snap.joint_velocity_rad_s);reasons=list(self.feedback.stale_reasons(now))
        if snap.robot_state not in rs.ALLOWED_RUNNING_STATES:reasons.append(f"ROBOT_STATE_NOT_RUNNING:{snap.robot_state}")
        if self.alarm.active():reasons.append("ROBOT_ALARM_ACTIVE")
        if np.max(np.abs(delta))>float(self.config["servoj_hold_max_delta_deg_abort"]):reasons.append("HOLD_DELTA_EXCEEDED")
        if np.max(speed)>float(self.config["servoj_hold_max_velocity_rad_s_abort"]):reasons.append("HOLD_VELOCITY_EXCEEDED")
        row={"t_monotonic":now,"stage":stage,"send_monotonic":send_time,
          "target_joint_deg_j1_to_j6":np.rad2deg(target).tolist(),"received_joint_rad":snap.joint_rad.tolist(),
          "received_velocity_rad_s":snap.joint_velocity_rad_s.tolist(),"robot_state":snap.robot_state,
          "joint_delta_deg":delta.tolist(),"alarm":self.alarm.record()}
        self.rows.append(row)
        if reasons:raise HoldFault(",".join(dict.fromkeys(reasons)))
    def _phase(self,label,duration,target,publish):
        rate=float(self.config["servoj_hold_rate_hz"]);count=int(round(float(duration)*rate))
        period=1.0/rate;start=self.clock();i=0;missed=0;max_late=0.
        stats={"missed_deadline_count":0,"max_lateness_ms":0.};self.deadlines[label]=stats
        while i<count:
            if self.abort_requested():raise HoldFault("OPERATOR_ABORT")
            deadline=start+i*period;now=self.clock();lateness=max(0.,now-deadline)
            max_late=max(max_late,lateness*1000);stats["max_lateness_ms"]=max_late
            if lateness>=period:
                skip=min(max(1,int(np.ceil(lateness/period))),count-i);missed+=skip;i+=skip
                stats["missed_deadline_count"]=missed;continue
            while self.clock()<deadline:
                self.sleep(max(0.0,min(.001,deadline-self.clock())))
            now=self.clock();late=max(0.,now-deadline);max_late=max(max_late,late*1000);stats["max_lateness_ms"]=max_late
            if late>=period:
                skip=min(max(1,int(np.ceil(late/period))),count-i);missed+=skip;i+=skip
                stats["missed_deadline_count"]=missed;continue
            send_time=None
            if publish:
                if self.inhibited:raise HoldFault("PUBLISH_PERMANENTLY_INHIBITED")
                self.publisher.publish(target,now);self.sent+=1;send_time=now;self.jitters.append((now-deadline)*1000)
            self._sample(label,target,send_time)
            i+=1
        stats.update(missed_deadline_count=missed,max_lateness_ms=max_late)
    def _observe_stop_confirmation(self,target):
        label="POST_MOVESTOP_OBSERVE";duration=float(self.config["servoj_hold_post_stop_observe_s"])
        try:self._phase(label,duration,target,False)
        except Exception as error:
            # No command can be sent here. Retain the observation error and
            # still derive an explicit failed confirmation from collected rows.
            self.deadlines.setdefault(label,{"missed_deadline_count":0,"max_lateness_ms":0.})
            self.deadlines[label]["observation_error"]=f"{type(error).__name__}:{error}"
        rows=[r for r in self.rows if r["stage"]==label];threshold=.001;hold=.3
        zero_since=None;confirmed_at=None
        for row in rows:
            speed=max(abs(x) for x in row["received_velocity_rad_s"])
            if speed<=threshold:
                zero_since=row["t_monotonic"] if zero_since is None else zero_since
                if row["t_monotonic"]-zero_since>=hold:confirmed_at=row["t_monotonic"];break
            else:zero_since=None
        self.stop_confirmation={"confirmed":confirmed_at is not None,"confirmed_monotonic":confirmed_at,
          "velocity_threshold_rad_s":threshold,"required_hold_s":hold,"sample_count":len(rows),
          "max_abs_velocity_rad_s":None if not rows else max(max(abs(x) for x in r["received_velocity_rad_s"]) for r in rows),
          "post_stop_travel_deg":None if not rows else max(
              max(abs(x) for x in r["joint_delta_deg"]) for r in rows)}
    def run(self,target):
        reason="COMPLETED";passed=False
        stop_observed=False
        try:
            self._phase("STREAM_HOLD",self.config["servoj_hold_stream_duration_s"],target,True)
            self.publisher.inhibit();self.inhibited=True
            self._phase("POST_STREAM_OBSERVE",self.config["servoj_hold_post_stream_observe_s"],target,False)
            self.inhibit_and_stop("NORMAL_HOLD_SEQUENCE")
            self._observe_stop_confirmation(target);stop_observed=True
            passed=bool(self.stop_confirmation["confirmed"])
        except Exception as error:
            reason=str(error);self.inhibit_and_stop(reason)
            if not stop_observed:self._observe_stop_confirmation(target)
        values=np.asarray(self.jitters or [0.]);by_stage={}
        for stage in ("STREAM_HOLD","POST_STREAM_OBSERVE","POST_MOVESTOP_OBSERVE"):
            rows=[r for r in self.rows if r["stage"]==stage]
            by_stage[stage]={"samples":len(rows),"max_joint_change_deg":0.0 if not rows else max(max(abs(x) for x in r["joint_delta_deg"]) for r in rows),
              "max_abs_velocity_rad_s":0.0 if not rows else max(max(abs(x) for x in r["received_velocity_rad_s"]) for r in rows)}
        all_delta=max((x["max_joint_change_deg"] for x in by_stage.values()),default=0.)
        all_speed=max((x["max_abs_velocity_rad_s"] for x in by_stage.values()),default=0.)
        missed=sum(x.get("missed_deadline_count",0) for x in self.deadlines.values())
        max_lateness=max((x.get("max_lateness_ms",0.) for x in self.deadlines.values()),default=0.)
        evidence_pass=(passed and all_delta<.01 and all_speed<.001 and
                       not self.alarm.ever_active and float(np.percentile(values,99))<2.)
        failure_detail=None
        if not evidence_pass:
            last=self.rows[-1] if self.rows else None
            failure_detail={"last_stage":None if last is None else last["stage"],
              "last_joint_deg":None if last is None else np.rad2deg(last["received_joint_rad"]).tolist(),
              "last_velocity_rad_s":None if last is None else last["received_velocity_rad_s"],
              "max_joint_change_deg":all_delta,"max_abs_velocity_rad_s":all_speed}
        return {"pass":evidence_pass,"sequence_completed":passed,"evidence_pass":evidence_pass,"reason":reason,"sent_count":self.sent,
          "send_jitter_ms":{"p50":float(np.percentile(values,50)),"p99":float(np.percentile(values,99)),"max":float(np.max(values))},
          "missed_deadline_count":missed,"max_lateness_ms":max_lateness,"deadline_by_stage":self.deadlines,
          "stages":by_stage,"alarm_events":self.alarm.events,
          "alarm_active_events":[x for x in self.alarm.events if x.get("active")],"move_stop":self.stop_calls[0] if self.stop_calls else None,
          "stop_calls":self.stop_calls,"stop_confirmation":self.stop_confirmation,
          "post_stop_travel_deg":None if self.stop_confirmation is None else self.stop_confirmation.get("post_stop_travel_deg"),
          "failure_detail":failure_detail,
          "gate_updates_applied":False,"candidate_evidence_for":["arm_stationary_verified"] if evidence_pass else []}

def run_hold_if_preflight(preflight_reasons,runner,target):
    if preflight_reasons:
        return {"pass":False,"evidence_pass":False,"sent_count":0,
          "reason":"PREFLIGHT:"+",".join(preflight_reasons),"gate_updates_applied":False}
    return runner.run(target)

def write_hold_outputs(run_dir,rows,summary):
    path=Path(run_dir);path.mkdir(parents=True,exist_ok=True)
    with (path/"hold_samples.jsonl").open("w") as stream:
        for row in rows:stream.write(json.dumps(row,separators=(",",":"))+"\n")
    (path/"summary.json").write_text(json.dumps(summary,indent=2)+"\n")
