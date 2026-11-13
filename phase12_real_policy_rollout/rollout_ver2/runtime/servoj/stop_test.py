"""Mockable ServoJ streaming-stop validation."""
from __future__ import annotations
import time
import numpy as np
from controller.trajectory import minimum_jerk
from runtime import robot_state as rs
from .tracking import tracking_threshold_deg,validate_servoj_time

class StopServoJRunner:
    def __init__(self,publisher,feedback,stop_transport,alarm,config,*,clock=time.monotonic,
                 sleep=time.sleep,abort_requested=lambda:False):
        self.publisher=publisher;self.feedback=feedback;self.stop=stop_transport;self.alarm=alarm
        self.config=config;self.clock=clock;self.sleep=sleep;self.abort_requested=abort_requested
        self.rows=[];self.events=[];self.inhibited=False
    def _stop(self,reason):
        if not self.inhibited:self.publisher.inhibit();self.inhibited=True;self.events.append({"event":"PUBLISH_INHIBITED","time":self.clock()})
        requested=self.clock();self.events.append({"event":"MOVESTOP_REQUESTED","time":requested})
        try:ack=self.stop.move_stop(stop_mode=int(self.config["move_stop_mode_candidate_unverified"]))
        except Exception as error:ack={"success":False,"error":str(error)}
        completed=self.clock();self.events.append({"event":"MOVESTOP_COMPLETED","time":completed,"ack":ack})
        return {"reason":reason,"requested_monotonic":requested,"ack_completed_monotonic":completed,
          "response_s":completed-requested,"ack":ack}
    def _observe_stop(self,trigger_q,timeout=1.):
        begin=self.clock();rows=[];zero_since=None;confirmed=None;safe=False
        while self.clock()-begin<float(timeout):
            now=self.clock();snap=self.feedback.snapshot(now);speed=float(np.max(np.abs(snap.joint_velocity_rad_s)))
            rows.append({"time":now,"joint_rad":snap.joint_rad.tolist(),"velocity_rad_s":snap.joint_velocity_rad_s.tolist(),"robot_state":snap.robot_state})
            safe|=snap.robot_state in rs.CONTROLLER_STOP_STATES
            if speed<=.001:
                zero_since=now if zero_since is None else zero_since
                if now-zero_since>=.3:confirmed=now;break
            else:zero_since=None
            self.sleep(max(0.,min(.01,begin+float(timeout)-self.clock())))
        travel=max((abs(np.rad2deg(np.asarray(r["joint_rad"])[5]-trigger_q[5])) for r in rows),default=0.)
        return {"confirmed":confirmed is not None,"confirmed_monotonic":confirmed,
          "trigger_to_confirmation_s":None if confirmed is None else confirmed-begin,
          "post_stop_travel_deg":float(travel),"safe_state_observed":safe,"samples":rows}
    def run(self,start_q):
        settings=self.config["stop_test_servoj"];time_s=validate_servoj_time(settings["time_s"],
          self.config["servoj_time_min_verified_s"],self.config["servoj_time_max_verified_s"])
        rate=100.;period=.01;duration=1.;start_q=np.asarray(start_q,float);target=start_q.copy();target[5]+=np.deg2rad(2.)
        u=np.linspace(0.,1.,101);commands=[]
        for value in start_q[5]+(target[5]-start_q[5])*minimum_jerk(u):
            q=start_q.copy();q[5]=value;commands.append(q)
        threshold=float(tracking_threshold_deg(time_s,3.75));trigger_speed=1.875
        begin=self.clock();index=0;trigger=None;reason=None;max_j15=np.zeros(5);max_error=0.;max_speed=0.
        missed=0;max_lateness=0.;jitters=[]
        try:
            while self.clock()-begin<3.0 and trigger is None:
                if self.abort_requested():raise RuntimeError("OPERATOR_ABORT")
                deadline=begin+index*period;late=max(0.,self.clock()-deadline);max_lateness=max(max_lateness,late*1000)
                if late>=period:
                    skip=max(1,int(np.ceil(late/period)));missed+=skip;index+=skip;continue
                while self.clock()<deadline:self.sleep(max(0.,min(.001,deadline-self.clock())))
                now=self.clock();late=max(0.,now-deadline);max_lateness=max(max_lateness,late*1000)
                if late>=period:
                    skip=max(1,int(np.ceil(late/period)));missed+=skip;index+=skip;continue
                command=commands[min(index,len(commands)-1)];self.publisher.publish_with_time(command,time_s,now);jitters.append((now-deadline)*1000)
                snap=self.feedback.snapshot(now);actual=np.rad2deg(snap.joint_rad);start_deg=np.rad2deg(start_q);cmd=np.rad2deg(command)
                error=abs(actual[5]-cmd[5]);speed=abs(np.rad2deg(snap.joint_velocity_rad_s[5]));max_error=max(max_error,error);max_speed=max(max_speed,speed)
                max_j15=np.maximum(max_j15,np.abs(actual[:5]-start_deg[:5]))
                row={"index":index,"send_monotonic":now,"command_joint_deg":cmd.tolist(),"actual_joint_deg":actual.tolist(),
                  "velocity_deg_s":np.rad2deg(snap.joint_velocity_rad_s).tolist(),"robot_state":snap.robot_state,
                  "j6_tracking_error_deg":error};self.rows.append(row)
                faults=list(self.feedback.stale_reasons(now))
                if np.max(max_j15)>.05:faults.append("J1_TO_J5_DELTA_EXCEEDED")
                if error>threshold:faults.append("J6_TRACKING_ERROR_EXCEEDED")
                if actual[5]-np.rad2deg(target[5])>.3:faults.append("J6_OVERSHOOT_EXCEEDED")
                if speed>10.:faults.append("J6_VELOCITY_EXCEEDED")
                if snap.robot_state not in rs.ALLOWED_RUNNING_STATES:faults.append(f"ROBOT_STATE_NOT_RUNNING:{snap.robot_state}")
                if self.alarm.active():faults.append("ROBOT_ALARM_ACTIVE")
                if faults:raise RuntimeError(",".join(dict.fromkeys(faults)))
                if speed>=trigger_speed:trigger={"time":now,"joint_rad":snap.joint_rad.copy(),"speed_deg_s":speed,"sample":row};break
                index+=1
            if trigger is None:
                snap=self.feedback.snapshot(self.clock());trigger={"time":self.clock(),"joint_rad":snap.joint_rad.copy(),
                  "speed_deg_s":abs(np.rad2deg(snap.joint_velocity_rad_s[5]))};reason="INVALID_TRIGGER_TIMEOUT"
            else:reason="VELOCITY_TRIGGER"
        except Exception as error:
            reason=f"SAFETY_FAULT:{type(error).__name__}:{error}"
            try:snap=self.feedback.snapshot(self.clock());trigger={"time":self.clock(),"joint_rad":snap.joint_rad.copy(),"speed_deg_s":abs(np.rad2deg(snap.joint_velocity_rad_s[5]))}
            except Exception:trigger={"time":self.clock(),"joint_rad":start_q.copy(),"speed_deg_s":None}
        stop_record=self._stop(reason);confirmation=self._observe_stop(trigger["joint_rad"])
        confirmation["trigger_to_confirmation_s"]=(None if confirmation["confirmed_monotonic"] is None else
          confirmation["confirmed_monotonic"]-trigger["time"])
        ack_ok=bool(stop_record["ack"].get("success")) or confirmation["safe_state_observed"]
        passed=(reason=="VELOCITY_TRIGGER" and ack_ok and confirmation["confirmed"] and
          confirmation["trigger_to_confirmation_s"] is not None and confirmation["trigger_to_confirmation_s"]<=1. and
          confirmation["post_stop_travel_deg"]<.3 and float(np.max(max_j15))<.05 and not self.alarm.active())
        trigger_record=dict(trigger);trigger_record["joint_rad"]=np.asarray(trigger_record["joint_rad"]).tolist()
        jitter=np.asarray(jitters or [0.])
        return {"pass":passed,"reason":reason,"tracking_threshold_deg":threshold,"trigger":trigger_record,
          "move_stop":stop_record,"stop_confirmation":confirmation,
          "post_stop_travel_deg":confirmation["post_stop_travel_deg"],"j1_to_j5_max_change_deg":max_j15.tolist(),
          "max_tracking_error_deg":max_error,"max_j6_velocity_deg_s":max_speed,"events":self.events,
          "send_jitter_ms":{"p50":float(np.percentile(jitter,50)),"p99":float(np.percentile(jitter,99)),"max":float(np.max(jitter))},
          "missed_deadline_count":missed,"max_lateness_ms":max_lateness,
          "sent_count":len(self.rows),"gate_updates_applied":False,"candidate_gate_updates":["stop_servoj_verified"] if passed else []}
