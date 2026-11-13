"""Mockable ServoJ time-parameter exploration state machine."""
from __future__ import annotations
import json,time
from pathlib import Path
import numpy as np
from controller.trajectory import minimum_jerk
from runtime import robot_state as rs

class TimeExploreFault(RuntimeError):pass

class TimeExploreRunner:
    def __init__(self,publisher,feedback,stop_transport,alarm,config,*,clock=time.monotonic,
                 sleep=time.sleep,abort_requested=lambda:False):
        self.publisher=publisher;self.feedback=feedback;self.stop=stop_transport;self.alarm=alarm
        self.config=config;self.clock=clock;self.sleep=sleep;self.abort_requested=abort_requested
        self.settings=config.get("servoj_time_explore",{})
        self.rows=[];self.trials=[];self.inhibited=False;self.stop_record=None
    def safety_reasons(self,start_rad,command_rad,final_j6_rad,direction,snapshot,now=None):
        reasons=list(self.feedback.stale_reasons(self.clock() if now is None else now))
        start=np.rad2deg(start_rad);actual=np.rad2deg(snapshot.joint_rad);command=np.rad2deg(command_rad)
        if np.max(np.abs(actual[:5]-start[:5]))>float(self.settings.get("j1_to_j5_abort_delta_deg",.05)):reasons.append("J1_TO_J5_DELTA_EXCEEDED")
        if abs(actual[5]-command[5])>float(self.settings.get("j6_tracking_error_abort_deg",1.)):reasons.append("J6_TRACKING_ERROR_EXCEEDED")
        final=np.rad2deg(final_j6_rad)
        overshoot=(actual[5]-final) if direction>0 else (final-actual[5])
        if overshoot>float(self.settings.get("j6_overshoot_abort_deg",.3)):reasons.append("J6_OVERSHOOT_EXCEEDED")
        if abs(np.rad2deg(snapshot.joint_velocity_rad_s[5]))>float(self.settings.get("j6_velocity_abort_deg_s",10.)):reasons.append("J6_VELOCITY_EXCEEDED")
        if snapshot.robot_state not in rs.ALLOWED_RUNNING_STATES:reasons.append(f"ROBOT_STATE_NOT_RUNNING:{snapshot.robot_state}")
        if self.alarm.active():reasons.append("ROBOT_ALARM_ACTIVE")
        return list(dict.fromkeys(reasons))
    def inhibit_and_stop(self,reason):
        if not self.inhibited:self.inhibited=True;self.publisher.inhibit()
        if self.stop_record is None:
            requested=self.clock()
            try:ack=self.stop.move_stop(stop_mode=int(self.config["move_stop_mode_candidate_unverified"]))
            except Exception as error:ack={"success":False,"error":str(error)}
            self.stop_record={"reason":reason,"requested_monotonic":requested,
              "completed_monotonic":self.clock(),"ack":ack}
    def _wait_slot(self,start,index,count,period,stats):
        deadline=start+index*period;now=self.clock();late=max(0.,now-deadline)
        stats["max_lateness_ms"]=max(stats["max_lateness_ms"],late*1000)
        if late>=period:
            skip=min(max(1,int(np.ceil(late/period))),count-index)
            stats["missed_deadline_count"]+=skip;return index+skip,None
        while self.clock()<deadline:self.sleep(max(0.,min(.001,deadline-self.clock())))
        now=self.clock();late=max(0.,now-deadline);stats["max_lateness_ms"]=max(stats["max_lateness_ms"],late*1000)
        if late>=period:
            skip=min(max(1,int(np.ceil(late/period))),count-index)
            stats["missed_deadline_count"]+=skip;return index+skip,None
        return index,deadline
    def _row(self,trial,stage,command,start,final,direction,send_time):
        now=self.clock();snap=self.feedback.snapshot(now)
        reasons=self.safety_reasons(start,command,final,direction,snap,now)
        row={"trial":trial,"stage":stage,"t_monotonic":now,"send_monotonic":send_time,
          "command_joint_deg_j1_to_j6":np.rad2deg(command).tolist(),
          "received_joint_rad":snap.joint_rad.tolist(),"received_velocity_rad_s":snap.joint_velocity_rad_s.tolist(),
          "robot_state":snap.robot_state,"alarm":self.alarm.record()}
        self.rows.append(row)
        if reasons:raise TimeExploreFault(",".join(reasons))
        return row
    def _send_leg(self,trial,label,start_q,from_j6,to_j6,time_param):
        rate=float(self.settings.get("rate_hz",100.));period=1./rate
        trajectory_duration=float(self.settings.get("trajectory_duration_s",1.))
        trajectory_count=int(round(trajectory_duration*rate))+1
        hold_count=int(round(float(self.settings.get("target_hold_duration_s",1.))*rate))
        u=np.linspace(0.,1.,trajectory_count);j6=from_j6+(to_j6-from_j6)*minimum_jerk(u)
        commands=[]
        for value in j6:
            q=start_q.copy();q[5]=value;commands.append(q)
        commands.extend([commands[-1].copy() for _ in range(hold_count)])
        stats={"missed_deadline_count":0,"max_lateness_ms":0.,"jitters_ms":[]};begin=self.clock();i=0
        direction=1 if to_j6>from_j6 else -1
        while i<len(commands):
            if self.abort_requested():raise TimeExploreFault("OPERATOR_ABORT")
            i,deadline=self._wait_slot(begin,i,len(commands),period,stats)
            if deadline is None:continue
            command=commands[i];now=self.clock();self.publisher.publish_with_time(command,time_param,now)
            stats["jitters_ms"].append((now-deadline)*1000)
            self._row(trial,label,command,start_q,to_j6,direction,now);i+=1
        return stats
    def _observe(self,trial,label,start_q,target_q,direction,duration=None):
        rate=float(self.settings.get("rate_hz",100.));period=1./rate
        duration=float(self.settings.get("post_stream_observe_s",2.) if duration is None else duration)
        count=int(round(duration*rate));stats={"missed_deadline_count":0,"max_lateness_ms":0.};begin=self.clock();i=0
        while i<count:
            if self.abort_requested():raise TimeExploreFault("OPERATOR_ABORT")
            i,deadline=self._wait_slot(begin,i,count,period,stats)
            if deadline is None:continue
            self._row(trial,label,target_q,start_q,target_q[5],direction,None);i+=1
        return stats
    @staticmethod
    def _leg_metrics(rows,from_deg,to_deg):
        sent=[r for r in rows if r["send_monotonic"] is not None]
        if not sent:return {"error":"NO_SENT_SAMPLES"}
        direction=1 if to_deg>from_deg else -1;half=(from_deg+to_deg)/2
        command_half=next((r["send_monotonic"] for r in sent if direction*(r["command_joint_deg_j1_to_j6"][5]-half)>=0),None)
        actual_half=next((r["t_monotonic"] for r in rows if direction*(np.rad2deg(r["received_joint_rad"][5])-half)>=0),None)
        target=to_deg;last_sent=sent[-1];post=[r for r in rows if r["t_monotonic"]>=last_sent["t_monotonic"]]
        target_command_time=next((r["send_monotonic"] for r in sent
          if abs(r["command_joint_deg_j1_to_j6"][5]-target)<=1e-9),last_sent["send_monotonic"])
        actual=np.array([np.rad2deg(r["received_joint_rad"][5]) for r in rows]);commands=np.array([r["command_joint_deg_j1_to_j6"][5] for r in rows])
        overshoot=max(0.,float(np.max(direction*(actual-target))))
        settle=next((r["t_monotonic"]-target_command_time for r in rows
          if r["t_monotonic"]>=target_command_time and abs(np.rad2deg(r["received_joint_rad"][5])-target)<=.01),None)
        at_stop=np.rad2deg(last_sent["received_joint_rad"][5]);final=np.rad2deg(rows[-1]["received_joint_rad"][5])
        return {"lag_50_percent_ms":None if command_half is None or actual_half is None else (actual_half-command_half)*1000,
          "max_tracking_error_deg":float(np.max(np.abs(actual-commands))),"overshoot_deg":overshoot,
          "post_send_final_error_deg":float(final-target),"post_send_additional_motion_deg":float(final-at_stop),
          "settling_time_s":settle}
    def _run_trial(self,index,time_param,start_q):
        trial_start=len(self.rows);delta=float(self.settings.get("j6_delta_deg",2.));forward=start_q.copy();forward[5]+=np.deg2rad(delta)
        deadline={};jitters=[]
        deadline["forward_send"]=self._send_leg(index,"FORWARD_SEND",start_q,start_q[5],forward[5],time_param);jitters+=deadline["forward_send"].pop("jitters_ms")
        deadline["forward_observe"]=self._observe(index,"FORWARD_OBSERVE",start_q,forward,1)
        deadline["return_send"]=self._send_leg(index,"RETURN_SEND",start_q,forward[5],start_q[5],time_param);jitters+=deadline["return_send"].pop("jitters_ms")
        deadline["return_observe"]=self._observe(index,"RETURN_OBSERVE",start_q,start_q,-1)
        rows=self.rows[trial_start:];forward_rows=[r for r in rows if r["stage"].startswith("FORWARD")]
        return_rows=[r for r in rows if r["stage"].startswith("RETURN")];values=np.asarray(jitters or [0.])
        actual=np.rad2deg(np.asarray([r["received_joint_rad"] for r in rows]));start_deg=np.rad2deg(start_q)
        final_error=np.rad2deg(np.asarray(rows[-1]["received_joint_rad"])-start_q)
        passed=bool(np.all(np.abs(final_error)<float(self.settings.get("final_start_tolerance_deg",.01))))
        return {"trial":index,"time_s":time_param,"pass":passed,
          "reason":"COMPLETED" if passed else "FINAL_NOT_START_POSE",
          "forward":self._leg_metrics(forward_rows,start_deg[5],start_deg[5]+delta),
          "return":self._leg_metrics(return_rows,start_deg[5]+delta,start_deg[5]),
          "j1_to_j5_max_change_deg":np.max(np.abs(actual[:,:5]-start_deg[:5]),axis=0).tolist(),
          "final_start_error_deg":final_error.tolist(),
          "send_jitter_ms":{"p50":float(np.percentile(values,50)),"p99":float(np.percentile(values,99)),"max":float(np.max(values))},
          "missed_deadline_count":sum(x["missed_deadline_count"] for x in deadline.values()),"deadline_by_stage":deadline}
    def _failure_detail(self,rows):
        if not rows:return {"sample_count":0}
        actual=np.rad2deg(np.asarray([r["received_joint_rad"] for r in rows]));velocity=np.rad2deg(
          np.asarray([r["received_velocity_rad_s"] for r in rows]))
        command=np.asarray([r["command_joint_deg_j1_to_j6"] for r in rows])
        last=rows[-1]
        return {"sample_count":len(rows),"interruption_monotonic":last["t_monotonic"],
          "interruption_joint_deg":actual[-1].tolist(),"interruption_velocity_deg_s":velocity[-1].tolist(),
          "max_abs_velocity_deg_s":np.max(np.abs(velocity),axis=0).tolist(),
          "max_abs_command_error_deg":np.max(np.abs(actual-command),axis=0).tolist(),
          "interruption_command_error_deg":(actual[-1]-command[-1]).tolist()}
    def _post_stop_observe(self,trigger_j6_rad,duration=1.0):
        start=self.clock();rows=[];zero_since=None;confirmed=None
        while self.clock()-start<float(duration):
            now=self.clock();snap=self.feedback.snapshot(now)
            row={"t_monotonic":now,"joint_rad":snap.joint_rad.tolist(),
              "joint_velocity_rad_s":snap.joint_velocity_rad_s.tolist(),"robot_state":snap.robot_state}
            rows.append(row);speed=float(np.max(np.abs(snap.joint_velocity_rad_s)))
            if speed<=.001:
                zero_since=now if zero_since is None else zero_since
                if now-zero_since>=.3 and confirmed is None:confirmed=now
            else:zero_since=None
            self.sleep(max(0.,min(.01,start+float(duration)-self.clock())))
        travel=max((abs(np.rad2deg(np.asarray(r["joint_rad"])[5]-trigger_j6_rad)) for r in rows),default=0.)
        return {"confirmed":confirmed is not None,"confirmed_monotonic":confirmed,"sample_count":len(rows),
          "post_stop_travel_deg":float(travel),"max_abs_velocity_rad_s":max((max(abs(x) for x in r["joint_velocity_rad_s"]) for r in rows),default=None)}
    def run(self,start_q,time_values=None):
        time_values=tuple(self.settings.get("time_values_s",(.10,.05,.02,.01))) if time_values is None else tuple(time_values)
        reason="COMPLETED"
        for index,value in enumerate(time_values):
            row_start=len(self.rows)
            try:trial=self._run_trial(index,float(value),np.asarray(start_q,float).copy())
            except Exception as error:
                partial=self.rows[row_start:]
                trial={"trial":index,"time_s":float(value),"pass":False,
                  "reason":f"{type(error).__name__}:{error}","failure_detail":self._failure_detail(partial)}
            self.trials.append(trial)
            if not trial["pass"]:
                reason=f"TRIAL_{index}_FAILED:{trial['reason']}";self.inhibit_and_stop(reason)
                trigger=(np.asarray(self.rows[-1]["received_joint_rad"])[5] if self.rows else np.asarray(start_q)[5])
                self.stop_record["stop_confirmation"]=self._post_stop_observe(trigger)
                self.stop_record["post_stop_travel_deg"]=self.stop_record["stop_confirmation"]["post_stop_travel_deg"]
                for skipped in range(index+1,len(time_values)):
                    self.trials.append({"trial":skipped,"time_s":float(time_values[skipped]),"pass":False,
                      "reason":"SKIPPED_AFTER_FAILURE","skipped":True})
                break
        return {"pass":reason=="COMPLETED","reason":reason,"trials":self.trials,
          "sent_count":sum(1 for r in self.rows if r["send_monotonic"] is not None),
          "move_stop":self.stop_record,
          "stop_confirmation":None if self.stop_record is None else self.stop_record.get("stop_confirmation"),
          "post_stop_travel_deg":None if self.stop_record is None else self.stop_record.get("post_stop_travel_deg"),
          "gate_updates_applied":False,"candidate_gate_updates":[]}

def write_time_explore_outputs(run_dir,rows,summary):
    path=Path(run_dir);path.mkdir(parents=True,exist_ok=True)
    with (path/"samples.jsonl").open("w") as stream:
        for row in rows:stream.write(json.dumps(row,separators=(",",":"))+"\n")
    (path/"summary.json").write_text(json.dumps(summary,indent=2)+"\n")
