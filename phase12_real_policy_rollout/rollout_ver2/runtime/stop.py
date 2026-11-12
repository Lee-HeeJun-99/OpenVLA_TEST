from __future__ import annotations
import time
import numpy as np


class StopCoordinator:
    def __init__(self, command_source, feedback, stop_transport, config,
                 clock=time.monotonic, sleep=time.sleep):
        self.command_source=command_source; self.feedback=feedback; self.transport=stop_transport
        self.config=config; self.clock=clock; self.sleep=sleep
        self.permanently_inhibited=False; self.events=[]

    def stop(self, reason):
        requested=self.clock(); self.command_source.inhibit()
        event={"reason":reason,"requested_at":requested,"publish_inhibited_at":self.clock(),
               "ack":None,"samples":[],"safe_state_preceded_ack_failure":False}
        try:
            stop_mode=int(self.config["move_stop_mode_candidate_unverified"])
            event["stop_mode"]={"value":stop_mode,
                                "name":self.config["move_stop_mode_name_candidate_unverified"],
                                "status":self.config["move_stop_mode_status"]}
            event["ack"]=self.transport.move_stop(stop_mode=stop_mode)
        except Exception as error:
            event["ack"]={"success":False,"error":str(error)}
        deadline=requested+float(self.config["stop_confirmation_timeout_s_candidate"])
        zero_since=None; safe_states=set(self.config["safe_stop_states"])
        while self.clock() <= deadline:
            try:
                sample=self.feedback.snapshot(self.clock())
                speed=float(np.max(np.abs(sample.joint_velocity_rad_s)))
                row={"time":self.clock(),"max_abs_velocity_rad_s":speed,
                     "robot_state":sample.robot_state}
                event["samples"].append(row)
                if speed <= float(self.config["zero_velocity_rad_s_candidate"]):
                    zero_since=row["time"] if zero_since is None else zero_since
                    if row["time"]-zero_since >= float(self.config["zero_velocity_hold_s_candidate"]):
                        ack_ok=event["ack"].get("success",False)
                        safe_override=(not ack_ok and sample.robot_state in safe_states)
                        if ack_ok or safe_override:
                            event["confirmed_at"]=row["time"]
                            event["state"]="STOP_CONFIRMED"
                            event["safe_state_preceded_ack_failure"]=safe_override
                            self.events.append(event); return event
                else: zero_since=None
            except RuntimeError as error:
                event["samples"].append({"time":self.clock(),"error":str(error)})
            self.sleep(0.01)
        event["state"]="STOP_CONFIRMATION_FAILED"
        self.permanently_inhibited=True
        self.command_source.inhibit(); self.events.append(event); return event
