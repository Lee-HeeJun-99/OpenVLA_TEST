from __future__ import annotations
import time


class BinaryIOGripper:
    """Latest physical contract: Tool DO1 OPEN double pulse, DO2 CLOSE pulse."""
    def __init__(self, transport, gate, config, *, enabled=False,
                 clock=time.monotonic, sleep=time.sleep):
        self.transport=transport; self.gate=gate; self.config=config
        self.enabled=bool(enabled); self.clock=clock; self.sleep=sleep
        self.last_state=None; self.events=[]

    def _write(self,index,value):
        return self.transport.set_tool_digital_output(index=index,value=value)

    def command_model_value(self, value):
        closed=float(value)>=0.5
        requested=self.clock()
        if closed==self.last_state:
            event={"state":"suppressed","reason":"duplicate","closed":closed,
                   "requested_at":requested}; self.events.append(event); return event
        if not self.enabled:
            event={"state":"record_only","closed":closed,"requested_at":requested}
            self.last_state=closed; self.events.append(event); return event
        self.gate.require("gripper_test")
        cfg=self.config["gripper"]; sequence=[]
        if closed:
            sequence=[(cfg["close_output_index"],1,cfg["close_high_hold_s"]),
                      (cfg["close_output_index"],0,0.0)]
        else:
            holds=cfg["open_high_holds_s"]
            sequence=[(cfg["open_output_index"],1,holds[0]),
                      (cfg["open_output_index"],0,cfg["open_between_low_hold_s"]),
                      (cfg["open_output_index"],1,holds[1]),
                      (cfg["open_output_index"],0,0.0)]
        writes=[]
        for index,output,hold in sequence:
            result=self._write(index,output); writes.append({"time":self.clock(),
                "index":index,"value":output,"ack":result})
            if not result.get("success",False):
                event={"state":"failed","closed":closed,"requested_at":requested,
                       "completed_at":self.clock(),"writes":writes}
                self.events.append(event); return event
            if hold: self.sleep(float(hold))
        self.last_state=closed
        event={"state":"completed","closed":closed,"requested_at":requested,
               "completed_at":self.clock(),"writes":writes,
               "completion_evidence":"SERVICE_ACK_ONLY_NO_INPUT_SIGNAL"}
        self.events.append(event); return event
