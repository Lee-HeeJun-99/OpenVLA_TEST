from __future__ import annotations
import time


class LevelIOGripper:
    """Default interlocked held-level contract with mandatory DO readback."""
    def __init__(self,transport,gate,config,*,enabled=False,execution_mode="gripper_test",
                 clock=time.monotonic):
        self.transport=transport;self.gate=gate;self.config=config
        if execution_mode not in ("gripper_test","policy"):raise ValueError("invalid gripper mode")
        self.enabled=bool(enabled);self.execution_mode=execution_mode
        self.clock=clock;self.last_state=None;self.events=[]
        if self.enabled and self.execution_mode=="policy":
            self.gate.require_item("gripper_io_verified")

    def _write(self,index,value):
        return self.transport.set_tool_digital_output(index=index,value=value)
    def _read_pair(self):
        cfg=self.config["gripper"]
        result={}
        for key in ("open_output_index","close_output_index"):
            index=int(cfg[key]);response=self.transport.get_tool_digital_output(index=index)
            if not response.get("success",False):raise RuntimeError(f"GRIPPER_READBACK_FAILED:DO{index}")
            result[index]=int(response["value"])
        return result
    def _both_off(self):
        cfg=self.config["gripper"];off=int(cfg["off_service_value_observed"]);writes=[]
        for index in (int(cfg["open_output_index"]),int(cfg["close_output_index"])):
            writes.append({"index":index,"value":off,"ack":self._write(index,off)})
        return writes

    def command_model_value(self,value):
        closed=float(value)>=.5;requested=self.clock()
        if closed==self.last_state:
            event={"state":"suppressed","reason":"duplicate","closed":closed,
                   "requested_at":requested};self.events.append(event);return event
        if not self.enabled:
            event={"state":"record_only","closed":closed,"requested_at":requested}
            self.last_state=closed;self.events.append(event);return event
        self.gate.require(self.execution_mode)
        cfg=self.config["gripper"];open_i=int(cfg["open_output_index"])
        close_i=int(cfg["close_output_index"]);on=int(cfg["on_service_value_inferred"])
        off=int(cfg["off_service_value_observed"])
        sequence=[(open_i,off),(close_i,on)] if closed else [(close_i,off),(open_i,on)]
        desired={open_i:off if closed else on,close_i:on if closed else off}
        if desired[open_i]==on and desired[close_i]==on:raise RuntimeError("GRIPPER_DUAL_ON_FORBIDDEN")
        writes=[]
        try:
            for step,(index,output) in enumerate(sequence):
                ack=self._write(index,output);writes.append(
                    {"time":self.clock(),"index":index,"value":output,"ack":ack})
                if not ack.get("success",False):raise RuntimeError(f"GRIPPER_WRITE_FAILED:DO{index}")
                # The first OFF is the polarity tripwire. A mismatch means the
                # observed 0=Off inference is wrong; de-energize and stop.
                if step==0:
                    observed=self._read_pair()
                    if observed[index]!=off:raise RuntimeError("GRIPPER_POLARITY_MISMATCH")
            observed=self._read_pair()
            if observed!=desired:raise RuntimeError("GRIPPER_READBACK_MISMATCH")
            if observed[open_i]==on and observed[close_i]==on:
                raise RuntimeError("GRIPPER_DUAL_ON_READBACK")
        except Exception as error:
            safe_off=self._both_off()
            event={"state":"fault","reason":str(error),"closed":closed,
                   "requested_at":requested,"completed_at":self.clock(),
                   "writes":writes,"safe_off_writes":safe_off}
            self.events.append(event);return event
        self.last_state=closed
        event={"state":"completed","closed":closed,"requested_at":requested,
               "completed_at":self.clock(),"writes":writes,"readback":observed,
               "completion_evidence":"SERVICE_READBACK_ONLY_PHYSICAL_JAW_REQUIRES_OPERATOR"}
        self.events.append(event);return event


class BinaryIOGripper:
    """Non-default legacy timed-pulse alternative."""
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
        cfg=self.config["gripper"];pulse=cfg["pulse_alternative"];sequence=[]
        if closed:
            sequence=[(cfg["close_output_index"],pulse["active_value"],pulse["close_high_hold_s"]),
                      (cfg["close_output_index"],pulse["inactive_value"],0.0)]
        else:
            holds=pulse["open_high_holds_s"]
            sequence=[(cfg["open_output_index"],pulse["active_value"],holds[0]),
                      (cfg["open_output_index"],pulse["inactive_value"],pulse["open_between_low_hold_s"]),
                      (cfg["open_output_index"],pulse["active_value"],holds[1]),
                      (cfg["open_output_index"],pulse["inactive_value"],0.0)]
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


def create_gripper(transport,gate,config,**kwargs):
    """Configuration factory; held-level is the explicit default."""
    mode=config["gripper"].get("command_mode","interlocked_level")
    if mode=="interlocked_level":return LevelIOGripper(transport,gate,config,**kwargs)
    if mode=="operator_timed_pulse":return BinaryIOGripper(transport,gate,config,**kwargs)
    raise ValueError("unknown gripper command_mode")
