# Gripper hardware/IO/power audit

Prior arm PASS and failed CLOSE artifact preserved unchanged; failed artifact SHA in gripper_code_audit.json. No new physical gripper/motion/Stop command. Hardware current REAL/AUTO/STANDBY, JointState RUNTIME_READY, fault null. Each audited scalar Tool/Control-box DI/DO getter on channel1/2 called once: all success=true, raw value0. Runtime source/receive >=100ms events0; maximum gaps 12.803/15.158ms. Following IO queries monitored10seconds with same subscriber.

| Candidate logical operation | Endpoint / channel | Value | Historical mode |
|---|---|---|---|
| OPEN | Tool/flange DO1 |1 then0 |1second pulse, twice in real collector |
| CLOSE | Tool/flange DO2 |1 then0 |1second pulse, once in real collector |

`output2` means Tool/flange digital port2. It does not mean Control-box DO2 and is not a connector physical pin number. ROS one-based2 becomes vendor GPIO_TOOL_DIGITAL_INDEX enum1. Control-box outputs have different service/API and enum. No wiring document/model-specific pinout found that binds the CURRENT gripper to these tool channels. Code's initial OPEN variable is commanded knowledge, not a physical sensor.

SDK bool bOnOff and DSR_ROBOT2 ON=1/OFF=0 support active1/release0. Some .srv comments reverse ON/OFF; they were not used to flip polarity. Current raw output0 is consistent with released/off state, but does not prove voltage present, wire continuity, valve operation or jaw OPEN. DI bits have no verified jaw limit-switch mapping. No power setter was invoked.

Historical collector uses pulses. Current bridge/Phase12 configuration says close4seconds; historical collector says1second. Phase12 sink ignores configured pulse counts. These differences are evidence to investigate, not permission to extend pulse or swap channels automatically. Actual current hardware pulse-trigger vs held-level requirement remains UNKNOWN without gripper model/wiring/manual. Prior failed pulse does not distinguish no reaction from an unobserved transient close/reopen.

Opposite channel explicit interlock not found in historical/Phase12 path; current both tool outputs raw0 does not verify dual-solenoid wiring. Abort value null; deenergizing a coil is not verified OPEN/CLOSE/HOLD_LAST behavior. No gripper failsafe jaw state is approved.

Tool power **UNKNOWN**. No audited live voltage readout found. SDK output-level setter existence is not voltage evidence and was not called. Need actual gripper model, flange/tool-vs-control-box connector/wiring photo, and pendant tool-power/IO or external supply evidence. Also clarify whether last test produced absolutely no jaw motion or a transient close then reopen.

CLOSE retest **NOT_EXECUTED**: power/wiring/current pulse-level evidence missing. OPEN **NOT_EXECUTED**: no successful CLOSE. Stop/multi-step **NOT_EXECUTED**: staged prerequisite failed; cannot promote to arm-only multi-step while Stop path remains unvalidated. Existing failed CLOSE remains GRIPPER_MAPPING_FAILED, prior arm PASS retained. Formal TCP remains supported not verified.

Final **MULTI_STEP_BLOCKED**. Physical commands this audit **0**. No channel brute force, retries, extra pulses, voltage/TCP setter, recovery/home or risky Stop trajectory. Next evidence gathering is read-only/manual wiring and power inspection, followed by separately approved grounded calibration.
