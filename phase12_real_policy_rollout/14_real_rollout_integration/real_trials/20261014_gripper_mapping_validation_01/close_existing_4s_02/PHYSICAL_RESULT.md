# CLOSE mapping failed

One explicitly approved CLOSE operation used Tool/flange digital output2, value1 for4seconds then value0. Both IO ACKs success=true, approximately5.1ms each. Operator explicitly reported "안닫혔어"; actual requested CLOSED state was not observed. Result GRIPPER_MAPPING_FAILED. No ACK-as-physical-success reinterpretation.

Arm commands0, physical Hold/Stop0; logical gripper operations1, IO service requests2. No additional automatic retry, longer level hold, opposite output or channel scan. Earlier artifacts retained unchanged. The initial1second pulse and this4second pulse both failed to produce confirmed CLOSE; pulse duration alone does not explain the failure on the present evidence.

Next: confirm actual gripper model, Tool/flange-vs-Control-box connector/wiring, tool or external power/air supply, and observed electrical/valve output. No software-inferred current wiring or safe abort jaw state. Stop test and multi-step remain blocked. Prior arm1-step PASS remains unchanged.
