"""Offline audit artifact generation. No robot/power/IO command capability."""
import argparse,hashlib,json
from pathlib import Path


def run(output):
    root=Path(__file__).resolve().parent
    def save(name,value):(output/name).write_text(json.dumps(value,indent=2))
    graph=json.loads((output/'ros_graph_snapshot.json').read_text())
    io=json.loads((output/'current_io_states.json').read_text())
    summary=json.loads((output/'summary.json').read_text())
    fail=root/'real_trials/20261014_gripper_stop_validation_01/close_once_01/gripper_mapping_test.json'
    save('gripper_code_audit.json',dict(model_closedness={'0':'OPEN','1':'CLOSED'},hysteresis=dict(open=.3,close=.7),
        historical_real_source='/home/ubuntu/robot_ws/src/doosan-robot2/dsr_example2/dsr_example/dsr_example/simple/single_robot_simple.py',
        real_collection='collect_episode001_from_align_home.py imports DigitalGripper; starts OPEN and calls CLOSE during grasp',
        current_bridge_source='/home/ubuntu/robot_ws/src/openvla_doosan_runtime/openvla_doosan_runtime/doosan_bridge_node.py',
        current_phase12_source='real_sink.py send_gripper',static_mapping_status='CHANNEL_CANDIDATE_FROM_REAL_CODE_NOT_CURRENT_WIRING_VERIFIED',
        pulse_variants=dict(historical_open_count=2,historical_close_count=1,historical_pulse_width_s=1,current_runtime_close_pulse_s=4,phase12_close_pulse_s=4,phase12_open_pulse_s=1,phase12_sink_pulse_count_not_consumed=True),
        legacy_model_hysteresis_inverted=True,failed_test_sha256=hashlib.sha256(fail.read_bytes()).hexdigest(),failed_test_preserved=True))
    contract=dict(service='/dsr01/io/set_tool_digital_output',type='dsr_msgs2/srv/SetToolDigitalOutput',
        index_semantics='ONE_BASED_FLANGE_TOOL_DIGITAL_PORT_1_TO_6; NOT_CONTROL_BOX_PORT_OR_PHYSICAL_PIN_NUMBER',
        driver_conversion='request.index -=1 -> GPIO_TOOL_DIGITAL_INDEX -> Drfl->set_tool_digital_output(index,value)',
        controller_IO_is_separate='/dsr01/io/set_ctrl_box_digital_output -> GPIO_CTRLBOX_DIGITAL_INDEX',
        open_candidate=dict(index=1,active_value=1,release_value=0,mode='pulse',width_s=1,historical_count=2),
        close_candidate=dict(index=2,active_value=1,release_value=0,mode='pulse',width_s=1,historical_count=1),
        current_states=io,output_zero_interpretation='OFF under DSR_ROBOT2 ON=1/OFF=0 and vendor bool bOnOff; not a jaw state',
        service_comment_discrepancy='Set/GetToolDigitalOutput.srv says0=ON,1=OFF; this conflicts with wrapper constants and vendor bool bOnOff. No inversion applied based on stale comments.',
        opposite_channel_handling='historical and current Phase12 pulse path toggle selected channel only; no explicit opposite-output-off interlock',
        wiring_binding='UNKNOWN',physical_pinout='UNKNOWN')
    save('gripper_io_contract.json',contract)
    save('live_io_endpoints.json',{key:[entry for entry in graph.get(key,[]) if any(word in entry[0] for word in ('/io/','/tool/','/modbus/','gripper'))] for key in ('topics','services','actions')})
    save('gripper_power_state.json',dict(classification='TOOL_POWER_UNKNOWN',actual_voltage=None,power_enabled=None,external_gripper_supply=None,pneumatic_supply=None,
        source_evidence='ROS/source search found no audited current tool-voltage getter or published measured voltage',
        tool_output_level_setter_exists_in_vendor_SDK=True,read_only_power_source_available=False,power_setter_called=False,
        needs_manual_evidence=['gripper model','actual Tool/flange or control-box connection','power/voltage setting or supply evidence','connector pinout/wiring']))
    save('gripper_close_test.json',dict(status='NOT_EXECUTED',reason='NO_NEW_POWER_WIRING_OR_PULSE_LEVEL_EVIDENCE',previous_fail=fail.as_posix(),previous_result='REQUEST_CLOSE_IO_ACK_PASS_ACTUAL_OPEN',new_physical_commands=0,automatic_retry=False))
    save('gripper_open_test.json',dict(status='NOT_EXECUTED',reason='CLOSE_MAPPING_NOT_PASS',new_physical_commands=0))
    save('gripper_abort_contract.json',dict(classification='GRIPPER_ABORT_BEHAVIOR_UNVERIFIED',abort_value=None,
        current_sink_abort='deenergize selected output using configured abort_value; refuses missing value',
        last_test_selected_coil_deenergization=0,jaw_failsafe_state='UNKNOWN',output_off_not_equivalent_to_safe_open_or_hold=True))
    save('initial_gripper_state.json',dict(expected='OPEN',last_operator_reported='OPEN',source='operator report following failed CLOSE; not fresh rollout state',current_readonly_DI_DO_not_jaw_feedback=True,fresh_before_rollout_required=True))
    save('final_readiness.json',dict(classification='MULTI_STEP_BLOCKED',gripper_mapping='GRIPPER_MAPPING_FAILED',stop_test='NOT_EXECUTED',
        prior_arm_test='ARM_ONLY_SHORT_HORIZON_PASS',tcp='TCP_ZERO_OFFSET_SUPPORTED_NOT_VERIFIED',physical_commands_this_audit=0,
        blockers=['CURRENT_WIRING_UNKNOWN','TOOL_OR_EXTERNAL_POWER_UNKNOWN','PULSE_VS_LEVEL_CURRENT_HARDWARE_UNVERIFIED','JAW_ABORT_BEHAVIOR_UNKNOWN','CLOSE_PHYSICAL_TEST_FAILED','OPEN_PHYSICAL_TEST_NOT_DONE','PHYSICAL_STOP_NOT_VALIDATED']))
    text=f'''# Gripper hardware/IO/power audit

Prior arm PASS and failed CLOSE artifact preserved unchanged; failed artifact SHA in gripper_code_audit.json. No new physical gripper/motion/Stop command. Hardware current REAL/AUTO/STANDBY, JointState RUNTIME_READY, fault null. Each audited scalar Tool/Control-box DI/DO getter on channel1/2 called once: all success=true, raw value0. Runtime source/receive >=100ms events0; maximum gaps {summary['current_session_runtime']['max_source_gap_s']*1000:.3f}/{summary['current_session_runtime']['max_receive_gap_s']*1000:.3f}ms. Following IO queries monitored10seconds with same subscriber.

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
'''
    (output/'GRIPPER_MAPPING_VALIDATION_REPORT.md').write_text(text)
    print(json.dumps(dict(classification='MULTI_STEP_BLOCKED',physical_commands=0,tool_power='UNKNOWN',IO_raw_values=[r['response']['value'] for r in io.values() if r.get('response')]),indent=2))


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('output',type=Path);run(parser.parse_args().output.resolve())
