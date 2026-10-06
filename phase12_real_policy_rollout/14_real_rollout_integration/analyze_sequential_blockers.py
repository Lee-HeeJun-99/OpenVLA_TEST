"""Recorded-only root-cause and sequential readiness reporting."""
import collections,csv,json,math,re,sqlite3,subprocess,sys
from pathlib import Path
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import JointState
ROOT=Path(__file__).resolve().parent
def main():
    out=ROOT/'real_trials'/sys.argv[1]
    old=ROOT/'real_trials'/'20261006_193719_integrated_rollout_readiness'
    camera=ROOT/'real_trials'/sys.argv[2] if len(sys.argv)>2 else None
    def write(name,data):(out/name).write_text(json.dumps(data,indent=2))
    def load(path):return json.loads(path.read_text())
    events=[]
    for label,p in [('prior_loaded',old),('jointstate_only',out)]:
        for kind in ('best_effort','reliable'):
            with (p/('jointstate_'+kind+'.csv')).open() as f:rows=list(csv.DictReader(f))
            for i,(a,b) in enumerate(zip(rows,rows[1:]),1):
                sourcegap=float(b['source'])-float(a['source']);receivegap=float(b['receive'])-float(a['receive'])
                if max(sourcegap,receivegap)<.1:continue
                events.append(dict(trial=label,subscriber=kind,index_before=i-1,index_after=i,source_before=float(a['source']),source_after=float(b['source']),receive_before=float(a['receive']),receive_after=float(b['receive']),wall_before=float(a['receive_wall']),wall_after=float(b['receive_wall']),source_gap=sourcegap,receive_gap=receivegap,elapsed_since_first=float(b['receive'])-float(rows[0]['receive']),header_age_after_s=float(b['receive_wall'])-float(b['source']),pattern='OLD_HEADER_RECEIVE_AFTER_WAIT' if receivegap>.1 and sourcegap<.1 else 'SOURCE_JUMP_BURST_RECEIVE' if sourcegap>.1 and receivegap<.1 else 'BOTH_GAPS',initial_only=i==1))
        bagrows=[]
        for db in (p/'jointstate_rosbag').glob('*.db3'):
            conn=sqlite3.connect('file:'+str(db)+'?mode=ro',uri=True)
            for stamp,data in conn.execute('select timestamp,data from messages order by timestamp'):
                m=deserialize_message(data,JointState);bagrows.append((stamp/1e9,m.header.stamp.sec+m.header.stamp.nanosec/1e9))
            conn.close()
        for i,(a,b) in enumerate(zip(bagrows,bagrows[1:]),1):
            if max(b[0]-a[0],b[1]-a[1])>=.1:events.append(dict(trial=label,subscriber='rosbag',index_before=i-1,index_after=i,source_before=a[1],source_after=b[1],receive_before=a[0],receive_after=b[0],source_gap=b[1]-a[1],receive_gap=b[0]-a[0],receive_clock='ROS_SYSTEM_NOT_MONOTONIC',initial_only=i==1))
    write('gap_events_precise.json',events)
    search=subprocess.run(['rg','-n','3000|3000ms|3s|3\\.0|wait_for|sleep_for|reconnect|timeout','/home/ubuntu/robot_ws/src/doosan-robot2','-g','*.cpp','-g','*.hpp','-g','*.h'],capture_output=True,text=True)
    (out/'timeout_source_search.txt').write_text(search.stdout)
    gate=load(out/'jointstate_clean_gate.json')
    rootcause='JOINTSTATE_PASS' if gate['status']=='JOINTSTATE_CLEAN_GATE_PASS' else 'JOINTSTATE_STILL_INCONCLUSIVE'
    (out/'01_jointstate_rootcause.md').write_text('''# JointState root-cause — not fixed

## Exact gap provenance

gap_events_precise.json contains both prior-loaded and new low-load BE/reliable/rosbag message indices, source before/after, receive times, wall times. Several >=3s events occur beyond index1 and recur during the connection window: not simply one cached first sample. Alternating OLD_HEADER_RECEIVE_AFTER_WAIT and SOURCE_JUMP_BURST_RECEIVE patterns must be kept separate. A source jump with a millisecond receive gap is not itself a3s subscriber receive stall, but preceding old-header wait events also exist.

## Timestamp generation — source verified

Installed joint_state_broadcaster2.53.1 counterpart source update(time,period): header.stamp=time; it reads numeric state interfaces and uses RealtimePublisher trylock()/unlockAndPublish(). Official source: https://github.com/ros-controls/ros2_controllers/blob/2.53.1/joint_state_broadcaster/src/joint_state_broadcaster.cpp lines325–355.
Installed controller_manager2.54.0 counterpart loop passes cm->now() to read/update/write. Official source: https://github.com/ros-controls/ros2_control/blob/2.54.0/controller_manager/src/ros2_control_node.cpp lines108–127. Package-version source correspondence is established; exact running binary was not decompiled against these sources in this test.
Thus header is host/node ROS-clock update time, not the controller RT feedback timestamp. Source gap alone cannot prove controller network packet generation stalled.

Local DRHWInterface::read line351 onwards calls Drfl.read_data_rt() and copies actual_joint_position/velocity into state interfaces (degree→radian); it does not assign JointState header. Vendor read_data_rt() internal transport/mutex code remains unavailable. The local real read null check logs but still dereferences data; a potential null bug, not a demonstrated cause here and not modified.

## Blocking candidates /3s search

Installed realtime_publisher.hpp: update-side trylock checks turn/mutex; publishingLoop waits on condition variable, copies outgoing, releases lock, then publisher_->publish(outgoing). Until publisher loop returns to next REALTIME turn, updates may skip serialization. A blocked rcl/rmw publish could cause old outgoing header followed by a fresh jump, matching observed pattern; this is a candidate, not runtime proof. Other candidates include hardware/read/update lock stalls and common DDS loss. No exact recurring3.07s logic was found in visible Doosan real read/update code. Startup500ms/1000ms waits and enum3000 values are not connected to current events. Prior client getter timeout3s was client-side and no getter ran here.

## Controlled low-load comparison

ZED launch3020648/OpenVLA3020619/RViz3017580 were SIGINT-stopped after PID/source validation; driver3017586 unchanged. Original parent launch preserved. Sixty-second JointState-only gate still failed with~3.07s max gap, no invalid values/duplicates/regressions. Removing these loads alone did not fix it; this does not rule out DDS/publish blocking or internal driverCPU. No claim of HOST_LOAD_RELATED as a confirmed cause.

Driver log delta during low-load trial was empty: no Skip-dt event can be aligned to these gaps. Static3s matches do not prove causal timing. Host raw stats/ROS-clock vsreceive-monotonic domains kept separate.

## Classification / next instrumentation

JOINTSTATE_STILL_INCONCLUSIVE. No speculative driver/core patch or restart was performed. Next safe evidence, if pursued: timestamps at cm read/update entry/exit; DRHWInterface read_data_rt entry/exit; broadcaster trylock success/failure; RealtimePublisher outgoing-copy→rcl/rmw publish entry/exit. Existing hardware traces require actual runtime enable evidence, not string presence. No current root-cause fix can honestly be declared.
''')
    if not camera:
        print(rootcause);return
    cam=load(camera/'camera_validation.json');shadow=load(camera/'openvla_shadow_summary.json');health=load(camera/'model_health.json')
    cold=ROOT/'real_trials'/'20261006_195100_sequential_camera_diagnostic'
    write('camera_cold_start_validation.json',load(cold/'camera_validation.json'))
    passive=ROOT/'real_trials'/'20261006_195234_automatic_hardware_state_audit'
    write('tcp_current_source_evidence.json',dict(source_trial=str(passive),active=load(passive/'active_tcp_tool.json'),offset=load(passive/'tcp_offset_search.json'),robot_state=load(passive/'robot_state_sources.json')))
    # Current binding search is read-only; a historical name is never promoted.
    configsearch=subprocess.run(['rg','-n','Tool_v1|tcp_offset|tool_offset|active_tcp','/home/ubuntu/robot_ws/src/openvla_doosan_runtime/config','/home/ubuntu/robot_ws/src/doosan-robot2/dsr_controller2/config'],capture_output=True,text=True)
    write('tcp_config_search.json',dict(current_active_name='UNKNOWN',verified_binding=False,output=configsearch.stdout,exit_code=configsearch.returncode))
    pred=[json.loads(line) for line in (camera/'openvla_shadow.jsonl').read_text().splitlines() if 'latency_breakdown' in json.loads(line)]
    summary={}
    for key in pred[0]['latency_breakdown'] if pred else []:
        vals=[r['latency_breakdown'][key] for r in pred if isinstance(r['latency_breakdown'].get(key),(float,int))]
        if vals:
            import statistics
            summary[key]=dict(median=statistics.median(vals),max=max(vals))
    write('02_camera_latency.json',dict(summary=summary,source_trial=str(camera),pixel_and_jpeg_contract_unchanged=True,hashing_after_inference=True,caveat='server inference_seconds times synchronized model call after processor/device transfer and after lock acquisition; not all HTTP time is GPU inference'))
    write('camera_final_validation.json',cam)
    write('openvla_shadow_summary.json',dict(shadow,health=health['status'],source_trial=str(camera)))
    (out/'05_openvla_shadow.jsonl').write_text((camera/'openvla_shadow.jsonl').read_text())
    write('03_tcp_getter_test.json',dict(requests=0,status='NOT_EXECUTED_PREREQUISITES_FAILED',jointstate=gate['status'],controller_connection='UNKNOWN',source_callback='getter-only',vendor_stability='UNVERIFIED'))
    write('04_robot_state.json',dict(connected='UNKNOWN',mode='UNKNOWN',authority='UNKNOWN',servo='UNKNOWN',protective_stop='UNKNOWN',emergency_stop='UNKNOWN',provenance='UNKNOWN'))
    write('manual_requirements.json',dict(operator='MANUAL_CONFIRMATION_REQUIRED',workspace='MANUAL_CONFIRMATION_REQUIRED',estop='MANUAL_CONFIRMATION_REQUIRED'))
    (out/'06_minimum_motion.jsonl').write_text(json.dumps(dict(event='BLOCKED_NOT_EXECUTED',command_issued=False,executed_action=None,robot_delivered_command=None))+'\n')
    report=f'''# Final sequential readiness — BLOCKED

STEP1: {rootcause}; clean60s FAIL. driver unchanged. See01_jointstate_rootcause.md and exact events.

```json
{json.dumps(gate['subscribers'],indent=2)}
```

STEP2: Camera{cam['status']}; {cam['count']} predictions/{cam['duration_s']:.3f}s; maxselectedage{cam['max_selected_frame_age_s']}s; camera_failure{cam['camera_failure']}. Health-before-snapshot/fresh<=10ms/fastRGB/JPEG95 kept; hashes computed after response. New detailedclient timing added without altering modelinput/productionthreshold. Cold-start observations included, not silently dropped. Delayed prediction rejected at safety boundary, not accepted by relaxing0.5s.

STEP3: TCP_FLANGE_ONLY (FKcapability, not actual TCP). ActiveTCP/tool/offset/currentposeUNKNOWN. Getter prereqsfailed, namequeries0; unstableposx0. Historicalvaluesnotreused.

STEP4: Connection/mode/authority/servo/protective/emergencyUNKNOWN; no fresh affirmative published signal verified. Eventsilence != connectionPASS.

STEP5: Operator/workspace/E-stopMANUAL_CONFIRMATION_REQUIRED, no repeated questions/noautomaticconfirmation.

STEP6: GPUhealth{health['status']}; {shadow['predictions']} realvision-onlypredictions, actionanomalies{shadow['anomalies']}; maxtranslation{shadow['translation_max_m']}m/maxrotation{shadow['rotation_max_deg']}deg/close{shadow['close_candidates']}. Jointstate andTCP incomplete: integratedShadowNOT_READY, notfullhardwarewatchdogPASS. Camera verdict above.

STEP7/8: AuthorizedNO, executedNO, physicalcommandcount0, requested/observeddeltanull. Protocol planned+0.5mmbaseX/rotation0/gripperdisabled, never sent. Short/full/OFT notexecuted.

Final: BLOCKED. Remaining: JointState runtime continuity/publish/DDS rootcause, selectedcameraage ifFAIL, activeTCPtransform, affirmativehardwarestates, manualsafety. No speculative driver modification; no controller/daemonrestart, setter, tool/modechange, motion, Home, gripper, physicalHold/Stop, realrollout. Camera/model were restored after controlledofftrial; RViz remains off and restorationisdocumented. Actualphysicalcommandsall0. NewdiagnosticcompilechecksPASS; nofullsuiteclaim.
'''
    report+='\n## Preserved cold-start and latest warm window\n\nFirst camera/model test after restart:68 predictions/30.205s, maxage0.645119s, camera_failure1 (frame0). Frame0 model call0.581026s, HTTP0.627919s, RGB3.617ms/JPEG12.292ms. Remaining67 frames max0.463698s. This was a real failed cold-start trial, not deleted. Current report uses a new independently recorded warm30s window; PASS does not guarantee future tail latency or waive revalidation after server restart. Existing verify_live_model_server.py sample inference can be used before fresh-window validation; any stale response is still rejected.\n\nLow-load CSV first sample header-vs-receive-wall age was~16.8/16.9ms, supporting first normal sample rather than an old3s initial cache in this trial. The shared gate window starts when both receive; sub-millisecond order means the earlier subscriber first row falls just outside it, causing coverage57s after the subsequent long gap. No samples were altered to manufacture PASS. Future diagnostic start selection now explicitly requires finite/name-valid and same-ROS-clock header age<100ms.\n\nLow-load condition removed ZED/OpenVLA/DoosanRViz; desktop/TeamViewer/other robot simulation processes were not forcibly killed. This tests these named heavy loads, not a perfectly isolated machine. RViz intentionally remains off; camera/model restored and driverPID3017586 unchanged.\n\nFocused regression: camera encoding4PASS and flange Shadow safety gate4PASS (8PASS/0FAIL/0SKIP); noROS command paths called. Whole-suite not rerun.\n'
    (out/'FINAL_SEQUENTIAL_READINESS_REPORT.md').write_text(report);print(report)
if __name__=='__main__':main()
