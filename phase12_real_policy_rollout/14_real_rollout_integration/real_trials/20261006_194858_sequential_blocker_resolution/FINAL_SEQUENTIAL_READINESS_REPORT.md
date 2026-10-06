# Final sequential readiness — BLOCKED

STEP1: JOINTSTATE_STILL_INCONCLUSIVE; clean60s FAIL. driver unchanged. See01_jointstate_rootcause.md and exact events.

```json
{
  "best_effort": {
    "count": 2646,
    "coverage": 56.97961651487276,
    "rate": 46.420108835053405,
    "max_source_gap": 3.0700149536132812,
    "max_receive_gap": 3.072312918957323,
    "latest_age": 0.001265964936465025,
    "invalid": 0,
    "duplicate": 0,
    "regression": 0,
    "first_receive_delay": 3.4728790940716863,
    "pass": false
  },
  "reliable": {
    "count": 2647,
    "coverage": 60.03372857393697,
    "rate": 44.07522342613139,
    "max_source_gap": 3.0700149536132812,
    "max_receive_gap": 3.072433311957866,
    "latest_age": 0.0010979687795042992,
    "invalid": 0,
    "duplicate": 0,
    "regression": 0,
    "first_receive_delay": 0.41893503116443753,
    "pass": false
  }
}
```

STEP2: CameraPASS; 68 predictions/30.189s; maxselectedage0.4690681351348758s; camera_failure0. Health-before-snapshot/fresh<=10ms/fastRGB/JPEG95 kept; hashes computed after response. New detailedclient timing added without altering modelinput/productionthreshold. Cold-start observations included, not silently dropped. Delayed prediction rejected at safety boundary, not accepted by relaxing0.5s.

STEP3: TCP_FLANGE_ONLY (FKcapability, not actual TCP). ActiveTCP/tool/offset/currentposeUNKNOWN. Getter prereqsfailed, namequeries0; unstableposx0. Historicalvaluesnotreused.

STEP4: Connection/mode/authority/servo/protective/emergencyUNKNOWN; no fresh affirmative published signal verified. Eventsilence != connectionPASS.

STEP5: Operator/workspace/E-stopMANUAL_CONFIRMATION_REQUIRED, no repeated questions/noautomaticconfirmation.

STEP6: GPUhealthMODEL_HEALTH_PASS; 68 realvision-onlypredictions, actionanomalies0; maxtranslation0.0026689753779743286m/maxrotation0.6262013152800626deg/close0. Jointstate andTCP incomplete: integratedShadowNOT_READY, notfullhardwarewatchdogPASS. Camera verdict above.

STEP7/8: AuthorizedNO, executedNO, physicalcommandcount0, requested/observeddeltanull. Protocol planned+0.5mmbaseX/rotation0/gripperdisabled, never sent. Short/full/OFT notexecuted.

Final: BLOCKED. Remaining: JointState runtime continuity/publish/DDS rootcause, selectedcameraage ifFAIL, activeTCPtransform, affirmativehardwarestates, manualsafety. No speculative driver modification; no controller/daemonrestart, setter, tool/modechange, motion, Home, gripper, physicalHold/Stop, realrollout. Camera/model were restored after controlledofftrial; RViz remains off and restorationisdocumented. Actualphysicalcommandsall0. NewdiagnosticcompilechecksPASS; nofullsuiteclaim.

## Preserved cold-start and latest warm window

First camera/model test after restart:68 predictions/30.205s, maxage0.645119s, camera_failure1 (frame0). Frame0 model call0.581026s, HTTP0.627919s, RGB3.617ms/JPEG12.292ms. Remaining67 frames max0.463698s. This was a real failed cold-start trial, not deleted. Current report uses a new independently recorded warm30s window; PASS does not guarantee future tail latency or waive revalidation after server restart. Existing verify_live_model_server.py sample inference can be used before fresh-window validation; any stale response is still rejected.

Low-load CSV first sample header-vs-receive-wall age was~16.8/16.9ms, supporting first normal sample rather than an old3s initial cache in this trial. The shared gate window starts when both receive; sub-millisecond order means the earlier subscriber first row falls just outside it, causing coverage57s after the subsequent long gap. No samples were altered to manufacture PASS. Future diagnostic start selection now explicitly requires finite/name-valid and same-ROS-clock header age<100ms.

Low-load condition removed ZED/OpenVLA/DoosanRViz; desktop/TeamViewer/other robot simulation processes were not forcibly killed. This tests these named heavy loads, not a perfectly isolated machine. RViz intentionally remains off; camera/model restored and driverPID3017586 unchanged.

Focused regression: camera encoding4PASS and flange Shadow safety gate4PASS (8PASS/0FAIL/0SKIP); noROS command paths called. Whole-suite not rerun.
