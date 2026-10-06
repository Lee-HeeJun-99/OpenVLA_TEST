# Rollout readiness software

상태: `SOFTWARE_ROLLOUT_READY` (recorded/mock command-disabled 경로).
`REAL_ROBOT_VALIDATION_PENDING`, 실제 rollout `NOT_AUTHORIZED`.

기존 CanonicalAction, corrected OFT timing, gripper-state decoupling,
SafetyPipeline, HardCommandGate, MockDoosanCommandSink, FsyncJsonlLogger를 재사용합니다.
새 코드는 protocol 상태, 증거 기반 readiness 검사, 로그 schema, metric 집계를 연결합니다.

## 실행

Bundle Python으로 `runtime/rollout_runner.py --model openvla|oft --input samples.jsonl
--output NEW_PATH.jsonl --initial-open-acknowledged`를 사용합니다.
`--initial-open-acknowledged`는 offline command knowledge 초기값이며 measured feedback이 아닙니다.
기본은 최초 safety reject에서 abort. `--audit-all`은 전체 recorded action을 분석하며
물리 rollout 모드가 아닙니다. `--protocol minimum_motion|short_horizon|full_task` 지원.
Config는 항상 command disabled이며 실제 API 전달 기능이 없습니다.
로그 output은 새 경로를 사용해야 합니다. 기존 logger는 append 방식입니다.

Readiness `check(evidence,safety,output_directory)`는 주어진 증거만 평가합니다.
camera/model live 증거가 없으면 false. `MOTION_READY`는 항상 false입니다.
Protocol config는 준비용이며 미래 actual command sink 구현을 대신하지 않습니다.

## 검증

기존 suite 100 tests 및 추가 readiness/abort/mock tests를 사용합니다.
Recorded Episode4의 유효 결과 선택: OpenVLA `openvla_step8130_retry`, OFT
`oft_vision_step28560_local`. 두 모델 모두 45 action을 audit-all로 처리했습니다.
OpenVLA 16 accepted /29 rejected, OFT 13/32. Task success는 null입니다.
기존 OpenVLA local 파일은 유효 prediction이 없어 0 records였고 이 기록도 보존했습니다.

## 연구 연결

episode_id, condition, matched_pair_id, sim/real_observation_id, observation_gap_score,
action_gap_translation/rotation/gripper를 기록합니다. 없는 값은 null.
Model intent close와 실제 gripper actuation timing은 구분합니다.
Recorded TCP path는 모델로 실행한 경로가 아닙니다.
성공·완료시간은 outcome 증거가 있을 때만 집계하며 현재 null입니다.

OFT software config 준비와 모델 행동 승인은 별개입니다.
`DEFERRED_FOR_MODEL_BEHAVIOR_REVIEW`를 유지합니다. Observation Gap 연구 방향은 유지합니다.

실제 command integration: `NOT_IMPLEMENTED`. 최소 현장 점검만으로 즉시 실제
rollout 가능한 상태라고 판정하지 않습니다. 향후 command sink와 실제 acknowledgement
연결 및 현장 검증이 필요합니다.
