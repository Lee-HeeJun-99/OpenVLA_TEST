# 입력·IK·endpoint·gripper offline 정렬 결과

## 1. 입력 전처리

`scripts/serve.sh vanilla`은 `runtime/vanilla-support/serve_adapter.py`를 실행한다. server는 JPEG를 PIL RGB로 decode하고 `crop_bottom_fraction=0.0`을 적용한 뒤 저장된 Prismatic processor를 호출한다. 별도의 입력 center crop은 없으며 health에도 `center_crop:false`가 명시돼 있다. processor 구현의 `CenterCrop(224)`는 이미 224×224로 resize한 결과에 적용되는 no-op이다.

| 단계 | matched15 sim 평가 | 기존 실기 runtime | rollout_ver2 |
|---|---|---|---|
| source color | RGB uint8 | ZED BGRA→RGB | RGB uint8 입력 계약 |
| client JPEG | quality **92**, optimize false | quality **95** | quality **92**, optimize false |
| server decode | RGB | 동일 vanilla server | 동일 동작 재현 |
| bottom crop | 0.0 | 0.0 | 0.0 |
| 입력 center crop | 없음 | 없음 | 없음 |
| processor | resize-naive bicubic 224×224, fused 6-channel normalization | 동일 server processor | 동일 저장 config 재현 |

`validation/preflight/input.jpg`의 동일 RGB를 sim client reference와 새 모듈로 각각 JPEG encode→decode→tensor화했다. JPEG bytes와 `(6,224,224)` FP32 tensor가 완전히 같았고 max absolute tensor diff는 `0.0`이다. 결과: `observation_preprocessing_validation.json`.

server 수정은 필요 없다. 읽기 전용 기존 client의 JPEG 95→92 변경 제안만 `proposed_runtime_preprocessing.diff`에 저장했으며 적용하지 않았다.

## 2. IK tolerance와 특이점 감시

모든 수치는 **[가정: offline 후보]**이며 pendant/인증 안전값이 아니다.

| 후보 | 값 |
|---|---:|
| IK 위치 허용 오차 | 0.05 mm |
| IK 회전 허용 오차 | 0.005° |
| Jacobian 최소 특이값 정지 | `σmin < 0.005` |
| 정지 사유 | `SINGULARITY_APPROACH` |

solver는 먼저 기존 `1e-8 m/rad` 정밀 해를 구하고 실패할 때만 후보 tolerance로 재시도한다. 처음부터 느슨한 tolerance를 쓰면 sample별 조기 종료로 joint acceleration이 불연속해졌기 때문에, tolerance는 fallback 인정 기준으로만 사용한다. action/해를 clamp하지 않는다.

기존 strict 실패점:

| frame | action/sample | q (deg) | σmin / condition | 판정 |
|---|---:|---|---:|---|
| world_left | 81/7 | `[-168.602,-45.471,-0.554,-32.557,-102.939,-111.297]` | `0.001474 / 1259.5` | J3≈0, elbow extension |
| tool_right | 120/25 | `[-164.290,-73.752,-1.430,-102.839,-69.116,273.880]` | `0.003776 / 494.8` | J3≈0, elbow extension |

J5는 각각 `-102.94°/-69.12°`이므로 wrist J5≈0 특이점이 아니며, shoulder보다 elbow extension 지표가 명확하다.

새 fail-closed replay:

| frame | 최초 정지 | σmin | family | 비고 |
|---|---:|---:|---|---|
| world_left | **step 80/sample 86** | `0.00498175` | `ELBOW_EXTENSION_J3` | 기존 81/7 전에 감시 정지 |
| tool_right | **step 120/sample 25** | `0.00377594` | `ELBOW_EXTENSION_J3` | fallback tolerance도 위치 잔차 0.1086 mm로 불충족 |

## 3. endpoint_policy와 성공 sim 로그

기본은 `endpoint_policy=log_only`; `stop` 설정에서만 N=3 streak가 정지 사유가 된다. 양 모드 모두 arm 6차원 event/streak를 기록하고 gripper는 제외한다.

matched15 성공 15 episode의 1,053 actions와 실기 190404의 224 actions를 같은 q01/q99 endpoint 값으로 분석했다. 각 셀은 `min/max 횟수 (최대 연속 min/max)`다.

| dim | matched15 성공 | 190404 실기 replay |
|---|---:|---:|
| dx | 5/7 (1/3) | 0/0 (0/0) |
| dy | 20/13 (4/11) | 0/0 (0/0) |
| dz | 38/23 (5/5) | 0/0 (0/0) |
| rx | 37/4 (4/2) | 25/0 (5/0) |
| ry | 17/2 (3/1) | 0/6 (0/1) |
| rz | 7/21 (2/9) | 0/4 (0/1) |

성공 sim에도 N=3 이상 streak가 여러 차원에서 존재하므로 기본 `log_only`가 데이터 근거와 맞는다. 상세 frequency는 `endpoint_policy_comparison.json`에 있다. 190404의 첫 N=3 event는 step 25 rx-min이지만 log_only에서는 정지하지 않는다.

## 4. gripper 정렬

matched15는 binary 즉시 반영이 아니다.

- action[6]을 `[0,1]` continuous absolute closedness target으로 사용한다.
- full travel duration은 0.6 s이고 60 Hz에서 physics step당 최대 변화는 `1/36`이다.
- 매 action은 12 physics step(0.2 s) 유지되며 매 step target 쪽으로 slew한다.
- 같은 `_apply_active_command()`에서 gripper를 먼저 advance하고 arm joint target도 적용하므로 팔과 병행한다.
- debounce, 상태변화 필터, arm 정지는 없다.

`ContinuousGripper`가 같은 absolute target/slew를 100 Hz 시간 등가(`1/(100·0.6)`)로 구현했다. out-of-range target은 clamp하지 않고 실패한다. dry-run에는 arm joint sample과 gripper command sample을 모두 기록한다.

**[미확인]** 실기 gripper API가 연속 absolute target, 0.6 s slew, 팔 동시 동작을 지원·허용하는지는 확인되지 않았다. 실제 publisher는 구현하지 않았으며 DESIGN 열린 항목으로 남겼다.

## 5. 검증

- `/usr/bin/python3 -m unittest discover -s . -p 'test_*.py' -v`: 19/19 PASS
- ROS import/노드/발행: 없음
- action clamp: 없음
- 기존 server/read-only runtime 수정: 없음
