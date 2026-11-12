# Sim home과 v1 실기 시작 자세 정렬

## 결론

기준은 IK 복원이 아니라 실제 Isaac 관절값과 현장 확인된 장착 오프셋이다.

Sim home은 `[-182.535617, -1.058471, -40.983990, 1.253979, -113.505675, -178.500246]°`다. **[확인: 현장 육안 확인, 2026-11-13]** 실기 그리퍼 장착은 link_6 z축 기준 180° 돌아가 있으며 `J6_real=J6_sim+180°`일 때 같은 물리 그리퍼 방향이다. 따라서 real home은 다음이다.

`[-182.535617, -1.058471, -40.983990, 1.253979, -113.505675, 1.499754]°`

관절별 ±2° 후보로 재판정하면 v1 10개 중 **8개 통과**, `190212`와 `183804` 2개가 거부된다. 요청문의 “9개 통과” 예상과 산술적으로 다르지만, 명시된 두 거부 런을 적용한 실제 결과는 8/10이다.

장착 보정을 포함하면 통과 8개에서 `tool0_real`은 sim의 `tool0_sim`과 위치 0.284–0.454 mm, 방향 0.033–0.144° 이내로 같다. `190212`는 9.968 mm/4.135°, `183804`는 117.021 mm/34.470° 차이다.

## 원자료와 코드 근거

- home schema는 joint 이름과 rad 값을 순서대로 검증한다: 읽기 전용 `collection_reference/a0509_control/home_pose.py:13-35,45-63,66-76`.
- collection runtime은 config가 있으면 관절값을 home으로 로드한다: `collection_reference/a0509_control/isaac_runtime.py:262-284`.
- 실제 home 값: `isaac_sim/config/a0509_home_pose.json:2-19`.
- evaluator는 validated home으로 reset한다: `isaac_sim/a0509_control/vla_validation.py:775-796`.
- layout에 `initial_joint_positions_rad`가 있을 때만 덮어쓴다: 같은 파일 `:851-893`. matched15 manifest `validation/layouts/validation_layouts_seed9406_15.json:1-45`에는 이 필드가 없다.
- 실제 Isaac 자료는 phase11 baseline 5개 파일의 `actual_joint_position_rad` 총 225 sample로, FK 검증에 사용한 동일 자료다.

## 실제 sim 관절 범위

| joint | home° | 225 sample min° | max° |
|---|---:|---:|---:|
| J1 | -182.536 | -217.630 | -182.536 |
| J2 | -1.058 | -34.953 | 2.155 |
| J3 | -40.984 | -99.657 | -40.984 |
| J4 | 1.254 | 0.141 | 5.926 |
| J5 | -113.506 | -113.506 | -68.638 |
| J6 | -178.500 | -214.441 | -178.500 |

실제 sim wrist는 J4 양수 근처, J5 음수, J6 약 -180° branch다. 기존 matched15 IK 복원의 J5 양수 범위는 다른 IK branch이며 시작 자세나 관절 사용 범위의 근거가 아니다.

실기 기준 범위는 J1–J5가 위와 같고 J6에 180°를 더한 `[-34.441, 1.500]°`다. 이는 관절 margin 비교 기준이며 pendant limit 자체가 아니다.

## v1 시작 관절각 비교

| run | ΔJ1..ΔJ6 from real home (deg) | 판정 | tool0_real↔tool0_sim mm/deg |
|---|---|---|---:|
| 181506 | -0.017,-0.013,-0.022,+0.003,+0.006,-0.000 | PASS | 0.284 / 0.033 |
| 182408 | +0.010,-0.016,-0.040,-0.102,+0.031,-0.143 | PASS | 0.454 / 0.144 |
| 183404 | -0.017,-0.013,-0.022,+0.003,+0.006,-0.000 | PASS | 0.284 / 0.033 |
| 183554 | -0.017,-0.013,-0.022,+0.003,+0.006,-0.000 | PASS | 0.284 / 0.033 |
| 183804 | +3.849,-16.632,+13.853,-1.242,-6.953,-30.056 | REJECT | 117.021 / 34.470 |
| 183848 | -0.017,-0.013,-0.022,+0.003,+0.006,-0.000 | PASS | 0.284 / 0.033 |
| 185038 | -0.017,-0.013,-0.022,+0.003,+0.006,-0.000 | PASS | 0.284 / 0.033 |
| 190008 | -0.017,-0.013,-0.022,+0.003,+0.006,-0.000 | PASS | 0.284 / 0.033 |
| 190212 | +0.320,-1.051,+0.239,-0.040,-0.517,-3.653 | REJECT | 9.968 / 4.135 |
| 190404 | -0.017,-0.013,-0.022,+0.003,+0.006,-0.000 | PASS | 0.284 / 0.033 |

## 사전 검사

`starting_home_reasons()`는 real home과 관절별로 직접 비교한다. angle wrapping, alternate branch 허용, clamp, 자동 home 이동은 하지 않는다.

- **[후보]** 허용 오차는 관절별 ±2.0°다.
- 초과 시 `START_NOT_REAL_HOME:J<n>`으로 차단한다.
- 기존 v1은 8개 통과, `190212`와 `183804`가 차단된다.
- 현장 encoder/펜던트 evidence로 후보를 검토하기 전에는 승인 한계가 아니다.

전체 수치와 원본 경로는 `home_alignment.json`에 있다.

## 불변성 테스트

동일한 world-left action 3개를 `q_sim/tool0_sim`과 `q_sim+[0,0,0,0,0,180°]/tool0_real`에 적용했다. 매 단계에서 world 기준 물리 그리퍼 frame을 비교한 결과 최대 위치 차이는 `0.0 m`, 최대 방향 차이는 `4.00e-14°`였다. 또한 동일 target을 link_6로 역변환했을 때 두 link frame이 정확히 Rz(180°) 관계임을 확인했다. 테스트는 `controller/test_controller.py::test_sim_real_mount_action_trajectory_invariance`다.

## 로그 비교 규약

- 실기: `FeedbackJsonlLogger`가 `frame=tool0_real`, `tool0_real_position_m`, `tool0_real_rotation_matrix`를 기록한다.
- sim: 기록된 `tool0_sim`을 사용한다.
- 비교: `tool0_real ↔ tool0_sim` 물리 그리퍼 frame끼리만 비교한다. `link_6` 또는 서로 다른 tool offset을 직접 비교하지 않는다.
