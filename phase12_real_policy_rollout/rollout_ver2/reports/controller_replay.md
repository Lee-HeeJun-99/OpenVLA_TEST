# Controller replay 보강 결과 (2026-11-13)

> **후속 갱신:** `reports/controller_policy_update.md`가 endpoint 기본 모드(`log_only`), IK tolerance/특이점 감시, continuous gripper를 반영한 최신 결과다. 아래 문서의 endpoint fail-closed step 25 및 strict IK 분류는 변경 전 비교 근거로만 유지한다.

> **2026-11-13 최신 갱신:** Default pendant+5° host margin과 matched15 branch 연속 기반 σmin 0.0803으로 replay를 다시 실행했다. world_left 최초 stop은 step 72/sample 12 `SINGULARITY_APPROACH`, tool_right는 step 36/sample 25 `JOINT_LIMIT_MARGIN:J5`다. 최신 근거는 `matched15_branch_and_ik_timing.md`와 `controller_replay.json`이다.

## 범위

**[확인]** 모든 계산은 기존 `20261112_190404_arm_only_servo_episode` 파일을 읽는 offline replay다. ROS/실기 연결과 command 발행은 없으며 출력은 `DryRunPublisher` JSONL뿐이다. 기계 판독 원본은 `controller_replay.json`과 `controller_ik_diagnostics.json`이다.

## 1. arm-only endpoint 감시

끝 토큰 streak는 `Δp,ω` 6차원만 갱신한다. gripper endpoint는 분포 이탈 감시에서 제외하고 binary 상태가 바뀐 step만 기록한다. N=3은 여전히 **[가정: 후보]**다.

| 항목 | 변경 전 | 변경 후 |
|---|---:|---:|
| 최초 fail-closed endpoint 정지 | step 2, gripper min | **step 25, rx min** |
| arm endpoint 발생 횟수(min/max) | rx 25/0, ry 0/6, rz 0/4 | 동일 |
| 기록된 gripper 상태 변화 | 없음 | step **117, 168, 169, 170** |

gripper 값이 endpoint bin에 반복되어도 arm streak count는 0이라는 단위 테스트를 추가했다. action 값은 수정/clamp하지 않는다.

## 2. matched15 sim 평가 코드 추적

대상 evidence는 `validation/evidence/vanilla/success_profile_s1_balanced_step8130_matched15_k1_seed9406_15ep.json`이다.

번들에 보존된 실행 진입점은 `scripts/run_isaac_validation.sh` → `isaac_sim/run_oft_validation.py` → `isaac_sim/a0509_control/vla_validation.py`다. evidence config의 seed 9406, 15 episode, hold 12, K=1, continuous gripper 0.6 s, 60 Hz/5 Hz 및 결과 schema가 이 경로와 일치한다. 단, evidence에는 source commit/hash가 없어 당시 Windows staging 파일과 현재 보존본의 byte-identical 여부 자체는 **[미확인]**이다.

| 항목 | 결론 | 코드/증거 |
|---|---|---|
| Δp 좌표계 | **[확인] world** | Lula world `current_position`; `target=current+action[:3]` |
| 회전 | **[확인] world_left** | `delta ⊗ current`, 즉 `R'=Exp(ω)R` (`vla_validation.py:119-151,1023-1060`) |
| 기준점 | **[확인] tool0** | planner EE frame `tool0`; 현재/목표 pose로 Lula IK 호출 |
| 실행 시간 | **[확인] action당 0.2 s** | physics 60 Hz에서 12 step hold, policy 5 Hz; K=1 |
| arm 실행 | **[확인] direct joint target** | evidence `dataset_timed_direct_joint_target`, policy motion limits 미적용 |
| gripper | **[확인] absolute [0,1], continuous slew** | full travel 0.6 s, physics step당 최대 1/36; binary threshold 미사용 |
| 지시문 | **[확인]** red/yellow/blue | `Pick up the {color} cube.` 15개 결과에 저장 |
| image | **[확인]** RGB, JPEG Q92 | client `Image.fromarray(..., RGB)`, quality 92 |
| crop/resize | **[확인]** center crop 없음 | bottom crop 0.0; server `center_crop=False`; processor `resize-naive` 224×224 |

평가기는 model raw action을 dataset q01/q99 범위로 `np.clip`한 뒤 적용했다. 이는 평가 provenance 설명이며, v2 controller의 **action clamp 금지** 계약을 변경하지 않는다.

따라서 v2 기본 `rotation_frame=world_left`는 [가정]에서 **[확인]**으로 승격했다. sim world↔base 회전은 기존 검증대로 이 scene에서 항등이므로 동일 수치의 ω를 base Jacobian에 사용할 수 있다.

## 3. IK 비수렴 진단

실패 target/seed는 `SegmentIKError`가 sample 단위로 보존한다. DLS damping `1e-2…1e-6`, 최대 반복 `250/1000/4000`과 bounded least-squares를 사용해 action을 바꾸지 않고 도달성만 진단했다.

| 항목 | world_left | tool_right |
|---|---:|---:|
| 실패 action / segment sample | 81 / 7 | 120 / 25 |
| target tool0 xyz (m) | `[0.612455,0.051542,0.597612]` | `[0.790321,0.095266,0.403645]` |
| target quat xyzw | `[-0.132271,0.925127,-0.267492,0.234716]` | `[0.101504,-0.586474,0.736251,-0.321994]` |
| seed q (deg) | `[-168.602,-45.471,-0.554,-32.557,-102.939,-111.297]` | `[-164.290,-73.752,-1.430,-102.839,-69.116,273.880]` |
| seed→target 초기 잔차 | 0.0785 mm / 0.0256° | 0.2022 mm / 0.1212° |
| 원 DLS 실제 반복 | 17 | 44 |
| Jacobian σmin / condition | `0.001474 / 1259.5` | `0.003776 / 494.8` |
| bounded 최저 잔차 위치/회전 | **0.009934 mm / 0.0000687°** | **0.108645 mm / 0.000720°** |
| 후보 workspace box 내부 | yes | yes |
| 분류/정지 code | **도달 불가 / `IK_TARGET_UNREACHABLE`** | **도달 불가 / `IK_TARGET_UNREACHABLE`** |

URDF 한계까지 seed 여유(deg), lower/upper 순서는 다음과 같다.

- world_left: lower `[191.399,314.529,159.444,327.444,257.062,248.704]`; upper `[528.603,405.472,160.553,392.558,462.940,471.298]`
- tool_right: lower `[195.711,286.249,158.568,257.162,290.884,633.881]`; upper `[524.290,433.753,161.428,462.839,429.117,86.121]`

어느 관절도 limit에 붙지 않았고 σmin도 0이 아니다. 다만 두 seed 모두 J3≈0인 arm extension 경계 근처이며 condition number가 크다. 반복/감쇠를 바꿔도 동일한 비영 잔차 하한에 수렴했고, J3 양/음 시작 및 30개 결정론적 perturbation을 둔 추가 least-squares에서도 더 나은 해가 없었다. 따라서 원인은 관절 한계나 단순 반복 부족이 아니라 Cartesian 직선의 해당 sample이 kinematic reachable set 바깥으로 나간 **도달 불가**로 분류한다. target 근사, clamp, path 우회는 하지 않고 정지한다.

일반 정지 code는 `IK_NONCONVERGENCE`, `IK_DISCONTINUITY`, `IK_JOINT_LIMIT`, `IK_FAILURE`로 구분해 저장하고, 위 offline 도달성 진단 결과는 `IK_TARGET_UNREACHABLE`로 세분화했다.

## 4. 기존 성공 구간 회귀 결과

| 항목 | world_left | tool_right |
|---|---:|---:|
| 성공 구간 | 81 | 120 |
| 끝점 위치 오차 p99 | `9.90e-10 m` | `2.15e-10 m` |
| 끝점 회전 오차 p99 | `1.26e-10 rad` | `5.52e-11 rad` |
| overshoot / path 위반 | 0 / 0 | 0 / 0 |
| D 연장 총 횟수(구간 수) | 4 (2) | 1 (1) |
| scaling 후 후보 동역학 위반 | 0 | 0 |

기록 JointState 독립 검사에서 historical J5 incident 후보 gate 최초 진입은 기존과 동일하게 step 223, J5 `-137.909°`다. 이는 pendant 한계가 아닌 진단 후보다.

## 5. 검증

`/usr/bin/python3 -m unittest discover -s . -p 'test_*.py' -v`: **14/14 PASS**. controller 9개와 kinematics 5개이며, ROS import/발행은 없다.
