# Kinematics validation 2: 실기 정지구간 보강과 sim 원인 진단

## 범위와 방법

**[확인]** 실기 실행, ROS node 실행, robot command 발행 없이 기존 로그와 source만 읽었다. 계산은 `/usr/bin/python3` 3.10으로 수행했다. 재현 코드는 `kinematics/validate_diagnostics.py`, 전체 수치와 입력 경로는 `reports/kinematics_validation_2.json`에 있다.

```bash
cd /home/ubuntu/a0509_vla_linux_field_bundle_20260903/lhj/phase12_real_policy_rollout/rollout_ver2
PYTHONPATH=. /usr/bin/python3 kinematics/validate_diagnostics.py
```

## [a] 실기 로그 보강

### a-1. 정지 구간 절대 오차: bias와 변동 분리

servo episode 9개에서 controller getter의 **고유** `latest_controller_feedback` 766개를 추출하고 각 feedback 수신시각에 가장 가까운 JointState를 연결했다. 중복 삽입된 동일 feedback timestamp는 한 번만 사용했다. 정지 분석 선택 조건은 `max(abs(current_velj[0:6])) ≤ 1e-6 rad/s`이며 11쌍이 남았다. 이 값은 데이터 선택용 수치로서 실기 안전 한계 후보가 아니다.

오차 정의는 `posx - FK`이고 회전 벡터는 `Log(R_posx R_FK^T)`의 world/base 표현이다.

| 절대 오차 norm | n | p50 | p99 | max |
|---|---:|---:|---:|---:|
| 위치 (mm) | 11 | 1.112 | 1.120 | 1.121 |
| 회전 (deg) | 11 | 0.178 | 0.206 | 0.208 |

| 벡터 오차 | bias/평균 `[x,y,z]` | std `[x,y,z]` |
|---|---|---|
| 위치 (mm) | `[+0.471, -0.574, -0.807]` | `[0.036, 0.018, 0.057]` |
| 회전 rotvec (deg) | `[-0.103, +0.013, -0.147]` | `[0.016, 0.008, 0.003]` |

**해석:** 변동보다 일정 bias가 지배적이다. 그러나 기록 pose는 paired `f_current_pos_world[1]`가 아니라 current `posx`이고 pendant TCP 설정이 미확인이므로, 이 bias를 URDF 보정값으로 적용하지 않는다. 위치/회전 목표 1 mm/0.1°도 통과하지 못했다.

### a-2. 0.2 s Δ 일치도

| 항목 | n | p50 | p99 | max |
|---|---:|---:|---:|---:|
| `Δp_posx-Δp_FK` norm (mm) | 0 | N/A | N/A | N/A |
| `Log(ΔR_posx ΔR_FK^-1)` norm (deg) | 0 | N/A | N/A | N/A |

**[미확인]** 유효 통계를 계산할 수 없었다. 고유 controller measurement의 연속 간격은 런별 최소 약 0.351 s였고 대부분 0.38–0.48 s다. servo log의 20 Hz 행에는 같은 getter 결과가 반복되어 있으며, 이를 새로운 0.2 s 측정으로 간주하면 허위 zero/stale Δ가 된다. 분석 조건은 같은 런에서 `0.2±0.03 s`인 고유 측정쌍이었고 해당 쌍은 0개다.

필요한 추가 정보는 controller source timestamp를 포함한 `posx` 또는 `f_current_pos_world[1]`를 5 Hz 이상으로 JointState와 동시 기록한 **오프라인 로그**다. 본 작업에서는 새 실기 기록을 만들지 않았다.

### a-3. JointState–posx 수신시각 차이

| n | p50 (ms) | p99 (ms) | max (ms) |
|---:|---:|---:|---:|
| 766 | 3.641 | 8.124 | 9.692 |

이는 host receive clock 차이다. controller measurement timestamp가 `null`이므로 물리적 측정 동시성 자체를 증명하지는 못한다.

## [b] sim 불일치 원인 진단

### b-1. 데이터 규약 확인

이전 검증에 사용된 후보의 실제 sample:

| 필드 | 실제 값/규약 |
|---|---|
| `joint_position_rad` | 길이 8: `[-3.186234, -0.018515, -0.715330, 0.021891, -1.981048, -0.000061, 0, 0]` |
| arm 순서 | 앞 6개가 `joint_1…joint_6`; 뒤 2개는 gripper joints |
| `joint_velocity_rad_s` | 길이 8, rad/s |
| `end_effector_pose.position_m` | 길이 3, m |
| `end_effector_pose.quaternion_wxyz` | 필드 선언은 wxyz, 길이 4 |
| `tcp_pose` | 위치 m + 회전 rad, 길이 6 |
| episode metadata | `coordinate_frame=base`, `pose_source=planned_commanded_pose`, `feedback_pose_samples=0` |

**핵심 판정:** `~/robot_ws/src/doosan-robot2/raw_dataset_oft`는 sim 수집물이 아니라 `single_robot_simple.py`가 만든 실기 portable dataset이다. `end_effector_pose`도 joint feedback과 동시 측정한 pose가 아니라 planned/commanded pose다. 이전 1,734 sample의 큰 잔차는 sim world/base 불일치의 증거가 아니었다.

실제 sim 진단에는 phase11의 Isaac joint replay 5 episode, 225 record를 사용했다. 각 record는 `actual_joint_position_rad[6]`와 `sim_end_effector_pose(position_m, quaternion_wxyz)`를 함께 가진다.

### b-2. 좌표계와 asset 코드 근거

| 조사 항목 | 코드/asset 근거 | 판정 |
|---|---|---|
| EE pose 기준 | `lula_planner.py:109-112`에서 `ArticulationKinematicsSolver.compute_end_effector_pose()` 결과를 직접 반환 | target/robot base pose가 반영된 stage world pose |
| planner base | `lula_planner.py:104-107`에서 `robot.get_world_pose()`를 RRT와 Lula solver 양쪽에 설정 | world에서 articulation base pose 사용 |
| robot root yaw | `a0509_control_app.py:1557-1574`, `/World/a0509`의 session-layer `baseYaw`; default 0° | 이번 데이터 yaw 0° |
| robot prim | `a0509_control_app.py` default `/World/a0509/base` | articulation root는 sim URDF의 `base` |
| scene/USD | `validation/a0509_project/USD/version_1/a0509_version_1_scaled_85x60.usda` 및 referenced SubUSDs | 실제 stage asset |
| planner URDF | `validation/a0509_project/urdf/a0509.urdf` | 실기 FK용 URDF와 다른 tool/base 고정 joint 포함 |
| base 고정변환 | sim URDF: `base → base_link`, xyz `[0,0,0.823528]`, rpy 0 | world→base_link translation의 주성분 |
| tool 고정변환 | sim URDF: `link_6 → tool0`, xyz `[0,0,0.01]`, rpy `[0,0,-1.5708]` | zero offset이 아님 |
| arm 관절 | descriptor `cspace=[joint_1…joint_6]`; gripper 2관절은 fixed rule | 기록 앞 6개와 일치 |

### b-3. 양쪽 상수변환 동시 추정

225개 record에 대해 `T_rec = T_world_baseLink · FK_baseLink_link6(q) · T_link6_tool0`의 좌우 6-DoF를 동시에 최소제곱 적합했다. 두 quaternion 순서를 각각 독립 실행했다.

| quaternion 해석 | 위치 residual p50/p99/max (mm) | 회전 residual p50/p99/max (deg) | 판정 |
|---|---|---|---|
| **wxyz** | `2.24e-13 / 4.90e-13 / 6.68e-13` | `1.31e-14 / 4.23e-14 / 5.11e-14` | 수치 정밀도 수준 완전 일치 |
| xyzw | `44.518 / 81.609 / 81.927` | `6.845 / 22.733 / 22.733` | 기각 |

wxyz 적합 변환:

| 변환 | xyz (m) | rpy xyz (deg) |
|---|---|---|
| `T_world_baseLink` | `[-0.0003895, -0.0002573, 0.8235262]` | `[0.00445, 0.01637, -0.00388]` |
| `T_link6_tool0` | `[0, 0, 0.0100000]` | `[0, 0, -90.0002105]` |

작은 world/baseLink xy 및 회전값은 asset의 고정값/URDF 근사(`1.571`, `-1.5708`)와 수치 적합 결과다. tool 결과는 sim URDF 선언과 직접 일치한다.

### b-4. 관절 부호/영점 탐색

wxyz 기본 적합 잔차가 이미 0이므로 추가 관절 수정은 필요하지 않다. 확인 목적으로 관절 하나씩 부호 반전한 결과는 모두 악화되었다.

| 가설 | 위치 residual p50/p99 (mm) | 회전 residual p50/p99 (deg) |
|---|---:|---:|
| J1 sign flip | 82.116 / 154.275 | 20.289 / 30.224 |
| J2 sign flip | 81.275 / 237.587 | 14.291 / 30.329 |
| J3 sign flip | 102.660 / 174.664 | 29.759 / 58.055 |
| J4 sign flip | 6.816 / 16.028 | 1.007 / 8.581 |
| J5 sign flip | 37.508 / 91.565 | 29.951 / 60.187 |
| J6 sign flip | 53.902 / 105.293 | 22.928 / 32.799 |

±90°/±180° joint offset도 탐색했다. J1/J6 offset은 각각 좌측 world transform/우측 tool transform에 흡수될 수 있는 gauge ambiguity라 잔차만으로 식별할 수 있다. 하지만 source URDF의 명시적 고정변환과 기본 q가 이미 정확히 일치하므로 offset 가설을 채택할 근거가 없다. 전체 탐색값은 JSON에 보존했다.

### b-5. 결론: action 좌표계와 `R_world_base`

**[확인]** OFT preprocessing은 `Δp=p_next-p_prev`를 기록 pose 좌표에서 계산하고, `ω=Log(q_next q_prev^-1)`를 사용한다. 즉 action translation과 rotation vector는 모두 **sim world frame** 표현이다.

**[확인]** 이번 학습/runtime scene에서 적합된 `R_world_baseLink`는 `[roll,pitch,yaw]=[0.00445°,0.01637°,-0.00388°]`, 즉 asset/수치 정밀도 안에서 항등이다. 따라서 이 scene의 action world axes와 robot `base_link` axes 사이 적용할 회전은

`R_world_base = I`.

translation origin은 약 `[−0.000389,−0.000257,0.823526] m` 떨어져 있지만 Δp와 world rotvec에는 원점 translation이 영향을 주지 않는다. runtime에 `--base-yaw-deg≠0`, 다른 USD scene 또는 다른 robot prim을 쓰면 이 결론을 재사용하면 안 되며 해당 run의 `robot.get_world_pose()`가 추가 정보로 필요하다.

## 상태 갱신

- **종료:** 현재 Isaac scene의 sim world↔`base_link` 회전, sim `link_6→tool0`, quaternion 순서(wxyz).
- **원인 확정:** 이전 sim 불일치는 실기 planned-pose dataset 오분류.
- **남음:** 실기 flange/TCP 설정과 URDF 절대 bias, controller timestamp 기반 0.2 s Δ 검증, legacy 학습 dataset 자체의 provenance/scene metadata.
