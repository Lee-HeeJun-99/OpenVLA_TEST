# FK/IK 오프라인 검증 보고서

## 범위와 재현 방법

실기 연결, ROS node 실행, command publish 없이 기존 파일만 읽었다. Python은 `/usr/bin/python3` 3.10만 사용했다.

```bash
cd /home/ubuntu/a0509_vla_linux_field_bundle_20260903/lhj/phase12_real_policy_rollout/rollout_ver2
PYTHONPATH=. /usr/bin/python3 -m unittest -v kinematics.test_solver
PYTHONPATH=. /usr/bin/python3 kinematics/validate_offline.py --output reports/kinematics_validation.json
```

- URDF: `/home/ubuntu/robot_ws/src/doosan-robot2/dsr_description2/urdf/a0509.urdf`
- chain: `base_link → link_6`, 6-DOF
- 실기 log root: `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/lhj/phase12_real_policy_rollout/14_real_rollout_integration/real_trials`
- sim root: `/home/ubuntu/robot_ws/src/doosan-robot2/raw_dataset_oft`
- 전체 입력 파일 목록과 sample 수: `reports/kinematics_validation.json`

## (a) 기존 실기 로그: JointState FK 대 controller pose

paired `f_current_pos_world[1]`(flange)을 저장한 로그는 찾지 못했다. 따라서 `episode_records.json` 16개에서 `jointstate_before`와 거의 같은 시각의 controller `observation.tcp_m_abc`(current posx)를 사용했다. 원 로그가 이를 `CONTROLLER_ZERO_OFFSET_ENGINEERING_ACCEPTED`로 표시하지만, pendant tool/TCP 설정은 확인되지 않았으므로 **flange 동등성은 가정**이다. controller 자세는 base 기준 m + intrinsic ZYZ degree로 해석했다. JointState는 이름으로 `joint_1…joint_6` 순서로 재배열했다.

| 항목 | n | p50 | p99 | max | 목표 | 판정 |
|---|---:|---:|---:|---:|---:|---|
| 위치 오차 (mm) | 487 | 0.875 | 1.238 | 1.254 | 1.0 | p50 통과, p99/max 실패 |
| 회전 오차 (deg) | 487 | 0.186 | 0.231 | 0.235 | 0.1 | 실패 |
| pose–JointState receive 시각차 (ms) | 487 | 9.425 | 44.650 | 58.458 | — | 동시 측정 아님 |

결론: 이 자료는 대략적인 일치를 보이지만 요청 목표 1 mm / 0.1°를 입증하지 못한다. 특히 controller source timestamp가 없고 pose–joint pairing gap이 있으므로 동적 구간의 오차에는 시간 정렬 오차가 섞인다. URDF–실기 일치 항목은 닫지 않는다.

## (b) sim joint_position_rad 대 end_effector_pose(tool0)

26개 episode, 1,734 sample을 사용했다. `joint_position_rad[:6]`로 `link_6` FK를 구하고 기록 필드 명칭대로 `quaternion_wxyz`를 해석했다. 매 sample에서 `T_link6^-1 T_recorded`를 구한 뒤 quaternion 평균/translation 평균으로 하나의 상수 offset을 적합했다.

| 항목 | 값 |
|---|---|
| 적합 offset xyz (m) | `[-0.0011016, 0.0014579, 0.0006806]` |
| 적합 offset rpy xyz (deg) | `[36.1696, -19.5443, -44.2364]` |
| 보정 후 위치 residual p50 / p99 / max (mm) | 6.818 / 68.622 / 72.305 |
| 보정 후 회전 residual p50 / p99 / max (deg) | 24.771 / 164.411 / 164.415 |

결론: residual이 매우 커서 이 평균 transform을 실제 offset으로 채택할 수 없다. `tool0=link_6, offset 0`도 확인되지 않았으며, 단일 상수 offset 가설 자체가 이 데이터/해석에서는 성립하지 않는다. 후보 원인은 pose/quaternion convention, Isaac asset과 URDF joint convention, pose–joint 시각 정렬이며 어느 하나로 단정하지 않는다. action이나 pose를 보정/clamp하지 않았다.

## (c) IK 단위 테스트

| 테스트 | 결과 | 확인 내용 |
|---|---|---|
| deg↔rad 왕복 | PASS | 단위 utility |
| FK→IK 왕복 + seed locality | PASS | 위치/회전 2e-6 이하, seed 인접 branch |
| J5≈0 wrist singularity 근처 | PASS | damping 적용, pose residual 5e-6 이하 |
| URDF joint limit 근처 | PASS | 공급된 범위 밖 해를 사용하지 않음 |
| 과대 joint 변화 | PASS | solution clamp 없이 `IKJointChangeError` |

총 5개 test, 5 PASS (`Ran 5 tests in 0.028s`, 2026-11-12 실행). 테스트의 40° 허용값은 수치 test fixture일 뿐 실기 안전 한계 후보가 아니다. 실기에서는 확인된 pendant limit−margin과 별도 `max_joint_change_rad`를 반드시 호출자가 공급해야 한다.

## 구현 판단

- FK와 Jacobian은 PyKDL을 사용한다. `/usr/bin/python3`에 `kdl_parser_py` Python module이 없어 URDF XML의 serial chain을 읽어 PyKDL `Chain`을 구성했다.
- IK는 이전 명령 joint를 seed로 한 local DLS다. world/base frame translation 및 rotation-vector residual을 사용하고, 수렴 실패·공급 limit 위반·seed 대비 과대 변화는 예외다.
- line search는 수치 solver의 수렴 절차이며 출력 action clamp가 아니다. 허용 범위 밖 candidate는 채택하지 않고 실패한다.
- pendant limit과 안전 margin은 미확인이므로 코드에 임의값을 넣지 않았다. URDF limit은 test/model 정보일 뿐 실기 승인 한계로 간주하지 않는다.
