# matched15 branch 연속 복원과 IK 사전 계산 시간

모든 작업은 `/usr/bin/python3` offline 계산이며 실기·ROS 접속이나 명령 발행은 없다.

## Branch 연속 복원

각 episode 첫 `tool0_sim` pose는 실제 `a0509_home_pose.json`의 sim home을 seed로 풀고, 이후 pose는 직전 해를 seed로 사용했다. world→base 변환은 `kinematics_validation_2`의 wxyz 양쪽 변환 적합값을 사용했다. 총 1,053 pose 모두 위치 0.05 mm/회전 0.005° 기준을 통과했다. 최대 잔차는 위치 `1.15e-11 m`, 회전 `9.17e-11°`다.

- J5 전 sample 음수: `[-107.061,-73.925]°`.
- J6는 연속적인 음수 branch: `[-288.773,-178.292]°`.
- Isaac 225 sim-basis J6 envelope `[-214.441,-178.500]°`와 겹치지만, matched15의 31.72%만 그 범위 안에 있고 더 음의 영역으로 연속 확장된다. 운영 `sim_joint_envelope`는 이 결과를 실기 기준으로 변환한 뒤 Isaac225와 관절별 합집합으로 갱신했으며 계속 `log_only`다. **출처:** `controller/controller_config.json`, `reports/joint_limits_analysis.json`.
- 따라서 wrist sign/연속 branch는 일치하지만 관측 envelope가 동일하다는 뜻은 아니다. envelope 초과는 기존 설계대로 log-only다.

## Isaac 225 비교

| joint | matched15 min/max° | Isaac225 min/max° |
|---|---:|---:|
| J1 | -197.812 / -157.562 | -217.630 / -182.536 |
| J2 | -41.210 / 20.895 | -34.953 / 2.155 |
| J3 | -88.883 / -37.518 | -99.657 / -40.984 |
| J4 | -3.441 / 5.642 | 0.141 / 5.926 |
| J5 | -107.061 / -73.925 | -113.506 / -68.638 |
| J6 | -288.773 / -178.292 | -214.441 / -178.500 |

| σmin | matched15 branch 연속 | Isaac225 실제 joint |
|---|---:|---:|
| p0 | 0.100446 | 0.109456 |
| p1 | 0.104796 | 0.109456 |
| p5 | 0.110880 | 0.109456 |
| p50 | 0.156459 | 0.177201 |

새 σmin 후보는 두 자료 중 더 낮은 matched15 p0의 약 80%인 **0.0803**이다. 과거 잘못된 branch 복원의 0.08 근거와 Isaac225만 사용한 0.0875 후보는 폐기한다.

## 겹치는 pose 검사

world `tool0_sim` 위치 1 mm 이하이면서 방향 0.1° 이하를 겹침으로 정의했다. 해당 pair는 **0개**여서 관절 오차를 보고할 수 없다. 가장 가까운 결합 후보도 위치 96.86 mm, 방향 10.25° 차이여서 동일 pose로 간주하지 않았다. 따라서 실제 관절과의 직접 paired joint-error 검증은 불가능하며 branch 판정은 seed 연속성, J5 부호, J6 연속 범위에 근거한다.

## IK 실행 구조

`SegmentGenerator.generate()`는 다음을 발행 전에 완료한다.

1. 전체 100 Hz pose 배열 생성: `controller/trajectory.py:52-59`.
2. 직전 sample 해를 seed로 모든 sample IK 계산: `:61-78`.
3. 전체 관절 속도·가속도 계산 및 D 연장 판단: `:80-102`.
4. 최종 segment의 모든 sample 관절 margin·특이점 검사: `:98-106` 부근.
5. 모두 통과해야 `Segment`를 반환하며 그 뒤에만 publisher가 호출된다. replay 근거는 `controller/replay.py:96-127,181-184`다.

전체 시퀀스 진단을 위한 offline replay만 `enforce_full_preflight=False`를 명시해 정지 이후 데이터도 계산한다. command-capable 기본값은 `True`이며 실패 시 `SegmentPreflightError`로 segment 자체가 반환되지 않는다.

## 계산 시간

동일 PC에서 190404 replay의 성공 segment를 `perf_counter`로 측정했다. 구간 시간에는 pose 생성, 모든 IK, 동역학 계산과 D 재계산이 포함된다.

| frame | samples/segments | sample IK p50/p99/max | segment precompute p50/p99/max |
|---|---:|---:|---:|
| world_left | 4191 / 81 | 1.143 / 1.776 / 2.844 ms | 58.61 / 196.81 / 396.75 ms |
| tool_right | 2499 / 49 | 0.975 / 1.709 / 2.717 ms | 58.03 / 64.01 / 64.78 ms |

matched15 branch 복원은 bounded nonlinear least-squares 진단이라 runtime DLS보다 느리며 최종 측정 sample p50/p99/max는 약 7.00/10.91/14.78 ms였다. 이는 runtime segment timing으로 사용하지 않는다. 시간값은 비실시간 OS의 단일 offline 실행 측정이다.

기계 판독 결과는 `matched15_branch_reconstruction.json`과 `controller_replay.json`에 있다.
