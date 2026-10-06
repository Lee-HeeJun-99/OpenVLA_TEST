# Phase 12 Episode 4 Safety Reject Analysis

## 1. 목적

이 분석은 recorded Episode 4에서 `SafetyPipeline`이 action을 거절한 이유를 분해한다. `accepted`는 기술적 safety gate 통과를 뜻할 뿐 task success나 실제 로봇 안전성을 뜻하지 않는다. 모든 계산은 offline이며 command capability를 사용하지 않았다.

## 2. 입력 데이터

- OpenVLA: `vanilla_s1_balanced_step8130`, `openvla_token`, K=1, 45 actions
- OFT: `oft_mixed480_step28560_merged`, `oftplus_h5_vision`, K=5, 9 chunks × 5 actions
- instruction: `Pick up the orange cube.`
- Phase 12 recorded outputs: `03_shadow_mode/recorded_shadow_episode4_{openvla,oft}_20261006.jsonl`
- position provenance: Phase 10 Episode 4의 `planned_actual_duration`; measured TCP가 아니다.
- canonical action: translation m, rotation rotvec rad, closedness `[0,1]`

기존 결과를 같은 limiter, 검사 순서, workspace, gripper state machine으로 replay했다. 결과는 OpenVLA 45/16/29, OFT 45/3/42(total/accepted/rejected)로 일치했고 action별 blocker mismatch는 0개였다.

## 3. 분석 방법

`blockers`는 현재 pipeline이 실제로 출력한 이유다. `diagnostic_blockers`는 early-return 뒤에 가려질 수 있는 raw 5 Hz 속도·가속도, raw workspace 및 gripper 조건을 독립 계산한 진단값이다. 둘을 합쳐 실제 blocker 수를 부풀리지 않았다.

Primary blocker는 현재 `CommandDisabledPolicyRuntime.evaluate()`가 blocker list를 구성하는 순서인 raw translation step → raw rotation step → `RuntimeSafetySupervisor`의 첫 실패 → workspace → gripper를 사용했다. Supervisor 내부 순서는 logger/communication/camera/JointState/TCP/model → finite/time/stale/duplicate → limited step → rate → velocity → acceleration이다.

분포의 implied velocity와 acceleration은 요청된 5 Hz, `dt=0.2 s`로 raw action sequence에서 계산했다. threshold sweep은 매 설정마다 stateful limiter, safety supervisor, workspace 및 gripper supervisor를 처음부터 replay했다. production YAML은 수정하지 않았다.

## 4. SafetyPipeline 기준

| 기준 | Production 값 |
|---|---:|
| Translation raw/limited step | 0.004 m |
| Rotation raw/limited step | 4 deg |
| Translation velocity | 20 mm/s |
| Rotation velocity | 20 deg/s |
| Translation acceleration | 20 mm/s² |
| Rotation acceleration | 20 deg/s² |
| Workspace x | [0.275, 0.554] m |
| Workspace y | [-0.355, 0.386] m |
| Workspace z | [0.267, 0.754] m |
| Gripper | ≤0.3 open, ≥0.7 close, middle hold |

실제 taxonomy는 `raw_translation_step_limit`, `raw_rotation_step_limit`, supervisor의 failure names, `workspace_violation`, `premature_gripper_close`, `gripper_<reason>`이다. 분석 CSV에는 동일 의미의 `gripper_stale_or_timeout`과 `gripper_unknown_state_blocks_command`를 canonical category `gripper_state_invalidated`로도 표시했다.

## 5. OpenVLA 결과

- total 45, accepted 16, rejected 29
- `raw_translation_step_limit`: 27 actions(전체 60.0%, reject의 93.1%)
- `premature_gripper_close`: 4 actions(전체 8.9%, reject의 13.8%)
- primary: translation 27, premature close 2
- translation norm: median 6.09 mm, p95 16.57 mm, max 21.29 mm; 4 mm 초과 27/45
- rotation norm: p95 0.83 deg, max 2.57 deg; 4 deg 초과 0/45
- raw implied translation speed max 106.43 mm/s, 20 mm/s 초과 27/45
- raw implied translation acceleration max 488.06 mm/s², 20 mm/s² 초과 15/44
- close candidate는 frame 23부터이고 첫 `grasp_close`는 frame 27이다. 즉 phase 기준 4 frames 이른 후보가 존재했다.
- actual workspace reject는 0개였으며 현재 위치가 4 mm boundary band에 있던 sample도 0개였다.

## 6. OFT 결과

- total 45, accepted 3, rejected 42
- `gripper_unknown_state_blocks_command`: 20 actions(전체 44.4%)
- `gripper_stale_or_timeout`: 19 actions(전체 42.2%)
- `raw_translation_step_limit`: 19 actions(전체 42.2%)
- `translation_acceleration_limit`: 19 actions(전체 42.2%)
- `premature_gripper_close`: 3 actions(전체 6.7%)
- primary: translation 19, unknown gripper 12, acceleration 8, premature close 3
- translation norm: median 3.39 mm, p95 12.49 mm, max 12.76 mm; 4 mm 초과 19/45
- rotation norm: p95 0.30 deg, max 0.70 deg; 4 deg 초과 0/45
- raw implied translation speed max 63.79 mm/s, 20 mm/s 초과 19/45
- raw implied translation acceleration max 296.62 mm/s², 20 mm/s² 초과 35/44
- close candidate는 첫 chunk의 frame 0에서 이미 발생하고 첫 `grasp_close` frame은 30이다. phase 밖 close candidate는 22/45다.
- actual workspace reject와 4 mm boundary-band sample은 모두 0개였다.

## 7. Blocker별 분석

Multi-blocker count 합은 reject 수보다 클 수 있다. OpenVLA의 대표 조합은 translation-only 25, premature-only 2, translation+premature 2다. OFT는 unknown-gripper-only 12, translation+acceleration+stale-gripper 11, acceleration+stale-gripper 8, translation+unknown-gripper 8, premature-only 3이다.

OFT의 `gripper_stale_or_timeout`은 모델 서버 timeout을 뜻하지 않는다. 현재 구현이 safety reject를 `fresh=false`로 gripper supervisor에 전달하면서 command knowledge를 UNKNOWN으로 무효화한 결과다. 이후 `gripper_unknown_state_blocks_command`가 연쇄 발생한다. 따라서 이 39건을 모두 독립적인 모델 gripper 오류로 세면 안 된다.

## 8. Threshold sensitivity

각 parameter만 독립적으로 변경해 전체 stateful pipeline을 replay했다.

- OpenVLA translation step 4→5/6 mm: 16 그대로; 8 mm: 29; 10 mm: 33. 비선형 변화는 다른 limiter/gripper state와의 상호작용 때문이다.
- OpenVLA rotation 완화: 증가 0. Translation acceleration 완화: 증가 0.
- OFT translation step, rotation step, translation velocity 완화: 증가 0.
- OFT translation acceleration 20→40 mm/s²: 3→12, 100 mm/s²: 3→14.

이 값은 안전 변경 권고가 아니라 `RECOMMENDATION_ONLY` 진단이다. 특히 10 mm step과 100 mm/s²는 현장 승인 없이 사용하면 안 된다.

## 9. Leave-one-blocker-out

Posthoc 결과로 OpenVLA translation blocker만 무시하면 16→41, premature-close만 무시하면 16→18이다. OFT unknown-gripper blocker만 무시하면 3→15, premature-close만 무시하면 3→6이고, translation/acceleration을 각각 단독 무시하면 중첩 blocker 때문에 증가가 0이다.

이 계산은 downstream state를 다시 만드는 causal replay가 아니므로 threshold sweep보다 낮은 수준의 counterfactual이다.

## 10. OFT K=5 분석

| Chunk index | Accepted/total | Reject rate | Mean translation | Mean closedness |
|---:|---:|---:|---:|---:|
| 0 | 2/9 | 77.8% | 2.38 mm | 0.595 |
| 1 | 1/9 | 88.9% | 3.57 mm | 0.716 |
| 2 | 0/9 | 100% | 5.88 mm | 0.793 |
| 3 | 0/9 | 100% | 7.22 mm | 0.859 |
| 4 | 0/9 | 100% | 7.34 mm | 0.908 |

뒤쪽 index에서 translation과 closedness가 함께 증가하며 index 2~4는 모두 reject됐다. 한 episode이므로 일반적인 K=5 성질로 확대 해석할 수 없다.

## 11. 모델 비교

OpenVLA 16/45와 OFT 3/45 차이는 rotation이나 workspace가 아니다. OpenVLA는 raw translation step 하나가 27/29 reject를 거의 설명한다. OFT는 raw translation 19건에 더해 acceleration 19건, safety rejection으로 인한 gripper-state invalidation 39건, phase 밖 close candidate 22건이 중첩된다. OFT 뒤쪽 chunk의 증가도 차이에 기여한다.

## 12. 결론

- OpenVLA: `MIXED`. reject의 직접 지배 요인은 4 mm translation threshold지만 p95가 16.57 mm(기준의 4.14배)이고 early close도 있어 단순히 threshold가 보수적이라고만 볼 수 없다.
- OFT: `MIXED`. acceleration을 분석상 완화하면 최대 11개가 추가 통과하지만 3→14에 그치며, K=5 후반 magnitude 증가와 조기 closedness, fail-closed gripper invalidation cascade가 함께 남는다.

## 13. 실제 rollout 전 권장 변경 여부

`RECOMMENDATION_ONLY`: production threshold는 변경하지 않는다. 우선 (1) gripper freshness가 safety rejection에 종속되어 UNKNOWN cascade를 만드는 정책을 별도 검토하고, (2) action magnitude가 checkpoint의 물리 단위/denormalization과 일치하는지 재확인하고, (3) 여러 episode에서 같은 분포가 재현되는지 확인해야 한다. 그 뒤 최소-motion validation으로 threshold의 물리적 적합성을 검증해야 한다.

## 14. 한계

- Episode 4 한 개의 recorded-input offline 분석이다.
- position은 measured TCP가 아니라 planned/commanded provenance다.
- raw implied velocity/acceleration은 5 Hz 실행 가정이며 실제 executed motion이 아니다.
- accept/reject는 success rate가 아니다.
- threshold sweep과 leave-one-out은 실제 robot safety 승인 근거가 아니다.
- Robot command, motion service/action, gripper, Home, trajectory, Hold/E-stop, real closed-loop 실행은 모두 0회다.

## 재현 명령

```bash
cd /home/ubuntu/a0509_vla_linux_field_bundle_20260903/lhj/phase12_real_policy_rollout
python 07_safety_reject_analysis/analyze_reject_reasons.py
python 07_safety_reject_analysis/analyze_threshold_sensitivity.py
python 07_safety_reject_analysis/analyze_oft_chunk_rejects.py
```

