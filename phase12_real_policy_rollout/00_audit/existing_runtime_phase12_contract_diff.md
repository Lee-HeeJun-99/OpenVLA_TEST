# Existing Real runtime vs Phase 12 contract

기준일: 2026-10-06. 기존 runtime은 읽기만 했으며 수정하지 않았다.

| 항목 | Existing runtime | Phase 12 safe boundary |
|---|---|---|
| Command isolation | `runtime.launch.py`가 ActionAdapter와 DoosanBridge를 함께 실행 | 두 node 제외, NullCommandSink만 생성 |
| Gripper | `0=close, 1=open`, open ≥0.7, close ≤0.3 | `0=open, 1=closed`, 0.3/0.7 hysteresis, measured=null |
| Translation | runtime yaml 2800, source default 1000으로 설정도 불일치 | canonical meter, boundary 1000 mm/m, gain 금지 |
| Rotation | rotvec 성분을 degree A/B/C delta처럼 적용 | `R_next=Exp(rotvec)@R_current`, matrix→Doosan ZYZ |
| OFT | legacy proprio step6000; K=5 command 전개 계약 없음 | Vision step28560, no proprio, sequential K=5 |
| Preprocessing | `pick up the cube`, center crop false | `Pick up the orange cube.`, Phase 11 preprocessing |
| TCP | 불안정한 `get_current_posx` client 생성 | measured/FK/recorded/unavailable abstraction |
| Safety | command node 내부의 개별 clip/workspace | canonical action 뒤 통합 fail-closed pipeline |
| Sink | motion/IO clients와 servol/speedl publisher | Null/in-memory mock만; Real skeleton은 생성 거부 |

기존 `runtime.launch.py`, ActionAdapter, DoosanBridge 및 runtime yaml은 Phase 12
rollout 경로로 승인되지 않는다. 향후 real sink는 Phase 12 boundary 뒤에서 별도
구현하고 minimum-motion 검증을 통과해야 한다.

