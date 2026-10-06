# Real command boundary audit (2026-10-02)

## 결론

기존 `runtime.launch.py`는 camera, inference와 함께 `action_adapter` 및
`doosan_bridge`를 기동한다. 현재 Phase 12 계약을 검증하기 위한
command-disabled 실행에는 사용할 수 없다.

## 기존 runtime의 command capability

- `action_adapter_node.py`는 `/vla/target_pose`와 `/vla/gripper_open` publisher를
  생성하고 action callback에서 직접 publish한다.
- 기존 문서/구현은 gripper를 `0=close, 1=open`으로 기술하며 threshold도
  open `>=0.7`, close `<=0.3`이다. Phase 11 모델의 canonical closedness
  (`0=open, 1=closed`)와 반대다.
- rotation을 Doosan pose의 마지막 세 성분에 성분별로 더하는 경로이며,
  검증된 world-rotvec/Doosan-ZYZ 행렬 합성 계약과 다르다.
- `doosan_bridge_node.py`는 생성 시 GetCurrentPosx, MoveLine, MoveBlending,
  MoveStop, tool digital output client와 servol/speedl publisher를 만든다.
- `dry_run` parameter만으로 위 capability의 생성 자체가 제거되지 않는다.

## Phase 12 결정

Real runtime 원본은 수정하지 않았다. Phase 12 command-disabled 경계에서는
`ActionAdapterNode`와 `DoosanBridgeNode`를 로드하지 않는다. 검증된 변환과
안전 검사는 `CommandDisabledPolicyRuntime`에서 수행하고 최종 결과는
`HardCommandGate`의 `NullCommandSink`로만 전달한다. 이 sink에는 publish,
service/action client 또는 robot delivery API가 없다.

이 결과는 production command runtime 통합 완료를 뜻하지 않는다. 실제 명령
sink는 의도적으로 존재하지 않으며, 물리 안전 확인과 별도의 motion 승인 이후
새 Gate에서만 설계·검토할 수 있다.

