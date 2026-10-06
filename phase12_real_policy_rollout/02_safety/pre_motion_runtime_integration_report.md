# Pre-motion runtime integration report

상태: `COMMAND_DISABLED_INTEGRATION_COMPLETE`

검증된 Phase 12 경계를 하나의 `PreMotionRuntimeFacade`로 통합했다.

```text
OpenVLA K=1 / OFT sequential K=5
→ canonical action
→ stateful 20-profile limiter
→ watchdog, stale/duplicate/NaN/timeout checks
→ 1000 mm/m translation conversion
→ world-rotvec + Doosan-ZYZ matrix composition
→ operator-approved workspace
→ open-loop gripper supervisor
→ HardCommandGate
→ NullCommandSink
```

기존 command-capable `ActionAdapter`와 `DoosanBridge`는 로드하지 않는다.
Facade에는 ROS import, publisher, service/action client 또는 명령 API가 없다.
unsafe flag를 true로 바꿔 활성화할 수 있는 구조도 없으며 실제 motion 단계는
별도 구현·검토·승인이 필요하다.

## 검증 결과

- 전체 unittest: 68/68 PASS
- 실제 저장 prediction: OpenVLA K=1 + OFT K=5, 총 6 candidate
- technical valid: 4/6
- alignment premature-close 차단: 2/6
- hard-gate delivery blocked: 6/6
- executed action null: 6/6
- robot-delivered command null: 6/6
- command issued false: 6/6
- logger failure는 facade를 영구 fault 상태로 만들고 후속 처리를 차단

실제 Robot command, motion service/action, gripper, Home, trajectory,
Hold/E-stop 및 closed-loop 실행은 모두 0회다.

다음 단계는 이 결과와 별개인 **최소 motion dry-run 명시적 승인**이다.

