# Command-disabled end-to-end dry-run

실행일: 2026-10-02

저장된 live stationary prediction을 다음 경로로 재생했다.

```text
OpenVLA K=1 / OFT K=5 saved prediction
→ canonical action
→ rate/velocity/acceleration limiter
→ workspace and runtime safety inspection
→ gripper hysteresis and premature-close gate
→ target-pose candidate calculation
→ HardCommandGate
→ NullCommandSink
```

결과 파일:

- `command_disabled_boundary_dry_run_20261002.jsonl`
- `command_disabled_boundary_dry_run_20261002.jsonl.status.json`

결과는 6개 candidate 전부 delivery blocked이며 실제 command capability는
정적·동적으로 0이다. 이 실행 중 ROS command topic publish, motion service,
gripper, Home, trajectory, Hold/E-stop 또는 closed-loop는 실행하지 않았다.

