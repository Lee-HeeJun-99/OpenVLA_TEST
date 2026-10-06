# 실제 로봇에서만 남은 검증

1. Hardware E-stop 접근성과 실제 차단 확인
2. Protective-stop, servo state, robot mode, alarm 확인
3. Hold acknowledgement 확인
4. JointState 60초 연속성과 name-based reorder 재확인
5. getter 없는 FK TCP 또는 안정적인 measured TCP 확인
6. Gripper 실제 polarity와 단일 open/close cycle 확인
7. 실제 workspace와 collision clearance 확인
8. Gripper 없는 minimum translation 1회
9. Minimum rotation 1회 및 matrix-equivalent pose 확인
10. Gripper single-cycle test
11. Stationary live prediction-only Shadow
12. Scripted-reference-controlled Shadow
13. Closed-loop prototype: 모델/시작 위치별 1회
14. 안전 검토 후에만 잔여 rollout

각 motion 단계는 별도 명시적 승인이 필요하다. 현재 Home, gripper, trajectory,
Hold/E-stop 및 AI closed-loop는 승인되지 않았다.

