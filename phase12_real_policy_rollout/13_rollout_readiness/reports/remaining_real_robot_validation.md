# 실제 로봇 검증

1. E-stop / protective stop 확인
2. robot mode / servo 상태 확인
3. JointState live validation
4. measured TCP 또는 승인된 FK source 확인
5. gripper 실제 polarity 확인
6. 실제 workspace 및 기존 속도·가속도 제한 확인
7. 별도 승인 후 minimum-motion test
8. 별도 승인 후 short-horizon rollout
9. 별도 승인 후 full-task rollout

선행 software integration 제한: 실제 Doosan sink는 구현되지 않았습니다.
현장 validation 전에 검토된 실제 command/acknowledgement/stop 경계 연결이 필요합니다.
현재 command-disabled build로 위 motion 단계들을 실행할 수 없습니다.
OFT 모델 행동 검토도 선행합니다.
