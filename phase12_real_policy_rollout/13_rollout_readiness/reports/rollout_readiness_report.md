# Readiness report

Software rollout pipeline: READY (offline/recorded only)
Mock full-pipeline: READY
OpenVLA rollout config: READY (disabled)
OFT rollout config: READY (disabled; model behavior review deferred)
Logging: READY
Metrics: READY (unobserved physical outcomes null)
Safety: READY (offline/mock validation)
Real command integration: NOT_IMPLEMENTED
Real robot validation: PENDING
Real rollout: NOT_AUTHORIZED

기존 100 tests 통과. 추가 4 tests에 두 모델 mock, 승인 게이트, readiness fail-closed,
camera/JointState/TCP/model/logger/manual-abort/underrun/workspace 및 action 크기 failure injection 포함.
Recorded audit 결과 OpenVLA 45/16 accepted, OFT 45/13 accepted.
Safety reject 시 strict runner는 중단합니다. audit-all 결과는 분석 목적입니다.
Physical task success, 실제 command latency, feedback acknowledgement는 검증하지 않았습니다.

Production safety config와 threshold는 변경하지 않았습니다.
Robot command / motion service/action / gripper / Home / trajectory / Hold/E-stop / real rollout: 모두 0.
