# dryrun process stall 분석

## 관찰 사실

- dryrun duration: 50.352 s
- stale 판정: host monotonic `136141.431210824`
- 마지막 JointState 수신: `136141.321279905`
- joint age: 109.931 ms, 직전 joint 간격: 9.906 ms
- 해당 시점 누적 JointState: 5,013개, robot_state age: 204.274 ms
- camera gap >0.1 s: 72회, 최대 0.30 s(사용자 제공 실행 결과)
- 99개 관측 이전까지 inference가 성공했고 command publish/state-changing call은 모두 0이다.

## prediction timeline 대조

| step | frame host 수신 | prediction 완료 | frame→완료 |
|---:|---:|---:|---:|
| 97 | 136140.287924 | 136140.729122 | 441.197 ms |
| 98 | 136140.785423 | 136141.208530 | 423.106 ms |
| 99 | 136141.258229 | 136141.678828 | 420.599 ms |

JointState 무응답 구간 `136141.321280–136141.431211`은 step 99의 전체 prediction 처리 창 `136141.258229–136141.678828` 안에 완전히 포함된다. 따라서 무거운 관측 처리와 시간상 겹친다는 가설은 지지된다.

기존 `predictions.jsonl`은 frame 수신과 전체 prediction 완료만 기록하고 JPEG 전처리 시작/종료, 파일 쓰기, HTTP 시작/종료를 분리하지 않았다. 그러므로 이번 자료만으로 BGRA→RGB, JPEG, 파일 I/O, HTTP/base64/JSON 중 특정 단계가 직접 원인이라고 확정할 수 없다. 새 observation process는 각 단계의 monotonic timing을 `processing_timing`에 기록한다.

## 조치

Python thread 추가로 해결하지 않고 image/JPEG/HTTP 전체를 별도 OS process B로 이동했다. ROS feedback과 안전 감시는 process A에 남기므로 B의 GIL 보유와 Python heap/GC pause가 A의 0.1 s freshness 감시를 멈추지 않는다. 기준값은 변경하지 않았다.
