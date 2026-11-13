# Camera QoS 진단 (2026-11-13)

## 안전 범위와 조건

- **[확인]** `/zed/zed_node/rgb/color/rect/image` 구독만 수행했으며 publisher와 서비스 client를 만들지 않았다.
- **[확인]** 각 QoS를 순서대로 20초 측정했다. 입력은 1280×720 `bgra8`, nominal 30 Hz다.
- 측정 원본: `camera_qos_diag.json`; 측정 도구: `tools/camera_qos_diag.py`.

## 측정 결과

| 구독 QoS | 프레임 | 수신률 | 30 Hz 대비 | 간격 p50 / p99 / max (ms) | BGRA→RGB 포함 callback p50 / p99 / max (ms) | 간격 >0.1 s |
|---|---:|---:|---:|---:|---:|---:|
| sensor_data BEST_EFFORT | 299 | 14.941 Hz | 49.80% | 38.444 / 207.187 / 433.738 | 6.353 / 10.841 / 12.757 | 52 |
| RELIABLE, depth 1, VOLATILE | 400 | 19.977 Hz | 66.59% | 33.920 / 166.008 / 168.470 | 6.319 / 11.479 / 14.941 | 31 |

RELIABLE depth 1이 수신률과 장시간 공백 횟수 모두 우세하므로 v2 기본으로 채택했다. 다만 이 측정기의 callback은 요청대로 BGRA→RGB 변환을 포함하므로, DDS 수신과 약 6 ms 변환 비용이 직렬화되어 두 경우 모두 nominal 30 Hz에 미달했다. 따라서 QoS 변경만으로 해결했다고 보지 않는다.

## 원인 진단과 실행 구조

- **[확인(code)]** 수정 전 `tools/dryrun_live.py`는 feedback과 camera를 같은 node에 두고 메인 100 Hz loop에서 `rclpy.spin_once(node, timeout_sec=0)`를 한 번 호출하는 단일-thread 구조였다. preflight에서도 같은 node를 `spin_once(..., 0.01)`했다. 수정 후 근거는 `tools/dryrun_live.py:31-42,55-66`이다.
- **[확인(code)]** 수정 전 camera callback은 매 3.7 MB BGRA message를 즉시 RGB contiguous array로 변환했다. 측정상 이 작업은 p50 약 6.3 ms였다. 수정 후 최소 callback과 지연 변환 근거는 `runtime/camera.py:20-27,34-45,56-67`이다.
- **[확인(code)]** 수정 전 명시적 post-settle frame barrier가 없었다. 준비 loop 뒤 곧바로 `latest()`를 호출해 시작 직후 age 기반 `CAMERA_STALE`이 발생할 수 있었다. 현재는 `runtime/camera.py`의 `frame_after/poll_frame_after`와 `tools/dryrun_live.py`의 모드 A 상태기계가 host/ROS 두 시각 모두 정착 뒤인 frame을 기다린다.
- **결론:** QoS 불일치 자체는 호환되지만 BEST_EFFORT에서 손실이 더 컸고, 큰 image의 callback 변환과 메인 loop의 단일-thread spin이 camera callback scheduling을 악화시킨 것이 주원인이다.

## 적용한 수정

- RELIABLE / KEEP_LAST depth 1 / VOLATILE 채택.
- camera 전용 node + `SingleThreadedExecutor` 전용 thread로 feedback/main loop와 분리.
- callback은 raw message reference, host 수신 monotonic, header stamp, 직전 수신 시각, 누적 count만 저장한다. BGRA→RGB는 `latest()` 호출 thread에서 수행한다.
- 첫 frame을 최대 5 s **후보** 동안 기다리고 초과 시 `CAMERA_NO_FIRST_FRAME`으로 중단한다.
- camera의 0.1 s 기준은 fault가 아니라 gap 통계 기준으로만 유지한다. 대기 상한 10 s 후보를 넘으면 `CAMERA_NO_FRAME_TIMEOUT` 정상 episode 종료로 기록한다.
- 진단에는 직전 간격, 마지막 host 수신 시각, header stamp, 누적 수신 수, 0.1 s/1 s 초과 횟수와 최대 간격을 남긴다.

## 수정 후 최소 callback 구조 재측정

2026-11-13에 RELIABLE / depth 1 / VOLATILE, 전용 node·executor thread, callback 내 색 변환 없음 조건으로 다시 20초 측정했다. 원본은 `camera_qos_diag_modified.json`이다.

| 구조 | 프레임 | 수신률 | 30 Hz 대비 | 간격 p50 / p99 / max (ms) | 최소 callback p50 / p99 / max (ms) | >0.1 s | >1 s |
|---|---:|---:|---:|---:|---:|---:|---:|
| 수정 후 RELIABLE depth 1 | 457 | 22.843 Hz | 76.14% | 33.350 / 166.573 / 238.502 | 0.0029 / 0.0071 / 0.0112 | 40 | 0 |

색 변환 제거로 callback 시간과 평균 수신률은 개선됐지만 0.1초 공백은 없어지지 않았다. 측정 순서와 시스템 부하가 다른 단일 20초 구간이므로 이전 31회와 횟수를 직접 성능 회귀로 해석하지 않는다. 1초 초과 공백은 없었다. 모드 A에서는 이 간격을 `log_only`로 기록하고, 정착 후 조건을 만족하는 새 프레임을 기다린다.
