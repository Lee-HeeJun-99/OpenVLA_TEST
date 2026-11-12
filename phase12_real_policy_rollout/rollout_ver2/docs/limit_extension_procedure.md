# 펜던트 관절 한계 확장 절차

첫 실기 세션은 Default를 사용한다. 이 문서는 이후 한계 확장이 실제로 필요하다는 로그 근거가 생긴 경우에만 적용한다. v2 코드가 펜던트를 변경하거나 조그 명령을 보내지는 않는다.

## 1. 사유와 대상 확정

`controller/pendant_limits.json`의 `extension_reason`에 반복 정지 로그와 대상 관절을 기록한다. 예: “J5 host margin 정지로 정상 rollout이 반복 조기 종료됨.” 단일 실패나 편의를 이유로 확장하지 않는다. 조그 결과는 `extended_range_clearance_record`의 `date`, `operator`, `joints_checked`, `angles_checked_deg`에 함께 기록한다.

## 2. 변경 전 기록

- 현재 Robot Limits 일반/감소 화면을 촬영한다.
- 날짜, 작업자, 로봇/펌웨어, tool/TCP, cable routing을 기록한다.
- 두산 설정 가능 최대치 이내인지 offline loader로 먼저 확인한다.

## 3. 수동 저속 조그 clearance 확인

로봇 주변을 비우고 물리 정지 수단을 담당자가 잡은 상태에서, 대상 관절만 사람이 펜던트로 저속 조그한다. 그리퍼, 케이블, 카메라, 테이블 및 로봇 자체 간섭을 단계별 각도에서 확인한다. 자동 명령이나 v2 publisher는 사용하지 않는다.

기록 필수 항목:

- 날짜와 작업자
- 관절 번호
- 양 방향 확인 각도
- 조그 속도/감소 모드
- 각 확인 지점의 사진 또는 영상
- 간섭, cable tension, 이상음 여부

문제가 있으면 즉시 중단하고 Default로 복구한다.

## 4. 펜던트 변경과 사진

승인된 범위만 펜던트에서 변경하고 일반/감소 모드 화면을 다시 촬영한다. 설정 가능 최대치를 넘지 않는다. 변경 후 실제 표시값과 단위를 독립 확인한다.

## 5. 설정과 evidence 등록 순서

1. 확장된 값을 `controller/pendant_limits.json`에 입력한다.
2. `source_date`, `evidence_path`, `profile=extended`, `extension_reason`을 채운다.
3. 펜던트 화면 사진을 근거로 `pendant_limits_recorded` gate evidence를 등록한다.
4. 조그 기록 파일을 `extended_range_clearance_checked` evidence로 등록한다.
5. offline invariant tests와 replay를 재실행한다.
6. 두 gate가 모두 통과한 뒤 별도 승인된 실행 세션을 시작한다.

둘 중 하나라도 false이거나 evidence 파일이 없으면 `EXTENDED_RANGE_GATE_BLOCKED`로 실행을 거부한다. host margin은 pendant 안쪽이어야 하며 action이나 관절 목표를 clamp하지 않는다.
