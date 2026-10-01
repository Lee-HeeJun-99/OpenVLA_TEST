# Phase 11 Sim condition replication dataset

Episode 1–5의 기존 Sim replay를 baseline으로 참조하고, 동일 joint trajectory에
`lighting_low`, `extra_object`, `distractor_swap` 조건을 각각 적용한다.

- 총 비교 단위: 5 trajectory × 4 condition = 20 episode
- 기존 baseline: 5 episode를 `dataset/episodes/baseline`으로 복사
- 신규 수집: 15 episode
- 저장 범위: episode 이미지, `frames.jsonl`, replay report JSON, 조건 JSON
- feature/model output: 생성하지 않음
- distractor 조건은 yellow/blue만 교환하고 orange/red target은 고정

준비 및 dry-run:

```bash
PY=/home/ubuntu/a0509_vla_linux_field_bundle_20260903/environment/a6000_ubuntu22_py310/bin/python
$PY scripts/prepare_dataset.py
$PY scripts/run_collection.py
$PY scripts/validate_dataset.py
```

실제 Sim 수집은 실행 중인 Isaac GUI를 종료한 뒤 명시적으로 실행한다.

```bash
$PY scripts/run_collection.py --execute
```

한 조건이나 episode만 재개할 수 있다.

```bash
$PY scripts/run_collection.py --execute --condition lighting_low --episode 1
```

수집기는 episode마다 남은 공간을 검사한다. 20 GB 미만이면 즉시 중단하고,
기존 결과나 불완전 결과는 자동 덮어쓰지 않는다.
