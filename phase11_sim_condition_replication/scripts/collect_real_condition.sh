#!/usr/bin/env bash
set -euo pipefail

ROOT=/home/ubuntu/a0509_vla_linux_field_bundle_20260903/lhj/phase11_sim_condition_replication
BUNDLE=/home/ubuntu/a0509_vla_linux_field_bundle_20260903
COLLECTOR=/home/ubuntu/robot_ws/src/doosan-robot2/dsr_example2/dsr_example/dsr_example/simple/collect_episode001_from_align_home.py
LOG="$ROOT/real_collection_work_log.txt"

if [[ $# -lt 2 || $# -gt 3 ]]; then
  echo "Usage: $0 {lighting_low|extra_object|distractor_swap} {1..5} [--execute]" >&2
  exit 2
fi

condition=$1
number=$2
execute=${3:-}
if [[ ! "$condition" =~ ^(lighting_low|extra_object|distractor_swap)$ ]]; then
  echo "Invalid condition: $condition" >&2
  exit 2
fi
if [[ ! "$number" =~ ^[1-5]$ ]]; then
  echo "Episode must be 1..5: $number" >&2
  exit 2
fi
if [[ -n "$execute" && "$execute" != "--execute" ]]; then
  echo "Third argument must be --execute or omitted" >&2
  exit 2
fi

id=$(printf '%06d' "$number")
source_episode="$BUNDLE/data/real_world/raw_dataset_oft/episodes/episode_$id"
source_layout=$(printf '%s/outputs/sim2real_analysis/trajectory%03d_manual_layout.json' "$BUNDLE" "$number")
layout="$source_layout"
if [[ "$condition" == "distractor_swap" ]]; then
  layout="$ROOT/dataset/generated/episode_${id}_distractor_swap_layout.json"
fi
output="$ROOT/real_dataset/$condition/episodes/episode_$id"

free_kb=$(df --output=avail -k "$ROOT" | tail -1 | tr -d ' ')
if (( free_kb < 20 * 1024 * 1024 )); then
  echo "BLOCKED: less than 20 GiB free" >&2
  exit 3
fi
if [[ -e "$output" ]]; then
  echo "Refusing to replace existing output: $output" >&2
  exit 4
fi

mkdir -p "$(dirname "$output")"
set +u
source /opt/ros/humble/setup.bash
source /home/ubuntu/robot_ws/install/setup.bash
set -u

cmd=(
  /usr/bin/python3 "$COLLECTOR"
  --episode-dir "$output"
  --reference-steps-jsonl "$source_episode/steps.jsonl"
  --layout-json "$layout"
  --instruction "Pick up the orange cube."
  --replay-mode tcp-z-offset
  --record-hz 5.0
  --skip-sim-replay
)
if [[ "$execute" == "--execute" ]]; then
  cmd+=(--execute)
fi

{
  echo
  echo "[$(date --iso-8601=seconds)] condition=$condition base_episode=$id mode=${execute:---dry-run}"
  echo "source_episode=$source_episode"
  echo "layout=$layout"
  echo "output=$output"
  echo "free_kb_before=$free_kb"
  printf 'command='; printf '%q ' "${cmd[@]}"; echo
} | tee -a "$LOG"

cd "$(dirname "$COLLECTOR")"
"${cmd[@]}" 2>&1 | tee -a "$LOG"

if [[ "$execute" == "--execute" ]]; then
  expected_steps=$(grep -cve '^[[:space:]]*$' "$source_episode/steps.jsonl")
  actual_steps=0
  if [[ -f "$output/steps.jsonl" ]]; then
    actual_steps=$(grep -cve '^[[:space:]]*$' "$output/steps.jsonl")
  fi
  echo "postcheck_expected_steps=$expected_steps postcheck_actual_steps=$actual_steps" | tee -a "$LOG"
  if [[ "$actual_steps" -ne "$expected_steps" ]]; then
    echo "INVALID_INCOMPLETE_EPISODE: step count mismatch; preserve for audit and do not analyze" | tee -a "$LOG" >&2
    exit 5
  fi
fi

echo "[$(date --iso-8601=seconds)] finished condition=$condition base_episode=$id" | tee -a "$LOG"
