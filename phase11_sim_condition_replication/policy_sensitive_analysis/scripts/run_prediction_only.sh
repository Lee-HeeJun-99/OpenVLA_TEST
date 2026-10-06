#!/usr/bin/env bash
set -euo pipefail
ROOT=/home/ubuntu/a0509_vla_linux_field_bundle_20260903/lhj/phase11_sim_condition_replication
BUNDLE=/home/ubuntu/a0509_vla_linux_field_bundle_20260903
P10=/home/ubuntu/a0509_vla_linux_field_bundle_20260903/lhj/phase10_planner_based_shadow_mode
PY=/home/ubuntu/a0509_vla_linux_field_bundle_20260903/environment/a6000_ubuntu22_py310/bin/python
OUT="$ROOT/policy_sensitive_analysis/01_predictions"
MODEL="${1:-}"
RUN_NAME="${2:-$MODEL}"

if [[ "$MODEL" != "openvla" && "$MODEL" != "oft" ]]; then
  echo "Usage: $0 <openvla|oft>" >&2
  echo "Run one model server at a time to limit GPU memory use." >&2
  exit 2
fi

server_args=()
if [[ "$MODEL" == "openvla" ]]; then
  server_args=(--openvla-server-url http://127.0.0.1:8766)
  "$BUNDLE/scripts/healthcheck.sh" vanilla http://127.0.0.1:8766
else
  server_args=(--oft-server-url http://127.0.0.1:8765)
  "$BUNDLE/scripts/healthcheck.sh" oft http://127.0.0.1:8765
fi

# Prediction only: this runner has no ROS import, publisher, service or action client.
for condition in baseline lighting_low extra_object distractor_swap; do
  for n in 1 2 3 4 5; do
    eid=$(printf 'episode_%06d' "$n")
    ep="$ROOT/real_dataset/$condition/episodes/$eid"
    dst="$OUT/$RUN_NAME/$condition/$eid"
    [[ -e "$dst/integrated_log.jsonl" ]] && { echo "skip $condition/$eid"; continue; }
    "$PY" "$P10/06_shadow_collection/shadow_mode_runner.py" \
      --episode-dir "$ep" --output-dir "$dst" \
      --trial-id "phase11_${condition}_${eid}" --condition-id "$condition" \
      --layout-id "$eid" "${server_args[@]}"
    "$PY" "$ROOT/policy_sensitive_analysis/scripts/validate_prediction_output.py" \
      --model "$MODEL" --samples "$dst/samples.jsonl"
  done
done
