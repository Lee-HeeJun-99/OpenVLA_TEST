#!/usr/bin/env bash
set -euo pipefail
BUNDLE=/home/ubuntu/a0509_vla_linux_field_bundle_20260903
ANALYSIS="$BUNDLE/lhj/phase11_sim_condition_replication/policy_sensitive_analysis"
PY="$BUNDLE/environment/a6000_ubuntu22_py310/bin/python"
EXTRACT="$BUNDLE/sim2real_analysis/04_features/extract_vla_features.py"
MODEL="${1:-}"
[[ "$MODEL" == openvla || "$MODEL" == oft ]] || { echo "Usage: $0 <openvla|oft>" >&2; exit 2; }
avail_kb=$(df --output=avail /home/ubuntu | tail -1)
(( avail_kb >= 20*1024*1024 )) || { echo "Less than 20 GiB free; refusing feature extraction" >&2; exit 3; }
"$PY" "$ANALYSIS/scripts/prepare_feature_inputs.py"
export PYTHONPATH="$BUNDLE:${PYTHONPATH:-}"
for condition in baseline lighting_low extra_object distractor_swap; do
  dst="$ANALYSIS/02_features/$MODEL/$condition"
  [[ -e "$dst/feature_manifest.json" ]] && { echo "skip completed $MODEL/$condition"; continue; }
  mapfile -t images < "$ANALYSIS/02_features/inputs/${condition}_images.txt"
  args=(--image-input "${images[@]}" --domain real --label "phase11_${condition}" --output-dir "$dst" --max-samples "${#images[@]}" --full --instruction 'Pick up the orange cube.' --local-files-only)
  if [[ "$MODEL" == openvla ]]; then
    "$PY" "$EXTRACT" --model vanilla "${args[@]}" --checkpoint "$BUNDLE/models/vanilla_s1_balanced_step8130"
  else
    "$PY" "$EXTRACT" --model oft "${args[@]}" --checkpoint "$BUNDLE/runtime_state/oft_mixed480_step28560_merged" --variant oftplus_h5_vision
  fi
  avail_kb=$(df --output=avail /home/ubuntu | tail -1)
  (( avail_kb >= 20*1024*1024 )) || { echo "Space guard triggered after $MODEL/$condition" >&2; exit 3; }
done
