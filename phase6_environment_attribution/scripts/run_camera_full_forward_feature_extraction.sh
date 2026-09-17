#!/usr/bin/env bash
set -euo pipefail

ROOT="/home/ubuntu/a0509_vla_linux_field_bundle_20260903"
PY="${ROOT}/environment/a6000_ubuntu22_py310/bin/python"
EXTRACT="${ROOT}/sim2real_analysis/04_features/extract_vla_features.py"
PHASE6="${ROOT}/lhj/phase6_environment_attribution"
INPUT_ROOT="${PHASE6}/03_camera/condition_inputs"
FEATURE_ROOT="${PHASE6}/03_camera/full_forward_features"
CHECKPOINT="${ROOT}/runtime_state/oft_mixed480_step28560_merged"
INSTRUCTION="Pick up the orange cube."

export PYTHONPATH="${ROOT}:${PYTHONPATH:-}"

conditions=(
  "C0_original"
  "C1_sim_shift_left_24px"
  "C2_sim_shift_right_24px"
  "C3_sim_shift_up_24px"
  "C4_sim_shift_down_24px"
  "C5_sim_zoom_in_1p05"
  "C6_sim_zoom_out_0p95"
  "C7_sim_rotate_ccw_2deg"
  "C8_sim_rotate_cw_2deg"
)

mkdir -p "${FEATURE_ROOT}"

for condition in "${conditions[@]}"; do
  for domain in real sim; do
    list_file="${INPUT_ROOT}/${condition}/${domain}_images.txt"
    output_dir="${FEATURE_ROOT}/${condition}/${domain}"
    if [[ ! -f "${list_file}" ]]; then
      echo "Missing list file: ${list_file}" >&2
      exit 1
    fi
    mapfile -t images < "${list_file}"
    echo "== Extracting camera ${condition}/${domain}: ${#images[@]} images =="
    "${PY}" "${EXTRACT}" \
      --model oft \
      --image-input "${images[@]}" \
      --domain "${domain}" \
      --label "phase6_camera_${condition}_${domain}" \
      --output-dir "${output_dir}" \
      --max-samples "${#images[@]}" \
      --full \
      --instruction "${INSTRUCTION}" \
      --checkpoint "${CHECKPOINT}" \
      --variant oftplus_h5_vision \
      --local-files-only
  done
done

echo "Camera full-forward extraction complete: ${FEATURE_ROOT}"
