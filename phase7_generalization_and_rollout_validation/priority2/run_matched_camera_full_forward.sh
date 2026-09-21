#!/usr/bin/env bash
set -euo pipefail

ROOT="/home/ubuntu/a0509_vla_linux_field_bundle_20260903"
PY="${ROOT}/environment/a6000_ubuntu22_py310/bin/python"
EXTRACT="${ROOT}/sim2real_analysis/04_features/extract_vla_features.py"
CHECKPOINT="${ROOT}/runtime_state/oft_mixed480_step28560_merged"
INSTRUCTION="Pick up the orange cube."

INPUT_ROOT="${ROOT}/lhj/phase7_generalization_and_rollout_validation/priority2/02_matched_camera_shift/condition_inputs"
OUTPUT_ROOT="${ROOT}/lhj/phase7_generalization_and_rollout_validation/priority2/02_matched_camera_shift/full_forward_features"

export PYTHONPATH="${ROOT}:${PYTHONPATH:-}"

conditions=(
  "M0_raw_original"
  "M1_raw_shift_right_24px"
  "M2_p4_shift_right_4px_matched"
  "M3_p4_shift_right_24px_control"
)

if [[ ! -x "${PY}" ]]; then
  echo "Python not executable: ${PY}" >&2
  exit 1
fi
if [[ ! -f "${EXTRACT}" ]]; then
  echo "Missing extract script: ${EXTRACT}" >&2
  exit 1
fi
if [[ ! -d "${CHECKPOINT}" ]]; then
  echo "Missing checkpoint dir: ${CHECKPOINT}" >&2
  exit 1
fi

mkdir -p "${OUTPUT_ROOT}"

for condition in "${conditions[@]}"; do
  for domain in real sim; do
    list_file="${INPUT_ROOT}/${condition}/${domain}_images.txt"
    output_dir="${OUTPUT_ROOT}/${condition}/${domain}"
    if [[ ! -f "${list_file}" ]]; then
      echo "Missing list file: ${list_file}" >&2
      exit 1
    fi
    mapfile -t images < "${list_file}"
    if [[ "${#images[@]}" -eq 0 ]]; then
      echo "No images listed in ${list_file}" >&2
      exit 1
    fi
    echo "== Extracting matched camera ${condition}/${domain}: ${#images[@]} images =="
    "${PY}" "${EXTRACT}" \
      --model oft \
      --image-input "${images[@]}" \
      --domain "${domain}" \
      --label "phase7_p2_camera_${condition}_${domain}" \
      --output-dir "${output_dir}" \
      --max-samples "${#images[@]}" \
      --full \
      --instruction "${INSTRUCTION}" \
      --checkpoint "${CHECKPOINT}" \
      --variant oftplus_h5_vision \
      --local-files-only
  done
done

echo "Matched camera full-forward extraction complete: ${OUTPUT_ROOT}"
