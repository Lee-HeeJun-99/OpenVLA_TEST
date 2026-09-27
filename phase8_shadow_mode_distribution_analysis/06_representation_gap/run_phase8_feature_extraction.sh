#!/usr/bin/env bash
set -euo pipefail

ROOT='/home/ubuntu/a0509_vla_linux_field_bundle_20260903'
PY='/home/ubuntu/a0509_vla_linux_field_bundle_20260903/environment/a6000_ubuntu22_py310/bin/python'
EXTRACT='/home/ubuntu/a0509_vla_linux_field_bundle_20260903/sim2real_analysis/04_features/extract_vla_features.py'
CHECKPOINT='/home/ubuntu/a0509_vla_linux_field_bundle_20260903/runtime_state/oft_mixed480_step28560_merged'
FEATURE_ROOT='/home/ubuntu/a0509_vla_linux_field_bundle_20260903/lhj/phase8_shadow_mode_distribution_analysis/06_representation_gap/full_forward_features'
INSTRUCTION='Pick up the orange cube.'

export PYTHONPATH="${ROOT}:${PYTHONPATH:-}"
mkdir -p "${FEATURE_ROOT}"

mapfile -t real_images < '/home/ubuntu/a0509_vla_linux_field_bundle_20260903/lhj/phase8_shadow_mode_distribution_analysis/06_representation_gap/feature_inputs/real_episode_000004_000008_000009_000010_images.txt'
echo "== Extracting Phase 8 real features: ${#real_images[@]} images =="
"${PY}" "${EXTRACT}" \
  --model oft \
  --image-input "${real_images[@]}" \
  --domain real \
  --label phase8_real_episode_000004_000008_000009_000010 \
  --output-dir "${FEATURE_ROOT}/real" \
  --max-samples "${#real_images[@]}" \
  --full \
  --instruction "${INSTRUCTION}" \
  --checkpoint "${CHECKPOINT}" \
  --variant oftplus_h5_vision \
  --local-files-only

mapfile -t sim_images < '/home/ubuntu/a0509_vla_linux_field_bundle_20260903/lhj/phase8_shadow_mode_distribution_analysis/06_representation_gap/feature_inputs/sim_episode_000004_reference_images.txt'
echo "== Extracting Phase 8 fixed sim reference features: ${#sim_images[@]} images =="
"${PY}" "${EXTRACT}" \
  --model oft \
  --image-input "${sim_images[@]}" \
  --domain sim \
  --label phase8_sim_episode_000004_reference \
  --output-dir "${FEATURE_ROOT}/sim_episode_000004_reference" \
  --max-samples "${#sim_images[@]}" \
  --full \
  --instruction "${INSTRUCTION}" \
  --checkpoint "${CHECKPOINT}" \
  --variant oftplus_h5_vision \
  --local-files-only

echo "Phase 8 feature extraction complete: ${FEATURE_ROOT}"
