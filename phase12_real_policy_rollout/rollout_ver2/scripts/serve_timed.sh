#!/usr/bin/env bash
set -euo pipefail

ROLLOUT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROOT="$(cd "${ROLLOUT_ROOT}/../../.." && pwd)"
PORT="${1:-8765}"
GPU="${2:-0}"
HOST="${3:-127.0.0.1}"
PYTHON="${ROOT}/environment/a6000_ubuntu22_py310/bin/python"
BASE="${ROOT}/models/base_openvla_7b"
STATE_ROOT="${A0509_RUNTIME_STATE_ROOT:-${ROOT}/runtime_state}"

[[ -x "${PYTHON}" ]] || {
  echo "Run ${ROOT}/scripts/bootstrap.sh first" >&2
  exit 2
}
if command -v ss >/dev/null && ss -ltn | awk '{print $4}' | grep -qE "(^|:)${PORT}$"; then
  echo "Port ${PORT} is already in use" >&2
  exit 3
fi

mkdir -p "${STATE_ROOT}" "${ROOT}/logs"
export CUDA_VISIBLE_DEVICES="${GPU}"
export HF_HOME="${STATE_ROOT}/huggingface"
export HF_HUB_OFFLINE=1
export TRANSFORMERS_OFFLINE=1
export TOKENIZERS_PARALLELISM=false
export PYTHONUNBUFFERED=1
export A0509_ACTION_CHUNK_SIZE=1
export PYTHONPATH="${ROOT}/runtime/vanilla-support:${ROOT}/runtime/openvla-oft"

exec "${PYTHON}" "${ROLLOUT_ROOT}/runtime/serve_adapter_timed.py" \
  --bundle "${ROOT}/models/vanilla_s1_balanced_step8130" \
  --model-id "${BASE}" \
  --host "${HOST}" \
  --port "${PORT}" \
  --local-files-only
