#!/usr/bin/env bash
set -euo pipefail

PHASE10_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REPO_ROOT="$(cd "${PHASE10_ROOT}/.." && pwd)"

cd "${REPO_ROOT}"
python -m compileall -q phase10_planner_based_shadow_mode
python -m unittest discover -s phase10_planner_based_shadow_mode/tests -v

