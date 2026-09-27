#!/usr/bin/env python3
"""Prepare image lists and extraction commands for Phase 8 representation analysis."""

from __future__ import annotations

import csv
import json
from pathlib import Path


ROOT = Path("/home/ubuntu/a0509_vla_linux_field_bundle_20260903")
PHASE8 = ROOT / "lhj" / "phase8_shadow_mode_distribution_analysis"
INPUT_ROOT = PHASE8 / "06_representation_gap" / "feature_inputs"
FEATURE_ROOT = PHASE8 / "06_representation_gap" / "full_forward_features"
EXTRACT = ROOT / "sim2real_analysis" / "04_features" / "extract_vla_features.py"
PYTHON = ROOT / "environment" / "a6000_ubuntu22_py310" / "bin" / "python"
CHECKPOINT = ROOT / "runtime_state" / "oft_mixed480_step28560_merged"
INSTRUCTION = "Pick up the orange cube."


def read_manifest() -> list[dict[str, str]]:
    path = PHASE8 / "01_data_validation" / "integrated_manifest.csv"
    with path.open("r", encoding="utf-8", newline="") as handle:
        return list(csv.DictReader(handle))


def write_list(path: Path, images: list[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(images) + "\n", encoding="utf-8")


def shell_quote(value: str) -> str:
    return "'" + value.replace("'", "'\"'\"'") + "'"


def main() -> int:
    rows = read_manifest()
    real_rows = [
        row for row in rows
        if row["domain"] == "real"
        and row["episode_id"] in {"episode_000004", "episode_000008", "episode_000009", "episode_000010"}
    ]
    sim4_rows = [
        row for row in rows
        if row["domain"] == "sim" and row["episode_id"] == "episode_000004"
    ]
    real_rows.sort(key=lambda r: (r["episode_id"], int(float(r["frame_id"]))))
    sim4_rows.sort(key=lambda r: int(float(r["frame_id"])))

    real_images = [row["image_path"] for row in real_rows]
    sim_images = [row["image_path"] for row in sim4_rows]
    write_list(INPUT_ROOT / "real_episode_000004_000008_000009_000010_images.txt", real_images)
    write_list(INPUT_ROOT / "sim_episode_000004_reference_images.txt", sim_images)

    manifest = {
        "status": "READY_FOR_FEATURE_EXTRACTION",
        "policy": {
            "model": "oft",
            "variant": "oftplus_h5_vision",
            "checkpoint": str(CHECKPOINT),
            "instruction": INSTRUCTION,
            "use_proprio": False,
        },
        "inputs": {
            "real": {
                "episode_ids": ["episode_000004", "episode_000008", "episode_000009", "episode_000010"],
                "image_count": len(real_images),
                "list_file": str(INPUT_ROOT / "real_episode_000004_000008_000009_000010_images.txt"),
            },
            "sim_reference": {
                "episode_ids": ["episode_000004"],
                "image_count": len(sim_images),
                "list_file": str(INPUT_ROOT / "sim_episode_000004_reference_images.txt"),
            },
        },
        "outputs_expected": {
            "real_feature_manifest": str(FEATURE_ROOT / "real" / "feature_manifest.json"),
            "sim_feature_manifest": str(FEATURE_ROOT / "sim_episode_000004_reference" / "feature_manifest.json"),
        },
    }
    (INPUT_ROOT / "feature_input_manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")

    run_script = PHASE8 / "06_representation_gap" / "run_phase8_feature_extraction.sh"
    real_list = INPUT_ROOT / "real_episode_000004_000008_000009_000010_images.txt"
    sim_list = INPUT_ROOT / "sim_episode_000004_reference_images.txt"
    script = f"""#!/usr/bin/env bash
set -euo pipefail

ROOT={shell_quote(str(ROOT))}
PY={shell_quote(str(PYTHON))}
EXTRACT={shell_quote(str(EXTRACT))}
CHECKPOINT={shell_quote(str(CHECKPOINT))}
FEATURE_ROOT={shell_quote(str(FEATURE_ROOT))}
INSTRUCTION={shell_quote(INSTRUCTION)}

export PYTHONPATH="${{ROOT}}:${{PYTHONPATH:-}}"
mkdir -p "${{FEATURE_ROOT}}"

mapfile -t real_images < {shell_quote(str(real_list))}
echo "== Extracting Phase 8 real features: ${{#real_images[@]}} images =="
"${{PY}}" "${{EXTRACT}}" \\
  --model oft \\
  --image-input "${{real_images[@]}}" \\
  --domain real \\
  --label phase8_real_episode_000004_000008_000009_000010 \\
  --output-dir "${{FEATURE_ROOT}}/real" \\
  --max-samples "${{#real_images[@]}}" \\
  --full \\
  --instruction "${{INSTRUCTION}}" \\
  --checkpoint "${{CHECKPOINT}}" \\
  --variant oftplus_h5_vision \\
  --local-files-only

mapfile -t sim_images < {shell_quote(str(sim_list))}
echo "== Extracting Phase 8 fixed sim reference features: ${{#sim_images[@]}} images =="
"${{PY}}" "${{EXTRACT}}" \\
  --model oft \\
  --image-input "${{sim_images[@]}}" \\
  --domain sim \\
  --label phase8_sim_episode_000004_reference \\
  --output-dir "${{FEATURE_ROOT}}/sim_episode_000004_reference" \\
  --max-samples "${{#sim_images[@]}}" \\
  --full \\
  --instruction "${{INSTRUCTION}}" \\
  --checkpoint "${{CHECKPOINT}}" \\
  --variant oftplus_h5_vision \\
  --local-files-only

echo "Phase 8 feature extraction complete: ${{FEATURE_ROOT}}"
"""
    run_script.write_text(script, encoding="utf-8")
    run_script.chmod(0o755)
    print(json.dumps(manifest, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
