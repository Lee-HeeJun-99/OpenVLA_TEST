#!/usr/bin/python3
"""Compare arm endpoint-token statistics in matched15 and real replay."""

from __future__ import annotations
import json
from pathlib import Path
import sys
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from controller.action import ActionInterpreter
from controller.precheck import EndpointStreak

BUNDLE = Path("/home/ubuntu/a0509_vla_linux_field_bundle_20260903")
EVIDENCE = BUNDLE / "validation/evidence/vanilla/success_profile_s1_balanced_step8130_matched15_k1_seed9406_15ep.json"
STATS = BUNDLE / "models/vanilla_s1_balanced_step8130/dataset_statistics.json"
REAL = BUNDLE / "lhj/phase12_real_policy_rollout/14_real_rollout_integration/real_trials/20261112_190404_arm_only_servo_episode/predictions.jsonl"


def summarize(sequences, interpreter):
    counts = np.zeros((6, 2), dtype=int); maximum = np.zeros((6, 2), dtype=int)
    action_count = 0
    per_sequence = []
    for sequence in sequences:
        tracker = EndpointStreak(); local_count = np.zeros((6, 2), dtype=int)
        local_max = np.zeros((6, 2), dtype=int)
        for raw in sequence:
            action = interpreter.parse(raw); action_count += 1
            local_count[:, 0] += action.endpoint_min[:6]
            local_count[:, 1] += action.endpoint_max[:6]
            local_max = np.maximum(local_max, tracker.update(action))
        counts += local_count; maximum = np.maximum(maximum, local_max)
        per_sequence.append({"actions": len(sequence), "counts": local_count.tolist(),
                             "max_streak": local_max.tolist()})
    return {"sequences": len(sequences), "actions": action_count,
            "endpoint_counts": counts.tolist(),
            "endpoint_frequency_per_action": (counts/max(action_count, 1)).tolist(),
            "maximum_consecutive_streak": maximum.tolist(),
            "per_sequence": per_sequence}


def main():
    interpreter = ActionInterpreter(STATS)
    evidence = json.loads(EVIDENCE.read_text())
    successful = [episode for episode in evidence["results"] if episode["success"]]
    sim_sequences = [[a["raw_action"] for a in episode["actions"]]
                     for episode in successful]
    real_sequence = [[json.loads(line)["raw_action"]
                      for line in REAL.open()]]
    output = {"sim_matched15_success": summarize(sim_sequences, interpreter),
              "real_190404": summarize(real_sequence, interpreter),
              "dimensions": ["dx", "dy", "dz", "rx", "ry", "rz"],
              "sides": ["min", "max"], "statistics": str(STATS)}
    path = ROOT / "reports/endpoint_policy_comparison.json"
    path.write_text(json.dumps(output, indent=2)+"\n")
    print(path)


if __name__ == "__main__":
    main()
