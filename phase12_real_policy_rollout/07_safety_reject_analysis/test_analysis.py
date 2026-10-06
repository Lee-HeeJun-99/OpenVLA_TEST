import csv
import json
import sys
import unittest
from pathlib import Path

ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT))
from analysis_core import RESULTS,CONFIG,OPEN_OUT,OFT_OUT,OPEN_SRC,OFT_SRC,enrich_model,read_jsonl

class RejectAnalysisTest(unittest.TestCase):
    def test_recorded_baseline_counts_and_exact_replay(self):
        expected={"openvla":(OPEN_OUT,OPEN_SRC,16),"oft":(OFT_OUT,OFT_SRC,3)}
        for model,(out,src,n) in expected.items():
            rows,mismatches=enrich_model(model,read_jsonl(out),src)
            self.assertEqual(len(rows),45)
            self.assertEqual(sum(r["accepted"] for r in rows),n)
            self.assertEqual(mismatches,[])

    def test_all_outputs_are_command_disabled(self):
        for name in ("openvla_action_analysis.csv","oft_action_analysis.csv"):
            with (RESULTS/name).open() as f: rows=list(csv.DictReader(f))
            self.assertTrue(rows and all(r["command_issued"]=="False" for r in rows))

    def test_action_rows_retain_all_actual_blockers(self):
        for _,out,name in (("openvla",OPEN_OUT,"openvla_action_analysis.csv"),("oft",OFT_OUT,"oft_action_analysis.csv")):
            source=read_jsonl(out)
            with (RESULTS/name).open() as f: result=list(csv.DictReader(f))
            self.assertEqual([json.loads(r["blockers"]) for r in result],
                             [r["safety_decision"]["reason"] for r in source])

    def test_production_config_unchanged_values(self):
        import yaml
        c=yaml.safe_load(CONFIG.read_text())
        self.assertEqual(c["translation_step_m"],.004)
        self.assertEqual(c["rotation_step_deg"],4.0)
        self.assertEqual(c["translation_velocity_mm_s"],20.0)
        self.assertEqual(c["translation_acceleration_mm_s2"],20.0)

    def test_oft_chunk_indices_complete(self):
        with (RESULTS/"oft_action_analysis.csv").open() as f: rows=list(csv.DictReader(f))
        self.assertEqual({int(r["chunk_index"]) for r in rows},set(range(5)))
        self.assertTrue(all(sum(int(r["chunk_index"])==k for r in rows)==9 for k in range(5)))

if __name__ == "__main__": unittest.main()
