#!/usr/bin/python3
"""V2 vanilla health contract: matched15 preprocessing has no center crop."""
import argparse,json
from urllib.request import urlopen

parser=argparse.ArgumentParser();parser.add_argument("--url",default="http://127.0.0.1:8766")
args=parser.parse_args()
with urlopen(args.url.rstrip("/")+"/health",timeout=15) as response:health=json.load(response)
checks={"ready":health.get("status")=="ready","variant":health.get("variant")=="openvla_token",
        "chunk_size":health.get("chunk_size")==1,"action_dim":health.get("action_dim")==7,
        "vision_only":health.get("requires_proprio") is False,
        "center_crop_disabled":health.get("center_crop") is False,
        "rgb":health.get("preprocessing",{}).get("color_order")=="RGB"}
result={"status":"PASS" if all(checks.values()) else "FAIL","checks":checks,"health":health}
print(json.dumps(result,indent=2))
if result["status"]!="PASS":raise SystemExit(1)
