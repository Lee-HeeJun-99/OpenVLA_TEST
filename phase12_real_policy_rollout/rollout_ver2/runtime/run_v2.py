#!/usr/bin/python3
"""V2 mode authorization front-end. Physical adapters remain opt-in."""
import argparse,json,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path: sys.path.insert(0,str(ROOT))
from runtime.executor import V2Executor,MODES
from runtime.gate import ExecutionGate,GateEntry


def main():
    parser=argparse.ArgumentParser();parser.add_argument("mode",choices=MODES)
    parser.add_argument("--gates",type=Path,default=Path(__file__).with_name("gates.json"))
    parser.add_argument("--enable-physical",action="store_true")
    args=parser.parse_args();raw=json.loads(args.gates.read_text())
    gate=ExecutionGate({k:GateEntry(bool(v["passed"]),v.get("evidence")) for k,v in raw.items()})
    executor=V2Executor(gate,physical_enabled=args.enable_physical)
    print(json.dumps({"mode":args.mode,"contract":executor.mode_contract(args.mode),
                      "gate_failures":gate.audit(args.mode),"physical_enabled":args.enable_physical},indent=2))
    confirmation=input("Type I_CONFIRM_V2_MODE:"+args.mode+" exactly: ")
    executor.authorize(args.mode,confirmation)
    if args.mode=="dryrun_offline":print("AUTHORIZED_DRYRUN_OFFLINE")
    else:raise RuntimeError("MODE_ADAPTER_NOT_BOUND_IN_CLI_FAIL_CLOSED")


if __name__=="__main__":main()
