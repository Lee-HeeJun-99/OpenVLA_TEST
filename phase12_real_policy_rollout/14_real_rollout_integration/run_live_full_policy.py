"""One explicitly authorized full-policy live episode; no automatic retry."""
import argparse,json
from pathlib import Path
from full_policy_live_mode import FullPolicyLiveMode
from run_arm_only_episode import run


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--approval',type=Path)
    p.add_argument('--open-confirmation',type=Path)
    p.add_argument('--direct',action='store_true',help='MoveLine의 명시적 무수동확인 정책; OPEN은 ACK 기반, 실제 jaw 상태 UNKNOWN')
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--max-duration-s',type=float)
    p.add_argument('--finalize-outcome',type=Path,help='offline outcome evidence; does not start ROS or motion')
    args=p.parse_args()
    if args.finalize_outcome:
        summary=json.loads((args.output/'trial_summary.json').read_text())
        if summary.get('evaluation_scope')!='FULL_POLICY_ROLLOUT':raise ValueError('not_full_policy_trial')
        updated=FullPolicyLiveMode.finalize_outcome(summary,json.loads(args.finalize_outcome.read_text()))
        (args.output/'trial_summary.json').write_text(json.dumps(updated,indent=2))
        print(json.dumps(updated,indent=2));return
    if args.approval is None:p.error('--approval launch request required')
    if args.direct:
        if args.open_confirmation is not None:p.error('--direct does not use manual OPEN evidence')
        from direct_moveline_mode import DirectMoveLineMode
        mode=DirectMoveLineMode()
    else:
        if args.open_confirmation is None:p.error('--open-confirmation required for confirmed mode')
        mode=FullPolicyLiveMode(args.open_confirmation)
    run(args.output,json.loads(args.approval.read_text()),args.max_duration_s,
        _full_policy_mode=mode)

if __name__=='__main__':main()
