"""One explicitly authorized full-policy live episode; no automatic retry."""
import argparse,json
from pathlib import Path
from full_policy_live_mode import FullPolicyLiveMode
from run_arm_only_episode import run


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--approval',type=Path)
    p.add_argument('--open-confirmation',type=Path)
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
    if args.approval is None or args.open_confirmation is None:p.error('--approval and --open-confirmation required for live execution')
    run(args.output,json.loads(args.approval.read_text()),args.max_duration_s,
        _full_policy_mode=FullPolicyLiveMode(args.open_confirmation))

if __name__=='__main__':main()
