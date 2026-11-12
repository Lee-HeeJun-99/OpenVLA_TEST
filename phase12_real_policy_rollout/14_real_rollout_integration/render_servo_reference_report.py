"""Render offline results and reviewable adapter diffs; never patch live code."""
import difflib
import json
from pathlib import Path

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'real_trials/20261112_190404_reference_replay_03'


def main():
    data=json.loads((OUT/'results.json').read_text())
    prefix='''"""Offline reference adapter proposal; no ROS or transport construction."""
from servo_experimental.reference_replay import delta_target, install_reference

'''
    original='''def update(tracker, limited_action, actual_pose, source, now, boundary=None):
    anchor = actual_pose
    target = delta_target(anchor, limited_action)
    tracker.set_target(target, source, now)
    if boundary is not None:
        boundary.begin_segment(source, limited_action)
    return anchor, target
'''
    suffix='''
def tick(tracker, now, feedback, feedback_received, boundary=None):
    row = tracker.tick(now)
    if row is not None and boundary is not None:
        # Replay provides fake publish/stop callbacks; never a live transport.
        boundary.inspect_and_publish(row['pos'], now, feedback, feedback_received)
    return row
'''
    baseline=prefix+original+suffix
    (OUT/'original_reference_adapter.py').write_text(baseline)
    for mode in ('command','actual'):
        new=original.replace('    anchor = actual_pose\n    target = delta_target(anchor, limited_action)\n',
            f'    anchor, target = install_reference(tracker, limited_action, actual_pose, "{mode}")\n')
        proposal=prefix+new+suffix
        compile(proposal,f'proposal_{mode}_reference_adapter.py','exec')
        (OUT/f'proposal_{mode}_reference_adapter.py').write_text(proposal)
        diff=''.join(difflib.unified_diff(baseline.splitlines(True),proposal.splitlines(True),
            fromfile='original_reference_adapter.py',tofile=f'proposal_{mode}_reference_adapter.py'))
        (OUT/f'proposal_{mode}.diff').write_text(diff)
    lines=['# 190404: stream reference audit and offline alternatives','',
        '**No robot/ROS/inference calls. No live launcher or physical settings/limits changed.**',
        '', '## 1. Code evidence',
        '', '- `start_servo_episode_confirmed.py:265-268`: tracker initialized from live CurrentPosX once.',
        '- `start_servo_episode_confirmed.py:328-335`: every inference constructs endpoint from newly read real controller pose, not last command.',
        '- `02_safety/command_disabled_runtime.py`: p_target=p_actual+1000*limited_translation; R_target=Exp(limited_world_rotvec)*R_actual.',
        '- `servo_experimental/controller.py:102-114`: set_target replaces only target/source; commanded pose and both velocities remain.',
        '- `servo_experimental/controller.py:135-150`: error is target minus virtual commanded state; velocity slew and trapezoidal integration persist. Slerp is local midpoint integration, NOT a bounded original-start-to-final-target interpolation.',
        '- `start_servo_episode_confirmed.py:349-353,365-381`: target replacement and ticks are serialized, but producer/stream run independently. Tick state can change before actual publication. Normally previous tick state equals last emitted request, not controller feedback or vendor-confirmed pose.',
        '- `servo_experimental/controller.py:151-153` and launcher `:378`: each 20Hz message carries unchanged transport time=0.7, vel=[20,20], acc=[40,40]. This is not Python interpolation duration or evidence of reaching each target in 0.7sec.',
        '- Local driver `dsr_controller2.cpp:2217-2227`: copies pos/vel/acc/time into Drfl->servol. Internal vendor retarget scheduling is not logged and cannot be inferred here.',
        '', '## 2. Step215..222 reference table',
        '', 'Each pose is `[x,y,z; A,B,C]`, mm / intrinsic ZYZ deg. Stream start is the LAST published-request pose before the source changes, not the first already-advanced tick. Actual pose is the logged controller sample used to form the target. ABC values are representation-nonunique.',
        '', '| Step | Stream start | Actual pose | Original target |', '|---:|---|---|---|']
    def pose(x):return '['+', '.join(f'{v:.4f}' for v in x[:3])+'; '+', '.join(f'{v:.4f}' for v in x[3:])+']'
    for s in data['selected_original_segments']:
        lines.append(f"| {s['step']} | {pose(s['stream_start'])} | {pose(s['actual_pose'])} | {pose(s['target'])} |")
    lines+=['','| Step | Limited rotation deg | Start/actual gap deg | Start/target distance deg | Stream cumulative rotation deg |',
        '|---:|---:|---:|---:|---:|']
    for s in data['selected_original_segments']:
        lines.append(f"| {s['step']} | {s['action_rotation_deg']:.6f} | {s['actual_start_rotation_deg']:.6f} | {s['start_target_rotation_deg']:.6f} | {s['stream_rotation_arc_deg']:.6f} |")
    lines+=['', 'At step222 the 0.111427deg limited action is relative to real pose, while tracker starts 11.568308deg away. Its target is therefore 11.673956deg from the tracker start. It traverses a cumulative 7.617239deg in that source interval. This includes the initial start-to-first-tick movement and all arc segments; the earlier 7.027deg first-to-last chord excludes the first tick and is a different quantity.',
        '', 'This establishes an algorithmic reference mismatch and retained-momentum/overshoot contribution. It does NOT prove why the physical controller began deviating or simulate its IK, Euler interpretation, or inner loop.',
        '', '## 3. Original-stream replay validation',
        '', 'All2143 original stream poses reproduced exactly: maximum translation difference=0mm, rotation=0deg. Time/source/actions are from the frozen run. No vendor/robot physics is simulated.',
        '', '## 4. Alternatives and diff files',
        '', '- **A command reference**: endpoint = last commanded pose + limited delta; virtual state/velocities retained. Following-error guard is required to prevent commanded reference drifting away from actual pose.',
        '- **B actual reference**: endpoint AND tracker start rebased to the same recorded real pose, velocities reset. This naive alternative is explicit for comparison, NOT approved: every rebase can jump from the prior published command and velocity continuity is lost.',
        '- `proposal_command.diff` / `proposal_actual.diff` are exact unified diffs against the included `original_reference_adapter.py`. Included adapters compile and have no ROS/transport construction. They are offline review proposals, NOT patches silently applied to the live launcher.',
        '', '### Whole recorded action sequence, guards disabled for diagnosis only',
        '', '| Mode | Replayed commands | Max rotation arc/action ratio | Outcome |', '|---|---:|---:|---|']
    for s in data['summary']:
        if not s['guarded']:
            lines.append(f"| {s['mode']} | {s['rows']} | {s['maximum_rotation_arc_action_ratio']:.3f} | {s['failure']} |")
    lines+=['', 'A does NOT remove all amplification; target-reference consistency alone leaves the existing integrator overshoot. A reaches unchanged workspace rejection at step197. B also does NOT solve the problem: rebasing to recorded real poses contributes artificial jumps and breaks velocity continuity. Do not adopt either naive alternative as physically validated.',
        '', '### Local replay seeded from the last published state before step215',
        '', '| Mode | Step222 action deg | Step222 cumulative stream rotation deg | Maximum rotation arc/action ratio |', '|---|---:|---:|---:|']
    for s in data['local_step215_seeded_comparison']:
        if not s['guarded']:
            lines.append(f"| {s['mode']} | {s['last_segment_action_rotation_deg']:.6f} | {s['last_segment_rotation_arc_deg']:.6f} | {s['maximum_rotation_arc_action_ratio']:.3f} |")
    lines+=['', 'The alternatives reuse recorded actual poses as exogenous inputs. Once command reference changes, those real future poses would not be the same in a physical counterfactual. Hence this is a generator stress replay, not a prediction of modified robot motion.',
        '', '## 5. Added checks — offline boundary only',
        '', '`servo_experimental/reference_replay.py::OfflineStreamBoundary` checks BEFORE a fake publish:',
        '', '1. Net translation/rotation AND cumulative translation/rotation arc must not exceed limited-action magnitude plus explicit tolerance. Cumulative arc prevents oscillation cancellation.',
        '2. Real/command distance uses geodesic orientation and command history at the feedback receive time, never a future candidate. Stale/future feedback fails closed.',
        '3. Fault latches future dispatch inhibition and invokes the supplied fake stop callback once if there was prior publication. Additional candidate attempts after abort publish zero commands.',
        '', 'No new tracking thresholds were installed in physical configuration. Tests use numerical action-budget tolerance1e-6mm/deg. Offline following-error sensitivity uses (5mm,1deg), (10mm,2deg), (20mm,4deg), (50mm,8deg); these are diagnostic values, not approved physical thresholds. The feedback age reference is the frozen existing max_action_age_sec1.2; this does not validate it as a sensor-age contract.',
        '', '### Both checks enabled',
        '', 'All three full-run variants inhibit at step0, before a proposed 0.915319deg excursion exceeds the first limited action0.8deg. This exposes existing overshoot even before the terminal incident. Stop callback once; commands after abort0. A strict action budget does not constitute a finished smooth trajectory controller.',
        '', 'Local seeded replay: original inhibits at step216; A and B inhibit at step215. Thus unsafe candidates are prevented rather than silently clipped/scaled.',
        '', '### Following check isolated (budget check disabled ONLY for diagnostic isolation)',
        '', '| Diagnostic bounds mm/deg | First blocked step | Measured comparison rotation deg |', '|---|---:|---:|']
    for s in data['following_limit_sensitivity']:
        e=s['failure']['event']
        lines.append(f"| {s['following_limit_mm']:g}/{s['following_limit_deg']:g} | {s['failure']['step']} | {e['following_rotation_deg']:.6f} |")
    lines+=['', 'With the loose50mm/8deg diagnostic bounds, inhibition occurs at monotonic78801.943377062, about0.161sec before recorded J5 first crossed the historical-135deg reference. No stopping-distance guarantee follows: a vendor Stop ACK/latency/physical stopping model is absent.',
        '', '## 6. Disposition',
        '', 'Current live execution path and every physical limit/configuration remain unchanged. Added checks are IMPLEMENTED AND TESTED in the offline proposal boundary, not enabled live without a selected reference/trajectory design and justified tracking tolerances. Simple reference switching alone is insufficient; endpoint-bounded interpolation/braking with continuous replans and validated following/joint protection is still needed.',
        '', 'Added source files: `servo_experimental/reference_replay.py`, `replay_servo_references.py`, `render_servo_reference_report.py`, `tests/test_servo_reference_replay.py`. Original trial remains unchanged.',
        '', 'Validation: **460 passed**,26 existing SciPy gimbal-lock warnings. New reference/guard suite10 tests. Original replay equality, named-anchor behavior, arc overshoot, following error, epoch-free timing, future-command avoidance, equivalent Euler representations, nonfinite candidate inhibition, stale feedback and post-abort no dispatch are covered.',
        '', 'Reproduce: `/usr/bin/python3 replay_servo_references.py` then `/usr/bin/python3 render_servo_reference_report.py`. Existing result directory must not already exist for replay. All output paths are separate from source trial.']
    (OUT/'REFERENCE_STREAM_REPORT.md').write_text('\n'.join(lines)+'\n')


if __name__=='__main__':main()
