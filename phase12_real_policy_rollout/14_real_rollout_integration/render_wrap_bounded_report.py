"""Render recorded-data analysis only; does not invoke the draft test runner."""
import csv
import json
from pathlib import Path

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'real_trials/20261112_190404_wrap_bounded_analysis_02'


def main():
    d=json.loads((OUT/'results.json').read_text())
    rows=list(csv.DictReader((OUT/'steps_200_222_error_growth.csv').open()))
    lines=['# 190404 C-wrap, signed lead, bounded interpolation and test draft','',
        '**Physical calls0. Original trial and all live limits/configurations unchanged.**',
        '', '## 1. Timing and interpretation',
        '', 'First emitted C outside[-180,180]: monotonic78799.595141412, C=-180.059768463deg (source step217).',
        '', 'The terminal uninterrupted increase begins at sample218. Its onset lies between sample217 at78799.423470039 (0.462328deg error) and sample218 at78799.900505753 (0.902106deg error). The C crossing falls INSIDE this interval. With pose sampled only once per inference and unknown acquisition/vendor-consumption delay, the logs cannot determine which physically began first.',
        '', 'A weaker earlier three-positive-growth sequence exists at205..207, before crossing. This rules out claiming that all error growth first started only at the C boundary. Late error first exceeds normal p99 at step220/time78800.901082415,1.305941sec after crossing. Correlation is strong in the terminal window, causality remains unproven.',
        '', 'Actual CurrentPosX is compared to the last requested command at or before its receive time; no future command is used. Initial step0 precedes the first stream request and is compared to the initial pose. All angles use rotation matrices/quaternions.',
        '', '### Step200..222 errors and growth rates',
        '', '| Step | Pose receive monotonic | Position error mm | Growth mm/s | Rotation error deg | Growth deg/s |',
        '|---:|---:|---:|---:|---:|---:|']
    for r in rows:
        lines.append('| '+ ' | '.join([r['step']]+[f'{float(r[k]):.6f}' for k in ('feedback_receive_monotonic','translation_error_mm','translation_error_growth_mm_s','rotation_error_deg','rotation_error_growth_deg_s')])+' |')
    lines+=['','### Signed ahead/behind projections',
        '', 'World translation error e=p_actual-p_requested; axis lead_i=e_i*sign(v_requested_i). Positive is ahead along that axis current requested direction; negative is behind. Zero requested axis speed is undefined, not forced to ahead/behind.',
        '', 'World rotation error is Log(R_actual*R_requested^-1). Its dot product with the unit requested world angular-velocity vector is signed rotational lead. Positive means geometrically ahead along that instantaneous direction, NOT vendor-confirmed tracking/latency or a proven cause.',
        '', '| Step | Axis signed lead X/Y/Z mm | Translation-direction lead mm | Rotation-direction lead deg |',
        '|---:|---|---:|---:|']
    for r in rows:
        values=json.loads(r['translation_axis_signed_lead_mm'])
        axis='['+', '.join('undefined' if x is None else f'{x:.6f}' for x in values)+']'
        lines.append(f"| {r['step']} | {axis} | {float(r['signed_translation_direction_lead_mm']):.6f} | {float(r['signed_rotation_direction_lead_deg']):.6f} |")
    lines+=['', 'Terminal rotation projection is positive at218..222 and reaches+11.498508deg. Translation is not uniformly ahead: at220 all three axis projections are negative; at222 X/Y are ahead but Z is behind. Therefore a blanket statement that every component of the robot runs ahead is incorrect.',
        '', '## 2. New offline generator',
        '', '`servo_experimental/bounded_segment.py::CommandSegmentInterpolator`:',
        '', '- Command reference: segment start is previous generated/published command; target is that same pose plus the recorded limited action.',
        '- Position: p(u)=p_start+u*(p_target-p_start). Orientation: shortest-arc Slerp(R_start,R_target,u). No velocity state or speed integration.',
        '- Time fraction u=clip((now-activation)/duration,0,1). Duration uses the last observed target activation interval (causal), first defaults to frozen1/control_rate_hz. Future inference duration is not used. A late target holds the endpoint; an early target preempts at current command without a position/orientation jump.',
        '- ABC is normalized to[-180,180], evaluating signed-B equivalent forms and endpoint wraps and selecting the nearest numeric representation to the previous normalized command. Matrix reconstruction is checked. This can create an unavoidable Euler numeric wrap even though the SO(3) path is continuous; it is NOT proof of Doosan interpolation behavior.',
        '- Net and cumulative position/rotation path budgets are bounded by limited action+1e-6mm/deg numerical tolerance. Workspace, action-age and tick-gap checks fail closed. Physical settings untouched.',
        '', '### Full recorded-input replay',
        '', '| Generator | Emitted offline rows | Valid segments | Translation/rotation budget violations | Translation/rotation target overshoot segments | Workspace outcome |',
        '|---|---:|---:|---|---|---|']
    for r in d['replay_comparison']:
        lines.append(f"| {r['mode']} | {r['rows']} | {r['segments']} | {r['translation_budget_violating_segments']}/{r['rotation_budget_violating_segments']} | {r['translation_target_overshoot_segments']}/{r['rotation_target_overshoot_segments']} | {r['failure']} |")
    lines+=['', 'Counts cover only the prefix before first rejection: original223 segments; proposalA197; bounded174. The bounded candidate hits unchanged workspace at step174, so it is not a completed full episode. Counts with different denominators are not a task-performance ranking.',
        '', 'Target overshoot is positive projection beyond the geometric endpoint; action budget separately counts net excursion or cumulative path beyond limited action. They are different metrics.',
        '', '### Same terminal window200..222, seeded from prior emitted pose',
        '', '| Generator | Segments | Translation/rotation budget violations | Translation/rotation overshoot segments | Workspace rejection |',
        '|---|---:|---|---|---|']
    for r in d['terminal_200_seeded_replay']:
        lines.append(f"| {r['mode']} | {r['segments']} | {r['translation_budget_violating_segments']}/{r['rotation_budget_violating_segments']} | {r['translation_target_overshoot_segments']}/{r['rotation_target_overshoot_segments']} | {r['failure']} |")
    lines+=['', 'The bounded alternative eliminates generator overshoot/budget amplification in this23-segment terminal replay. It is NOT a robot physics simulation; changed commands would change future camera feedback/policy outputs.',
        '', '### Dynamic feasibility warning — do not hide this result',
        '', 'With a linear time law, derivative jumps remain at retarget/start/stop. Against unchanged planning limits20mm/s,20deg/s,20mm/s²,20deg/s², full-prefix offline diagnostics find136 translational-speed,357 translational-acceleration and343 rotational-acceleration violating samples. Terminal-only replay has45/42 acceleration violations. Consequently the mathematical budget fix is **NOT_PHYSICAL_READY**. No limiter was silently removed, no bound raised, and no live path changed. A future time law must honor the same physical bounds, possibly taking longer than one inference cycle.',
        '', '## 3. Empirical following-error distribution: designated steps0..216',
        '', '217 controller-pose samples including the initial pre-stream reference; this is the user-designated nominal subset, not independently certified fault-free. Errors use controller pose, not URDF FK, and receive-time alignment, not vendor execution timing.',
        '', '| Metric | p50 | p95 | p99 | max |','|---|---:|---:|---:|---:|']
    for key,values in d['normal_steps_0_216_distribution'].items():
        lines.append('| '+key+' | '+' | '.join(f'{values[k]:.6f}' for k in ('p50','p95','p99','max'))+' |')
    lines+=['', '| Diagnostic candidate | Translation mm | Rotation deg | Nominal samples rejected | First terminal rejection |', '|---|---:|---:|---:|---:|']
    for c in d['threshold_candidates']:
        lines.append(f"| {c['name']} | {c['translation_mm']:g} | {c['rotation_deg']:g} | {c['normal_rejections']}/217 | {c['first_late_rejection']} |")
    lines+=['', 'Recommendation for further offline validation:10mm/3deg is a max-covering candidate;7.5mm/2.4deg is more sensitive but rejects2 historical nominal samples. Neither is installed, neither is a certified safety limit, and neither proves stopping distance. Validate on independent data and a functioning stop path before choosing. Temporal correlation precludes treating217 samples as independent trials.',
        '', '## 4. Physical synthetic test DRAFT — NOT EXECUTED',
        '', '`draft_servo_C_wrap_test.py` was written, syntax-checked and its pure geometry helpers unit-tested only. The physical executor was never called, no ROS node/publisher/service was created by this work.',
        '', '- One +/-3deg intrinsic C motion, either positive or negative, chosen per invocation. No automatic reverse, retry, home, gripper or model inference.',
        '- Already reviewed safe joint posture must be supplied and matched within the existing0.5deg start criterion. No invented safe pose and no automatic move to it. Current C must already be within3deg of the selected boundary; otherwise refuse.',
        '- Separate normalized and unwrapped modes; identical SO(3) rotation is verified in pure tests. The unwrapped mode intentionally crosses the candidate Euler input range for investigation, so vendor/profile approval is mandatory.',
        '- Eight-second analytic smoothstep profile: peak approximately0.704deg/s, acceleration0.271deg/s². Existing transport vel/acc/time remain unchanged. This test time law differs from the new linear-time replay generator to keep the diagnostic motion low-speed.',
        '- Require reviewed controller joint limits, explicit positive joint margins, verified TCP offset, controller version/evidence, verified stop path/profile, and fresh operator/authority/stops confirmation. Default template has all approvals false and all unknown quantities null.',
        '- Current joint proximity and nominal Jacobian/IK forecast block motion; TCP/flange transform is explicitly applied. Nominal forecast is not a vendor IK guarantee.',
        '- Following-error guard, JointState freshness/gap checks, workspace, state and stream-stall checks inhibit dispatch. Existing HOLD(3) once on exit; ACK and fresh post-stop STANDBY/zero-velocity window must confirm. No automatic recovery after failure.',
        '', 'The current known Stop ACK problem means the reviewed stop prerequisite cannot simply be assumed true. The script is a draft, not authorization or a ready-to-run recommendation. No execution command is provided here.',
        '', '## Files and validation',
        '', '- `steps_200_222_error_growth.csv`: all requested rates and projections.',
        '- `all_steps_error_projection.csv`: full per-step error samples.',
        '- `bounded_commands.csv`, `bounded_segments.csv`: full-prefix candidate.',
        '- `terminal_200_bounded_commands.csv`: same-window candidate.',
        '- `results.json`, `input_sha256.json`: metrics, caveats and source integrity.',
        '', 'Added source: `servo_experimental/bounded_segment.py`, `analyze_wrap_and_bounded_replay.py`, `render_wrap_bounded_report.py`, `draft_servo_C_wrap_test.py`, `tests/test_bounded_segment.py`, and an unapproved test-contract template. Existing reference replay was imported read-only. No production launcher/limit files changed.',
        '', 'Final test result recorded separately in VALIDATION.txt. Draft runner was NOT invoked; only pure helpers were tested.']
    (OUT/'WRAP_BOUNDED_REPORT.md').write_text('\n'.join(lines)+'\n')


if __name__=='__main__':main()
