#!/usr/bin/env python3
"""Recorded files only; no model loading, network, or robot capability."""
import csv
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parent
RESEARCH = ROOT.parent.parent
BUNDLE = RESEARCH.parent


def load_rows(path):
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]


def write_csv(path, rows):
    with path.open('w', newline='', encoding='utf-8') as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def distribution(values):
    return dict(count=len(values), mean=sum(values)/len(values),
                close_ratio_ge_0p7=sum(x >= .7 for x in values)/len(values),
                binary_close_ratio_ge_0p5=sum(x >= .5 for x in values)/len(values))


def future_grippers(labels):
    # Same terminal policy as A0509OFTDataset: last gripper retained.
    return [[labels[min(t+k, len(labels)-1)] for k in range(5)] for t in range(len(labels))]


def main():
    out = ROOT / 'results'
    out.mkdir(parents=True, exist_ok=True)
    inputs = RESEARCH / 'train_dataset'
    meta = json.loads((inputs/'episode.json').read_text())
    rows = sorted(load_rows(inputs/'steps.jsonl'), key=lambda r:r['step_index'])
    hz = float(meta['record_frequency_hz'])
    labels = [float(r['action'][6]) for r in rows]
    close = next((i for i,g in enumerate(labels) if g >= .7), None)
    targets = [r.get('control',{}).get('gripper_closedness_target') for r in rows]
    target_close = next((i for i,g in enumerate(targets) if g is not None and g >= .7), None)
    timing = dict(episode_id=inputs.name, record_frequency_hz=hz,
                  demonstration_type=meta['demonstration_type'], num_steps_declared=meta['num_steps'],
                  num_steps_actual=len(rows), first_close_step=close,
                  first_close_time_s=None if close is None else close/hz,
                  close_ratio=sum(g >= .7 for g in labels)/len(labels),
                  first_control_target_close_step=target_close,
                  label_target_mismatch_count=sum(g != t for g,t in zip(labels,targets)),
                  instruction=meta['language_instruction'], target_color=meta['target_cube_color'],
                  timestamp_wall_elapsed_to_close_s=(rows[close]['timestamp_ns']-rows[0]['timestamp_ns'])/1e9,
                  sim_time_nonmonotonic_count=sum(b['sim_time_s']<=a['sim_time_s'] for a,b in zip(rows,rows[1:])))
    write_csv(out/'train_gripper_timing.csv',[timing])
    write_csv(out/'train_step_labels.csv',[
        dict(step_index=r['step_index'], nominal_time_s=i/hz, action_gripper=g,
             planner_phase=r['control']['planner_phase'], gripper_closedness_target=t,
             timestamp_ns=r['timestamp_ns'], sim_time_s=r['sim_time_s'])
        for i,(r,g,t) in enumerate(zip(rows,labels,targets))])
    chunks = future_grippers(labels)
    train = [dict(k_index=k, horizon_offset_s=k/hz, **distribution([c[k] for c in chunks])) for k in range(5)]
    write_csv(out/'train_k5_distribution.csv',train)
    prediction = []
    comparison = []
    timing_comparison = []
    sources = [
        ('PHASE10_EPISODE4', [RESEARCH/'phase10_planner_based_shadow_mode/06_shadow_collection/offline_recorded_episode4/oft_vision_step28560_local/samples.jsonl']),
        ('PHASE11_OFT_RUN2', sorted((RESEARCH/'phase11_sim_condition_replication/policy_sensitive_analysis/01_predictions/oft_run2').glob('*/*/samples.jsonl'))),
    ]
    provenance = []
    for source,paths in sources:
        by_k = [[] for _ in range(5)]
        for path in paths:
            saved = load_rows(path)
            valid = [r for r in saved if r.get('oft_denormalized_action_chunk') and r.get('valid') and not r.get('fixture_smoke_test_only')]
            expanded = []
            for r in valid:
                acts = r['oft_denormalized_action_chunk']
                if len(acts)!=5:
                    raise ValueError(f'Invalid K: {path}')
                for k,a in enumerate(acts):
                    by_k[k].append(float(a[6]))
                    expanded.append((int(r['frame_id'])+k,float(a[6])))
            first = next((step for step,g in sorted(expanded) if g>=.7),None)
            if first is not None:
                lead_s=timing['first_close_time_s']-first*.2
                timing_comparison.append(dict(source=source, episode_id=saved[0]['episode_id'],
                    condition=saved[0].get('condition_id','real_recorded'), prediction_hz=5,
                    first_prediction_step=first, first_prediction_time_s=first*.2,
                    training_first_close_step=close, training_hz=hz,
                    training_first_close_time_s=timing['first_close_time_s'],
                    lead_seconds=lead_s, lead_steps_at_prediction_5hz=lead_s*5,
                    matched_observation_comparison=False))
            provenance.append(dict(path=str(path),sha256=hashlib.sha256(path.read_bytes()).hexdigest(),valid_chunks=len(valid)))
        for k in range(5):
            stat=distribution(by_k[k])
            prediction.append(dict(source=source,k_index=k,horizon_offset_s=k*.2,**stat))
            comparison.append(dict(source=source,k_index=k,train_mean=train[k]['mean'],prediction_mean=stat['mean'],
                mean_difference=stat['mean']-train[k]['mean'],
                train_close_ratio=train[k]['close_ratio_ge_0p7'],prediction_close_ratio=stat['close_ratio_ge_0p7'],
                close_ratio_difference=stat['close_ratio_ge_0p7']-train[k]['close_ratio_ge_0p7'],
                train_horizon_offset_s=k/hz,prediction_horizon_offset_s=k*.2,
                matched_horizon_duration=(k==0)))
    write_csv(out/'prediction_k5_distribution.csv',prediction)
    write_csv(out/'train_vs_prediction.csv',comparison)
    write_csv(out/'early_close_comparison.csv',timing_comparison)
    summary=dict(classification='INSUFFICIENT_TRAINING_DATA',training=timing,
        training_episode_count=1,nominal_episode_count=0,corrective_recovery_episode_count=1,
        checkpoint_training_membership='UNVERIFIED',
        checkpoint='oft_mixed480_step28560 / oftplus_h5_vision',
        train_k0_to_k4_close_ratio_increase=train[4]['close_ratio_ge_0p7']-train[0]['close_ratio_ge_0p7'],
        observed_prediction_k_increase={s: next(r['close_ratio_ge_0p7'] for r in prediction if r['source']==s and r['k_index']==4)-next(r['close_ratio_ge_0p7'] for r in prediction if r['source']==s and r['k_index']==0) for s,_ in sources},
        limitations=['One blue corrective episode; predictions use orange real observations.',
            '10 Hz K5 spans 0.4 s; 5 Hz K5 spans 0.8 s. K-index distributions have different physical horizons.',
            'Episode sample-weighted prevalence depends on episode length and post-close hold duration.',
            'Exact mixed480 membership and any resampling from this source episode are unverified.',
            'sim_time_s resets; nominal close time uses record_frequency_hz. Wall timestamps differ.'],
        answers={
            'label_early_close':'Cannot determine premature intent from elapsed time alone. action close at move_grasp step28; control target switches at step29. Transition-aligned label needs source-generation review.',
            'k_close_ratio_increases':True,
            'prediction_increase_larger':'Yes descriptively: see CSV; different task, domain and Hz prevent causal attribution.',
            'checkpoint_amplifies_training_bias':'Not established with one unmatched training episode and unequal horizons.',
            'rollout_recommendation':'Keep phase safety gate. Audit exact mixed480 train/val corpus and compare checkpoints on matched observations before deciding on calibration/retraining.'},
        input_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in [inputs/'episode.json',inputs/'steps.jsonl']},
        prediction_provenance=provenance,production_safety_threshold_changed=False,
        execution_counts={k:0 for k in ['robot_command','motion_service_action','gripper_command','home','trajectory','hold_estop','real_rollout']})
    (out/'summary.json').write_text(json.dumps(summary,indent=2,ensure_ascii=False)+'\n')
    table='\n'.join(f"| {r['source']} | {r['k_index']} | {r['train_close_ratio']:.4f} | {r['prediction_close_ratio']:.4f} | {r['close_ratio_difference']:+.4f} |" for r in comparison)
    report=f'''# Training label vs OFT prediction

판정: `INSUFFICIENT_TRAINING_DATA`.

제공된 실제 데이터는 1 episode: blue cube / corrective_recovery / {hz:g} Hz / {len(rows)} steps.
Checkpoint mixed480 학습에 포함됐음을 증명하는 manifest는 미확인입니다.

## Training label timing

action[6]은 0=open, 1=closed. 최초 >=0.7은 step {close}, nominal {close/hz:.1f} s입니다.
control.gripper_closedness_target은 step {target_close}부터 close입니다. 1 row 차이는
observation_t→t+1 action alignment와 일치할 가능성이 있으나 생성 코드와 대조는 미완료입니다.
해당 step phase가 move_grasp이므로 경과 시간만으로 premature라고 판단할 수 없습니다.
Wall timestamp 경과는 {timing['timestamp_wall_elapsed_to_close_s']:.3f} s입니다.
sim_time_s는 {timing['sim_time_nonmonotonic_count']}회 비단조이므로 경과 계산에 사용하지 않았습니다.

## K=5 comparison

Terminal padding은 loader처럼 마지막 gripper 값을 유지합니다. 전체 t를 동일 가중치로 집계합니다.
Training K0→K4 ratio: {train[0]['close_ratio_ge_0p7']:.4f}→{train[4]['close_ratio_ge_0p7']:.4f}。
10 Hz training horizon은 0.4 s, 5 Hz prediction horizon은 0.8 s입니다.
이 표의 K 대응은 index 비교이며 동일 물리 horizon의 paired comparison은 아닙니다.

| Source | K | Training close ratio | Prediction close ratio | Prediction − training |
|---|---:|---:|---:|---:|
{table}

## Early-close comparison

Phase10 Episode4 prediction 최초 close는 step2 / 0.4 s입니다.
이번 training label 2.8 s와 시간차는 2.4 s, 5 Hz 환산 12 step입니다.
기존 4.8 s는 Episode4 자체 Scripted Reference Command 5.2 s와의 차이입니다.
두 값은 서로 다른 비교입니다. Raw 28−2 step은 Hz가 달라 사용하지 않습니다.
전체 episode 시간차는 results/early_close_comparison.csv에 저장했습니다.

## 해석과 다음 작업

K 후반의 close label 비율은 증가하며 prediction 증가폭은 관측상 더 큽니다.
다만 blue corrective / orange real 차이, horizon 차이, episode 길이·hold 시간 차이 때문에
checkpoint가 training bias를 증폭했다는 인과 결론은 낼 수 없습니다.
제공 데이터의 nominal episode는 0, corrective는 1로 demonstration 비교는 미실시입니다.
Exact mixed480 corpus와 resampling 기록을 확보하고 동일 observation의 checkpoint 비교로
calibration/retraining 필요성을 판단해야 합니다. 현재 phase safety gate는 유지합니다.

## Reproduction and safety

`python3 phase12_real_policy_rollout/12_train_label_vs_oft_prediction/analyze_train_vs_prediction.py`

입력 hash와 prediction provenance는 summary.json에 저장했습니다. 신규 inference는 없습니다.
Production safety threshold 변경 없음. Robot/motion/gripper/Home/trajectory/Hold/E-stop/real rollout 모두 0회.
'''
    (ROOT/'README.md').write_text(report)
    print(json.dumps(dict(training=timing,classification=summary['classification'],prediction_k5=prediction),indent=2))


if __name__=='__main__':
    main()
