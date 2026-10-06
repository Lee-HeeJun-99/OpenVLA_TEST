"""Recursive training re-audit; preserves the original single-episode results."""
import json
import hashlib
import statistics
from collections import defaultdict
from analyze_train_vs_prediction import ROOT, RESEARCH, load_rows, future_grippers, distribution, write_csv


def main():
    out=ROOT/'results_added_data'
    out.mkdir(exist_ok=True)
    timing=[]; seen={}; duplicates=[]; groups=defaultdict(list); manifest=[]
    for p in sorted((RESEARCH/'train_dataset').rglob('episode.json')):
        s=p.with_name('steps.jsonl')
        digest=hashlib.sha256(s.read_bytes()).hexdigest()
        if digest in seen:
            duplicates.append({'path':str(s),'duplicate_of':seen[digest]});continue
        seen[digest]=str(s)
        m=json.loads(p.read_text());r=sorted(load_rows(s),key=lambda x:x['step_index'])
        assert len(r)==m['num_steps']
        assert [x['step_index'] for x in r]==list(range(len(r)))
        hz=float(m['record_frequency_hz']);g=[float(x['action'][6]) for x in r]
        assert all(0<=x<=1 for x in g)
        f=next((i for i,x in enumerate(g) if x>=.7),None)
        typ=m['demonstration_type'];chunks=future_grippers(g)
        groups['ALL'].extend(chunks);groups[typ].extend(chunks)
        control=[x['control']['gripper_closedness_target'] for x in r]
        timing.append(dict(episode_path=str(p.parent),demonstration_type=typ,target_color=m['target_cube_color'],
            record_frequency_hz=hz,num_steps=len(r),first_close_step=f,first_close_time_s=f/hz if f is not None else None,
            close_ratio=distribution(g)['close_ratio_ge_0p7'],first_close_phase=r[f]['control']['planner_phase'] if f is not None else None,
            first_control_close_step=next((i for i,v in enumerate(control) if v>=.7),None),
            label_control_mismatch_count=sum(a!=b for a,b in zip(g,control)),steps_sha256=digest))
        manifest.append(dict(episode_json=str(p),episode_sha256=hashlib.sha256(p.read_bytes()).hexdigest(),steps_jsonl=str(s),steps_sha256=digest))
    train=[dict(demonstration_type=typ,k_index=k,horizon_offset_s=k/10.0,**distribution([c[k] for c in chunks])) for typ,chunks in groups.items() for k in range(5)]
    # All provided episodes are 10 Hz; explicit assertion prevents mixed-Hz pooling.
    assert {t['record_frequency_hz'] for t in timing}=={10.0}
    stats=[]
    for typ in groups:
        vals=[x['first_close_time_s'] for x in timing if typ=='ALL' or x['demonstration_type']==typ]
        stats.append(dict(demonstration_type=typ,episode_count=len(vals),min_time_s=min(vals),median_time_s=statistics.median(vals),mean_time_s=statistics.mean(vals),max_time_s=max(vals)))
    import csv
    predictions=list(csv.DictReader((ROOT/'results/prediction_k5_distribution.csv').open()))
    comparison=[]
    for x in predictions:
        k=int(x['k_index']);t=next(t for t in train if t['demonstration_type']=='ALL' and t['k_index']==k)
        comparison.append(dict(source=x['source'],k_index=k,training_close_ratio=t['close_ratio_ge_0p7'],prediction_close_ratio=float(x['close_ratio_ge_0p7']),difference=float(x['close_ratio_ge_0p7'])-t['close_ratio_ge_0p7'],training_mean=t['mean'],prediction_mean=float(x['mean']),training_horizon_s=k/10,prediction_horizon_s=k/5))
    write_csv(out/'train_gripper_timing.csv',timing);write_csv(out/'train_k5_distribution.csv',train)
    write_csv(out/'demonstration_timing_summary.csv',stats);write_csv(out/'train_vs_prediction.csv',comparison)
    summary=dict(unique_episode_count=len(timing),total_steps=sum(x['num_steps'] for x in timing),duplicates=duplicates,
        demonstration_summary=stats,classification='INSUFFICIENT_TRAINING_DATA',
        supported_findings=['K5 label close prevalence rises','Checkpoint K rise is larger descriptively'],
        unresolved=['Exact mixed480 membership/resampling unverified','Unmatched blue/red/yellow sim versus orange real inputs','10 Hz versus 5 Hz horizons','Only one nominal episode'],
        provenance=manifest,production_config_changed=False,robot_command_count=0)
    (out/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    table='\n'.join(f"| {x['source']} | K{x['k_index']} | {x['training_close_ratio']:.3%} | {x['prediction_close_ratio']:.3%} | {x['difference']*100:+.2f}%p |" for x in comparison)
    overall=stats[0]
    report=f'''# 추가 학습 데이터 재감사

원본 single-episode 분석은 보존하고 이번 결과는 results_added_data에 저장했습니다.

고유 episode {len(timing)}개, {summary['total_steps']} steps. 모두 10 Hz.
corrective_recovery 10개, nominal 1개. SHA-256 중복 {len(duplicates)}개.
첫 close는 {overall['min_time_s']}–{overall['max_time_s']}초, 중앙값 {overall['median_time_s']}초입니다.
Nominal은 step32/3.2초, corrective 중앙값은 2.45초입니다.

| Source | K | Training close ratio | OFT close ratio | 차이 |
|---|---:|---:|---:|---:|
{table}

Training은 실제 label이며 reference proxy와 구분했습니다. Terminal은 loader처럼 마지막
gripper 상태를 유지합니다. 통계는 전체 timestep 가중치 기준입니다.
Training K5는 0.4초, prediction K5는 0.8초 horizon이므로 K별 차이는 기술적 비교입니다.
Phase10 Episode4 첫 close 0.4초는 이번 training 중앙값 2.5초보다 2.1초 빠르지만,
동일 영상·시작 상태 비교가 아니므로 조기 close 원인의 인과 증거로 사용할 수 없습니다.

판정: INSUFFICIENT_TRAINING_DATA. Label K 증가와 더 큰 prediction K 증가가 관찰됐지만
정확한 mixed480 포함 여부·resampling, 동일 task/input 비교가 미확인입니다.
Nominal 1개로 demonstration type 일반화를 할 수 없습니다.
재학습 결정 전 exact training split 및 같은 입력의 checkpoint별 prediction을 확보해야 합니다.
Production threshold 변경 없음. 신규 inference·robot command 모두 0회.
'''
    (ROOT/'added_data_report.md').write_text(report)
    print(json.dumps(summary,indent=2))


if __name__=='__main__':main()
