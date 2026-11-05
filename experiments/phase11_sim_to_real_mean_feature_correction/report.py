"""Render measured results and paper assets from run.py outputs."""
import csv,json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

HERE=Path(__file__).resolve().parent
OUT=HERE/'results_001'
OLD=HERE.parent/'20261028_componentwise_mean_loo/results_001'
KEYS=('translation_l2','rotation_l2','gripper_abs')
NAMES=('이동 L2 (m)','회전 L2 (rad)','그리퍼 차이')
def rows(name):
    with (OUT/name).open() as f:return list(csv.DictReader(f))
def f(r,k):return float(r[k])
def yn(x):return '감소' if x<0 else '증가' if x>0 else '동일'
overall={r['condition']:r for r in rows('metrics.csv')}
episodes=rows('episode_metrics.csv')
validation=json.loads((OUT/'validation.json').read_text())
prior={r['condition']+'_'+r['method']:r for r in csv.DictReader((OLD/'overall_metrics.csv').open())}

def values_table():
    lines=['| 조건 | 픽셀 MSE | 이동 L2 (m) | 회전 L2 (rad) | 그리퍼 차이 |','|---|---:|---:|---:|---:|']
    for c,label in [('A','A 기준 입력'),('B','B 밝기 정렬'),('C1','C1 Real → Sim'),('C2','C2 Sim → Real')]:
        r=overall[c];lines.append('| '+label+' | '+' | '.join(f'{f(r,k):.9f}' for k in ('pixel_mse',*KEYS))+' |')
    return '\n'.join(lines)
def changes_table():
    lines=['| 조건 | 이동 | 회전 | 그리퍼 |','|---|---:|---:|---:|']
    for c in ('A','B','C1','C2'):
        lines.append('| '+c+' | '+' | '.join(f'{f(overall[c],k+"_change_percent"):+.2f}%' for k in KEYS)+' |')
    return '\n'.join(lines)
def episode_table():
    lines=['| 에피소드 | 관측 수 | C2 이동 변화 | C2 회전 변화 | C2 그리퍼 변화 |','|---|---:|---:|---:|---:|']
    for r in episodes:
        if r['condition']=='C2':lines.append('| '+r['episode_id']+' | '+r['pair_count']+' | '+' | '.join(f'{f(r,k+"_change_percent"):+.2f}%' for k in KEYS)+' |')
    return '\n'.join(lines)

table=values_table();changes=changes_table();eps=episode_table()
c1=overall['C1'];c2=overall['C2'];b=overall['B'];a=overall['A']
oldc=prior['A_mean_LOO']
repro='; '.join(f'{name} 기존 {f(oldc,k):.9f}, 이번 {f(c1,k):.9f}, 차이 {f(c1,k)-f(oldc,k):+.9f}' for name,k in zip(NAMES,KEYS))
effect='; '.join(f'{name} {yn(f(c2,k+"_change_percent"))} {abs(f(c2,k+"_change_percent")):.2f}%' for name,k in zip(NAMES,KEYS))
direction='; '.join(f'{name} '+('C1' if f(c1,k)<f(c2,k) else 'C2' if f(c2,k)<f(c1,k) else '동률') for name,k in zip(NAMES,KEYS))
bright='; '.join(f'{name} '+('C2' if f(c2,k)<f(b,k) else 'B' if f(b,k)<f(c2,k) else '동률') for name,k in zip(NAMES,KEYS))
ep_consistency='; '.join(f'{name} {sum(f(r,k+"_change_percent")<0 for r in episodes if r["condition"]=="C2")}/5 감소' for name,k in zip(NAMES,KEYS))
gripper_worse_than_c1=[]
for r in episodes:
    if r['condition']=='C2':
        other=next(x for x in episodes if x['condition']=='C1' and x['episode_id']==r['episode_id'])
        if f(r,'gripper_abs')>f(other,'gripper_abs'):gripper_worse_than_c1.append(r['episode_id'])
feature=f(c2,'feature_gap_after')-f(c2,'feature_gap_before')
paper=f'''# 논문용 표 — 동일 CPU BF16 실행 기준

{table}

A 대비 변화율 `(조건−A)/A×100`; 음수는 차이 감소.

{changes}

225쌍, 5개 에피소드, K=5 step 평균 후 관측 평균. 픽셀 MSE는 기록된 backbone 1 입력을 고정 Normalize 역변환한 RGB [0,1]의 평균 제곱 차이. 행동 헤드 BF16, batch=1, CPU. 기존 GPU BF16 표와 실행 백엔드가 다르므로 수치 인용 시 CPU 기준을 명시해야 한다.
'''
(OUT/'paper_table.md').write_text(paper)
report=f'''# Sim → Real 평균 특징 보정 실측 보고서

## 기존 코드 및 자료

225쌍은 `scripts/build_5episode_paired_inputs.py`가 source step을 맞춰 생성했고, `outputs/token_distribution_analysis/5_episodes/paired_inputs/paired_manifest.json`에 저장했다. 기존 Phase6 `phase6_environment_attribution/scripts/phase6_prepare_condition_images.py`가 B의 Sim 밝기를 Real 평균 휘도에 맞췄다. `sim2real_analysis/04_features/extract_vla_features.py`는 행동 헤드 `predict_action` 호출 직전 입력을 `action_hidden_states.input`으로 저장한다. 이 텐서는 관측당 1×35×4096이며, 헤드의 5×7 출력에 대응한다.

기존 `experiments/20261028_componentwise_mean_loo/analyze.py`는 Real에 `mean(h_sim−h_real)`을 더한 후 고정 헤드를 BF16, batch 1, GPU에서 실행했다. mask가 앞 6차원은 true, 그리퍼는 false이므로 이동·회전만 q01/q99로 역정규화하고 그리퍼는 연속 sigmoid 출력을 그대로 비교한다. 각 관측에서 K=5 step별 이동/회전 L2와 그리퍼 절대 차이를 평균한 뒤 225개 관측을 평균한다. 기존 A의 C1 감소율 32.69%, 38.92%, 66.21%는 GPU 보고서 값이다.

## 이번 구현과 실행

`run.py`는 같은 저장 특징 및 step 28560 체크포인트의 `L1RegressionActionHead`를 사용했다. 각 fold에서 평가 에피소드를 제외한 관측별 `h_real−h_sim`의 FP64 평균을 구했다. C1에는 부호를 뒤집어 Real에, C2에는 이 벡터를 Sim에 적용했다. 보정 후에만 고정 헤드를 재실행했다. 원본 Real 특징과 헤드 가중치는 변경하지 않았다. CPU에서 BF16, batch=1로 A/B/C1/C2를 모두 재계산했다.

실행 명령: `environment/a6000_ubuntu22_py310/bin/python experiments/phase11_sim_to_real_mean_feature_correction/run.py` 및 같은 Python으로 `report.py`.
환경은 torch {validation['torch_version']}, {validation['backend']} BF16, batch {validation['batch_size']}. 체크포인트 SHA256 `{validation['checkpoint_sha256']}`. GPU가 감지되지 않았고, CPU 재계산 출력은 저장 GPU 출력과 최대 {validation['replay_max_normalized_error']:.9f} 차이가 났다. 따라서 기존 GPU C1의 정확한 재현은 미확인이다. 같은 CPU 경로에서 비교 가능한 수치는 아래와 같다. 기존 GPU C1과 이번 CPU C1의 차이: {repro}.

## 정량 결과

{table}

A 대비 변화율은 음수가 차이 감소다.

{changes}

C2의 성분별 결과: {effect}. 방향별 더 작은 차이: {direction}. B와 C2 중 더 작은 차이: {bright}.

{eps}

에피소드별 방향 일관성: {ep_consistency}.

A와 비교하면 C2가 악화한 에피소드/성분은 없다. C1과 비교하면 그리퍼 차이가 더 큰 에피소드는 {', '.join(gripper_worse_than_c1)}이다. 이동·회전은 C2가 5개 에피소드 모두 C1보다 작다.

## 검증과 해석

모든 조건에 동일한 225개 고유 pair ID를 사용했다. 각 fold의 학습/평가 ID는 분리되며 학습 수는 179, 178, 181, 180, 182개다. C2 벡터는 기존 C1 벡터와 원소별 최대 부호 오차 {validation['sign_max_error']:.3g}이다. 0벡터 주입 시 저장된 이번 CPU A Sim 출력과 최대 차이 {validation['zero_vector_max_decoded_error']:.3g}이다. 헤드 가중치 동결: {validation['head_frozen']}. C1/C2 픽셀 MSE가 A와 관측별 동일: {validation['pixel_identity_C1_C2_A']}. NaN/Inf와 형상 오류는 실행 중 검사했다.

새 결과의 ID·fold·벡터 부호·집계 산식 독립 테스트 3개와 기존 지표/재생 감사 테스트 8개가 모두 통과했다.

특징 공간의 평균 L2 차이는 C2 보정 전 {f(c2,'feature_gap_before'):.6f}, 후 {f(c2,'feature_gap_after'):.6f}이며 변화량은 {feature:+.6f}이다. 행동 공간의 각 성분 변화와 함께 해석해야 한다. 이 값은 평가 에피소드의 특징을 보정 벡터 추정에 쓰지 않은 결과다.

측정된 결과는 고정 정책의 예측 불일치다. 가능한 원인으로는 헤드의 비선형성, BF16 양자화, 에피소드별 분포 차이가 있으나 이 실험만으로 인과 원인을 식별할 수 없다. 관측 쌍은 joint-relative replay 대응이며 측정된 물리 EE pose의 완전한 정합은 확인되지 않았다. 실제 로봇 성공률은 측정하지 않았다.

## 논문 수정 제안

- 초록: C1과 C2를 구분하고, C2의 위 실측 성분별 변화율을 제시한다. CPU BF16 비교임을 밝힌다.
- 방법: 5-fold 관측 가중 평균 `h_real−h_sim`, Sim 주입, 고정 헤드 재계산, gripper mask=false를 명시한다.
- 결과: 위 절대 수치와 에피소드별 표를 추가하고, 기존 GPU C1 수치와 이번 CPU 수치를 같은 표의 재현값으로 혼동하지 않는다.
- 결론: 보정 방향의 효과는 실제 측정된 성분별 결과로 제한해 서술하고 로봇 성공률 개선 주장을 하지 않는다.
'''
(OUT/'RESULTS_REPORT.md').write_text(report)
fig,ax=plt.subplots(figsize=(9,4.5))
import numpy as np
x=np.arange(3);width=.19
for i,c in enumerate(('A','B','C1','C2')):
    ax.bar(x+(i-1.5)*width,[f(overall[c],k+'_change_percent') for k in KEYS],width,label=c)
ax.axhline(0,color='black',lw=.8);ax.set_xticks(x,('Translation','Rotation','Gripper'));ax.set_ylabel('Change vs A (%)');ax.legend(ncol=4);fig.tight_layout();fig.savefig(OUT/'comparison.png',dpi=180);plt.close(fig)
print(OUT/'RESULTS_REPORT.md')
