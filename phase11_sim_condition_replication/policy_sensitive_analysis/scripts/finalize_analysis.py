#!/usr/bin/env python3
import csv,json
from collections import defaultdict
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt

R=Path(__file__).resolve().parents[1]; STAT=R/'08_statistics'; REC=R/'10_rollout_recommendation'; FIG=R/'09_figures'; FINAL=R/'final_report'
for p in (STAT,REC,FIG,FINAL): p.mkdir(exist_ok=True)
def read(p): return list(csv.DictReader(open(p)))
def write(p,rr):
 with open(p,'w',newline='') as f: w=csv.DictWriter(f,fieldnames=rr[0].keys());w.writeheader();w.writerows(rr)
ep=read(R/'06_action_gap/episode_level_action_metrics.csv'); grip=read(R/'06_action_gap/gripper_condition_summary.csv'); pol=read(R/'05_policy_sensitive/policy_sensitive_condition_summary.csv'); frame=read(R/'06_action_gap/frame_level_action_metrics.csv')
rng=np.random.default_rng(20261001); boot=[]
for key in sorted({(x['model'],x['condition']) for x in ep}):
 vals=np.array([float(x['condition_shift_standardized_l2_mean']) for x in ep if (x['model'],x['condition'])==key]); sims=np.array([rng.choice(vals,len(vals),replace=True).mean() for _ in range(10000)])
 boot.append({'model':key[0],'condition':key[1],'episodes':len(vals),'mean':float(vals.mean()),'ci95_low':float(np.percentile(sims,2.5)),'ci95_high':float(np.percentile(sims,97.5)),'effect_vs_zero_standardized':float(vals.mean()/max(vals.std(ddof=1),1e-12))})
write(STAT/'action_shift_trial_bootstrap.csv',boot)
phase=[]
for key in sorted({(x['model'],x['condition'],x['phase']) for x in frame}):
 ss=[x for x in frame if (x['model'],x['condition'],x['phase'])==key]; fp=sum(int(x['reference_binary_close'])==0 and int(x['condition_binary_close'])==1 for x in ss); fn=sum(int(x['reference_binary_close'])==1 and int(x['condition_binary_close'])==0 for x in ss)
 phase.append({'model':key[0],'condition':key[1],'phase':key[2],'n':len(ss),'false_close':fp,'false_open':fn,'false_close_rate':fp/max(sum(int(x['reference_binary_close'])==0 for x in ss),1),'action_shift_standardized_mean':float(np.mean([float(x['condition_shift_standardized_l2']) for x in ss]))})
write(R/'07_gripper_scale_audit/gripper_phase_semantic_errors.csv',phase)

conds=['distractor_swap','extra_object','lighting_low']; models=['openvla','oft']
plt.figure(figsize=(8,4)); x=np.arange(3); w=.36
for j,m in enumerate(models):
 vals=[next(float(z['mean']) for z in boot if z['model']==m and z['condition']==c) for c in conds]; lo=[next(float(z['ci95_low']) for z in boot if z['model']==m and z['condition']==c) for c in conds]; hi=[next(float(z['ci95_high']) for z in boot if z['model']==m and z['condition']==c) for c in conds]
 plt.bar(x+(j-.5)*w,vals,w,label=m,yerr=[np.array(vals)-lo,hi-np.array(vals)],capsize=3)
plt.xticks(x,conds);plt.ylabel('training-stat standardized action shift');plt.title('Episode-level paired bootstrap (95% CI)');plt.legend();plt.tight_layout();plt.savefig(FIG/'episode_paired_bootstrap.png',dpi=160);plt.close()
plt.figure(figsize=(8,4));
for j,m in enumerate(models):
 vals=[next(int(z['fp']) for z in grip if z['model']==m and z['condition']==c) for c in conds];plt.bar(x+(j-.5)*w,vals,w,label=m)
plt.xticks(x,conds);plt.ylabel('false-close frames');plt.title('Gripper semantic false-close count');plt.legend();plt.tight_layout();plt.savefig(FIG/'gripper_false_close.png',dpi=160);plt.close()

recommend=[
 {'category':'LOW_POLICY_IMPACT','condition':'distractor_swap','basis':'lowest scale-normalized action shift for both models'},
 {'category':'HIGH_POLICY_IMPACT','condition':'extra_object','basis':'highest mean scale-normalized action shift for both models'},
 {'category':'PHASE_SPECIFIC_IMPACT','condition':'lighting_low','basis':'OFT gripper false-close concentrated before/reference grasp despite not highest normalized total shift'}]
write(REC/'rollout_condition_recommendation.csv',recommend)
(REC/'rollout_recommendation.md').write_text('''# Phase 11 rollout condition recommendation\n\n- LOW_POLICY_IMPACT: `distractor_swap`.\n- HIGH_POLICY_IMPACT: `extra_object`.\n- PHASE_SPECIFIC_IMPACT: `lighting_low`, focused on pre-grasp/gripper-close timing.\n\nThese are offline hypotheses, not rollout outcomes. Use paired layouts, record reach/alignment/grasp/lift failure, close timing, minimum cube–gripper distance, timeout and hold. A practical exploratory design is at least 10 paired rollouts per selected condition and model; power should be recalculated from pilot outcomes.\n''')
summary={'status':'COMPLETED_PHASE11_POLICY_SENSITIVE_ANALYSIS','valid_pairs':15,'aligned_samples':684,'actual_predictions':{'openvla':912,'oft_k5_inferences':192},'feature_records':1824,'feature_pair_layer_rows':3420,'recommendations':recommend,'limitations':['five paired episode IDs only','OpenVLA action-hidden hook unavailable; projector proxy used','train-selected subspace did not consistently outperform random projection','offline prediction does not establish closed-loop success']}
(FINAL/'analysis_summary.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary,indent=2))
