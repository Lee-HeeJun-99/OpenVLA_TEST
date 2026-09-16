# Phase5 Policy-Relevant Completion Summary

## 1. Existing Verified Evidence
The 225-pair `oftplus_h5_vision` baseline remains the scope. Raw Action Gap is about `0.370245` chunk mean L2. Phase4 established that representation distance alone is not policy relevance.

## 2. Correction Robustness
LOO correction ablation showed structured corrections outperform controls:
- no correction: `0.370245`
- global mean: `0.125657`
- progress: `0.072325`
- phase: `0.066682`
- progress+phase: `0.061829`
- random matched-norm: `0.367535`
- shuffled progress: `0.130975`
- wrong direction: `0.485313`

## 3. Gate Generalization
Train-selected gates generalize under episode LOO:
- progress_all: `0.072325`, worsened `33`
- phase_gate_train: `0.072086`, worsened `14`
- progress_gate_lambda: `0.073059`, worsened `2`
- progress_gate_zero_worse: `0.081698`, worsened `1`
- oracle: `0.071488`, worsened `0`

## 4. Layer-wise Policy-Relevant Tracing
Action-hidden remains the strongest verified layer for policy-relevant correction. Low-rank sensitive basis at `action_hidden_states.input` achieved gap `0.006676` with only `0.030669` representation reduction.

## 5. Upstream Perturbation
Exact projector/vision full-forward perturbation is `UNVERIFIED / RE-RUN REQUIRED` because it requires full VLM forward injection. Feasibility blockers are documented.

## 6. Upstream Token/Component Findings
Surrogate tracing found upstream signal but weaker than hidden-level correction:
- projector pooled -> hidden sensitive surrogate: `0.145533`
- vision pooled -> hidden sensitive surrogate: `0.138268`

## 7. Environment Attribution
Not executed in Phase5. It requires controlled feature/action re-extraction under preprocessing/environment changes. Current status: `PLANNED / RE-RUN REQUIRED`.

## 8. Deployability-Compatible Correction
Most deployability-compatible current candidate is progress/phase-aware gated correction. It does not require paired Sim hidden at each frame if progress/phase can be estimated.

## 9. Low-Rank Sensitive Subspace
Low-rank sensitive subspace strongly supports the core hypothesis: a small policy-sensitive component can explain most Action Gap while reducing very little total representation distance. It is not deployable as-is because test Δh projection still uses paired Sim hidden.

## 10. Failure Analysis
Progress correction failures concentrate in low raw-gap phases (`grasp_close`, `lift`). Gating reduces over-correction.

## 11. Final Evidence Chain
Current evidence supports:
`Observation/representation gap -> policy-sensitive hidden component -> Action Gap`.
Upstream vision/projector deltas carry partial signal, but exact full-forward causal attribution remains unresolved.

## 12. Limitations
See `limitations.md`.

## 13. Next Real-World Experiments
Before real robot testing, build a runtime-compatible progress/phase gate and validate offline on more layouts. Real robot execution remains a separate approval-required phase.
