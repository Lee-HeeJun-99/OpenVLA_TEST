# Deployability Summary

[Status]
Offline only. No Real robot or runtime-integrated correction has been executed.

[Most deployability-compatible current result]
Train-selected progress gate / phase-aware progress correction.

[Why]
- Does not require paired Sim hidden at each deployment frame if a progress/phase estimator is available.
- `progress_gate_lambda` held-out gap: `0.073059`, close to unconditional progress correction `0.072325`, with fewer worsened frames.
- `progress_gate_zero_worse` is more conservative: gap `0.081698`, worsened `1`.

[Not deployable as-is]
- Full paired Δh correction.
- Sensitive projection.
- Low-rank sensitive basis projection using held-out Δh.
- Projector/vision surrogate maps requiring Real→Sim paired deltas.

[Next deployability step]
Build a correction module that uses only Real observation metadata available at runtime: progress estimate, phase estimate, or confidence/uncertainty proxy.
