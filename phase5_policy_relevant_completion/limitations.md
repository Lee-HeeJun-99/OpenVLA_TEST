# Limitations

- All Phase5 results are offline action-head or tensor-level evidence.
- No Real robot motion or real-world success measurement was performed.
- Episode LOO is not unseen-layout or unseen-environment generalization.
- Exact projector/vision perturbation through full VLM forward remains `UNVERIFIED / RE-RUN REQUIRED` in this execution environment.
- Projector/vision upstream tracing used pooled-delta surrogate maps, not causal full-forward injection.
- Low-rank sensitive subspace uses held-out Real→Sim Δh projection and is not deployable as-is.
- Environment/preprocessing attribution was not executed; it requires full feature/action re-extraction under controlled preprocessing/factor changes.
- Token indices are not physical object or image-region attributions.
