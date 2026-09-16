# Projector Perturbation Feasibility

[Purpose]
Check whether `projector.output` perturbation can be injected into the actual downstream policy forward path.

[Hypothesis]
If `projector.output` can be patched before multimodal token construction, we can evaluate `z_real_proj + alpha * delta_proj` through the LLM and action head to measure upstream policy relevance directly.

[Inputs]
- Feature files contain:
  - `vision_backbone.output`: `(1, 256, 2176)`
  - `projector.output`: `(1, 256, 4096)`
  - `action_hidden_states.input`: `(1, 35, 4096)`
  - `action_head.output`: `(1, 5, 7)`
- Model code:
  - `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/runtime_state/oft_mixed480_step28560_merged/modeling_prismatic.py`
  - `PrismaticForConditionalGeneration.forward`
  - `OpenVLAForActionPrediction.predict_action`
- Extraction code:
  - `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/sim2real_analysis/04_features/extract_vla_features.py`

[Checked]
- `projector.output` corresponds to `projected_patch_embeddings` from `_process_vision_features`.
- Downstream path after projector is:
  `projected_patch_embeddings -> _build_multimodal_attention -> language_model -> action hidden states -> action_head`.
- Saved projector tensor cannot be sent directly to the standalone action head.
- Exact perturbation requires full VLM forward with hook/patch at `_process_vision_features`.
- Current execution environment reports `CUDA_ERROR_NO_DEVICE` during torch/tensorflow initialization in previous Phase5 runs.

[Decision]
Direct projector perturbation through the actual full VLM forward is `UNVERIFIED / BLOCKED` in the current execution environment because it likely requires loading/running the full 7B VLM. It should not be approximated by feeding projector tokens directly to the action head.

[Next]
Use stored paired tensors for an offline upstream tracing surrogate:

`projector.output delta -> action_hidden_states.input policy-sensitive delta -> action effect`

This surrogate is not a replacement for exact full-forward injection, but it can test whether upstream projector differences linearly predict the policy-sensitive hidden component.

[Status]
UNVERIFIED / RE-RUN REQUIRED for exact projector perturbation.

[Problems]
- Full VLM forward hook not executed.
- No direct deployment claim.
- Surrogate tracing must be labeled as offline tensor-level attribution, not exact causal forward injection.
