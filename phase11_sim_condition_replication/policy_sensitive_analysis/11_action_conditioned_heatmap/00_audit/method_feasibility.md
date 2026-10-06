# Method feasibility audit

- Stored Phase 11 NPZ files contain vision/projector activations and, for OFT, action-hidden/action-head activations. They do **not** contain per-layer attention matrices or input gradients.
- Existing OpenVLA inference runs under `torch.inference_mode()` and discrete action generation breaks a direct denormalized-action gradient path.
- Existing OFT `get_vla_action()` also wraps prediction in `torch.inference_mode()`. A custom differentiable forward must be validated before gradient or Integrated Gradients output can be accepted.
- Attention rollout requires a new instrumented model pass and validation of the actual ViT token/head convention. Stored projector tokens must not be mislabeled as attention.
- Occlusion sensitivity is supported through prediction-only HTTP servers and measures actual output intervention effects. It remains susceptible to occlusion-distribution artifacts, so local-mean and Gaussian-blur replacements are both used.
- Until instrumented attention/gradient passes are produced, the honest overall state is partial attribution rather than a completed three-method triangulation.
