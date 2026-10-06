# Reuse audit

- Phase 8 `phase8_pair_alignment.py`: extended conceptually from fixed-index pairing to phase-normalized alignment.
- Phase 8 `phase8_observation_gap.py`: observation metric definitions reused; Phase 11 uses a local SSIM implementation because `skimage` is absent from Bundle Python.
- Phase 8 feature extraction launcher and `sim2real_analysis/04_features/extract_vla_features.py`: selected for future real-model feature extraction.
- Phase 5 `low_rank_sensitive_subspace.py`: selected as the basis for k={1,2,4,8,16,32,64,128} analysis once Phase 11 hidden tensors exist.
- Phase 5 `loo_gate_generalization.py`: selected for train-only phase/λ selection; its λ definition is an objective penalty on worsened rate, not the weighted score formula proposed in the request.
- Phase 10 OpenVLA/OFT adapters, canonicalizer, gripper semantics and prediction-only runner: reusable without ROS or command publishing.
- Phase 10 reference extractor/metrics: reusable with the label `Scripted Reference Command`; never Ground Truth Action.

Existing Phase 4–10 results were not copied into Phase 11 as new evidence. They are method/code references only.
