# Priority2 Limitations

- New image-to-final-action full-forward was not executed because `nvidia-smi` could not communicate with the NVIDIA driver.
- Camera shift and letterbox ablation therefore have prepared inputs and reproducible manifests, but no generated action numbers.
- Action-head-only alignment operates on stored `action_hidden_states.input`; it is not equivalent to full VLM inference from modified images.
- Existing older correction artifacts generally lack saved corrected action chunks; exact full multi-metric recomputation requires action-head rerun or regenerated chunks.
- No Priority3 data collection, Shadow Mode, closed-loop rollout, or Real robot performance evidence was produced.
