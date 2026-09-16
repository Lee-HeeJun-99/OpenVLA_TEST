# Phase 6 Limitations

- Full image -> final action forward could not run in the current session because CUDA is unavailable.
- Current Phase 6 numeric results are observation-side only.
- Environment/preprocessing attribution to policy behavior is not verified until full-forward extraction completes.
- No camera, geometry, or appearance causal claim is made.
- No Real robot motion or hardware change was performed.
- `oftplus_h5_proprio` / ROS deployment is not mixed with this offline `oftplus_h5_vision` track.
- Sensitive/null energy columns are not populated yet because the serialized Phase5 sensitive basis is not available in the current Phase6 output set.
