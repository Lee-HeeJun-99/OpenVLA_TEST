# Gripper semantics audit

## Conclusion

The step-1 event is not a de-normalization, canonicalization, or chunk-expansion bug. The OFT checkpoint is trained and served with `0=open, 1=closed` continuous closedness. Its first K=5 response is `[0.4219, 0.6953, 0.8438, 0.8984, 0.9258]`, so the dataset-semantics 0.5 threshold crosses at chunk index 1 / expanded step 1.

The deployed Real `ActionAdapterNode` has the opposite contract: its source documentation and `_resolve_gripper_state` treat high values as OPEN and low values as CLOSE, using 0.7/0.3 hysteresis. The Real OFT node also returns only `action_array[0]` and discards chunk indices 1–4. Thus step 1 is not a runtime-equivalent close event. Applying the actual runtime code to the saved responses gives a first OFT close at step 10 and reopening at step 25; this is still an incorrect early/transient close relative to Reference step 26.

## Stage trace

| Stage | Source File/Function | Input Range | Output Range | Open Value | Close Value | State/Delta | Threshold |
|---|---|---:|---:|---:|---:|---|---:|
| Model raw | `action_heads.py:L1RegressionActionHead.predict_action` | unbounded logit internally | sigmoid `[0,1]` for last dim | 0 | 1 | continuous closedness state target | none |
| De-normalized | `modeling_prismatic.py:_unnormalize_actions` | `[0,1]` gripper | unchanged `[0,1]` | 0 | 1 | continuous state | action-stat mask is false |
| Server response | `serve_a0509_oft.py:OFTRuntime.predict` | K=5×7 unnormalized action | JSON K=5×7 | 0 | 1 | continuous state per chunk step | none |
| Phase 10 adapter | `oft_adapter.py:predict_recorded` | server `actions` | same K=5 values | 0 | 1 | continuous state | none |
| Canonical | `action_canonicalizer.py:canonicalize_action` | `[0,1]` | unchanged `[0,1]` | 0 | 1 | `gripper_closedness` | validation only; no clipping |
| Offline binary diagnostic | `analyze_gripper_semantics.py` | `[0,1]` | Bool close | below 0.5 | at/above 0.5 | binary state | 0.5 |
| Real runtime OFT selection | `openvla_oft_inference_node.py:_predict_action` | K=5×7 | only chunk `[0]` | unchanged numeric value | unchanged numeric value | first chunk state | none |
| Real runtime postprocess | `action_adapter_node.py:_resolve_gripper_state` | continuous value | `gripper_open: bool` | value ≥0.7 | value ≤0.3 | hysteretic binary state | open 0.7 / close 0.3 |
| Hardware command | `DoosanBridgeNode`, `runtime_oft.yaml` | Bool open | tool digital pulse | `True`, output index 2 | `False`, output index 1 | event command; repeats suppressed upstream | state change only |

`-1` has no valid physical closedness meaning for this bounded checkpoint. The bounded action head prevents it for gripper. For dimensions whose statistics mask is true, BOUNDS_Q99 unnormalization maps normalized `[-1,1]` to `[q01,q99]`; gripper mask is false, so that formula is not applied to the last dimension.

The saved `raw_model_output.actions` is the server response after model unnormalization. Pre-unnormalization tensors were not logged and are `UNRESOLVED` per request. For gripper, source inspection proves the value is unchanged across unnormalization because the mask is false.

## Repetition and clipping

Phase 10 rejects closedness outside `[0,1]` and does not clip by default. The Real ActionAdapter does not clip gripper values; it applies thresholds. It suppresses publishing when the resolved open/close Bool equals the previous state. The bridge then pulses configured tool outputs. This suppression changes command count, not the resolved state trace.

## Configuration scope

The analyzed server is `oftplus_h5_vision` step 28560. The current Real runtime config names a different `oftplus_h5_proprio` step-6000 checkpoint. The post-processing polarity/chunk-selection mismatch is in shared runtime code, but the exact Episode 4 predictions must not be attributed to the proprio checkpoint.
