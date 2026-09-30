# OFT early-close diagnosis

Final classification: **H. MULTIPLE_CONTRIBUTING_FACTORS**.

## Evidence

- Dataset/model contract is confirmed continuous closedness: `0=open, 1=closed`.
- Bounded gripper sigmoid and statistics mask=false make a polarity-changing de-normalization impossible in this path.
- Phase 10 canonicalization preserves the last value without inversion.
- The valid OFT run contains nine K=5 chunks at source frames `0,5,...,40`. Expansion covers steps 0–44 exactly once, with no missing, duplicate or off-by-one step.
- Under dataset semantics and threshold 0.5, Reference closes at 26, OpenVLA at 23, and OFT at step 1 (`inference frame 0`, `chunk index 1`).
- Persistence N=1/2/3 stays at step 1, but N=5 moves OFT to step 24. This shows early chunk predictions oscillate rather than forming one stable close state.
- Real ActionAdapter polarity is reversed relative to the dataset and uses 0.7-open/0.3-close hysteresis.
- Real OFT runtime discards K=5 indices 1–4 and publishes only chunk 0. Applying both behaviors yields OFT runtime-equivalent close at step 10, not step 1, followed by reopening at step 25.

Therefore the saved model genuinely predicts high closedness before the Reference event under its training semantics, but the quoted “5 seconds early” is not the event the current Real runtime would produce. The current runtime itself has a confirmed polarity/post-processing mismatch and a K=5 utilization mismatch.

## Excluded primary causes

- `C. DENORMALIZATION_ERROR`: not supported; gripper mask=false and bounded output passes through.
- `D. CANONICALIZATION_ERROR`: not supported; value is preserved and range checked.
- `F. CHUNK_ALIGNMENT_ERROR`: not supported for offline expansion; mapping is exact. Runtime discarding four chunk elements is a separate execution-design mismatch.
- A pure `A. CONFIRMED_MODEL_EARLY_CLOSE` classification is insufficient because the runtime-equivalent requirement is not met at step 1.

## Event summary

| Signal | First close | Persistent N=2 | N=3 | N=5 | Open→close | Close→open | False-close frames | Missed-close frames |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| Reference | 26 | 26 | 26 | 26 | 1 | 0 | 0 | 0 |
| OpenVLA, dataset semantics | 23 | 23 | 23 | 23 | 1 | 0 | 3 | 0 |
| OFT K=5 expanded, dataset semantics | 1 | 1 | 1 | 24 | 5 | 4 | 16 | 0 |
| OpenVLA, Real-runtime semantics | 0 | 0 | 0 | 0 | 0 | 1 | 23 | 19 |
| OFT chunk0-hold, Real-runtime semantics | 10 | 10 | 10 | 10 | 1 | 1 | 15 | 19 |

At 5 Hz, OpenVLA dataset-semantics timing error is −0.6 s; OFT expanded first-crossing error is −5.0 s; OFT runtime-equivalent error is −3.2 s.

## Research limitation

This is one recorded Episode 4 and measures agreement with scripted Reference Commands. It does not show that OFT fails on Real hardware, that the predicted state would physically actuate at these times, or that OpenVLA is generally superior.
