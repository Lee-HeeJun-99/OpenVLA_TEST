# Episode 4 reference-command-relative analysis

Status: `COMPLETED_WITH_MODEL_SERVER`. Episode 4 supplies 45 planned/commanded Reference Commands, not measured ground truth. OpenVLA step 8130 produced 45 H=1 predictions. OFT vision step 28560 produced nine non-overlapping K=5 chunks, covering the same 45 reference steps. Both runs had zero inference errors, zero fixture predictions, zero AI executed actions and zero robot-delivered commands.

| Model | Translation L1 mean (m) | Translation L2 RMSE (m) | Rotation L1 mean (rad) | Rotation L2 RMSE (rad) | Gripper MAE | Gripper accuracy |
|---|---:|---:|---:|---:|---:|---:|
| OpenVLA step 8130 | 0.017200 | 0.014574 | 0.232868 | 0.249586 | 0.068366 | 0.9333 |
| OFT vision step 28560 | 0.020589 | 0.017581 | 0.228374 | 0.246284 | 0.360308 | 0.6444 |

The Reference Command first closes at step 26. Under the training/dataset convention (`0=open`, `1=closed`) and a diagnostic 0.5 threshold, OpenVLA first crosses at step 23 (approximately 0.6 s early) and expanded OFT K=5 first crosses at step 1 (approximately 5.0 s early). That OFT step-1 event is inference frame 0, chunk index 1, and is not a chunk-expansion off-by-one error.

This is not the Real runtime-equivalent result. The inspected runtime treats low values as close, uses close/open thresholds 0.3/0.7 with state retention, executes only chunk index 0, and suppresses repeated gripper commands. Under those copied runtime semantics, OpenVLA first resolves close at step 0 and OFT first resolves close at step 10; OFT subsequently resolves open at step 25. Therefore the original “OFT closes at step 1 / 5.0 s early” statement describes the offline dataset-semantics trace, not the command event the current Real runtime would produce.

The diagnosis is `H. MULTIPLE_CONTRIBUTING_FACTORS`: the model output itself crosses the verified dataset-semantics closedness threshold early, while the Real runtime has an opposite polarity, different hysteretic thresholds, and a different K=5 consumption rule. No evidence supports a dataset de-normalization error, Phase 10 canonicalization error, or expanded-step alignment error. See `gripper_semantics_audit.md`, `gripper_early_close_diagnosis.md`, the step-level trace and sensitivity/confusion tables.

OpenVLA has smaller translation errors and much higher gripper accuracy in this single recorded episode. This is offline response agreement, not closed-loop success evidence. Frame samples are correlated and this one episode is insufficient for statistical model ranking.

Rotation uses `collector_rpy_assumption`. Large alignment rotation errors reflect comparison against the collector-interpolated orientation trajectory and must not be presented as independently verified native Doosan physical rotation error.

Latency in this one run must not be generalized: OpenVLA p50/p95/p99/max were 0.3153/0.3315/0.4511/0.5435 s including warm-up; OFT values were 0.2143/0.7845/1.0819/1.1563 s. Excluding the first request, OFT p95 was 0.2233 s and max was 0.2267 s. One OFT request exceeded the 1 s inference budget; all OpenVLA requests exceeded a synchronous 0.2 s budget, which affects closed-loop feasibility but does not prevent prediction-only Shadow logging.

Files include `episode4_model_reference_metrics.csv`, `episode4_phase_summary.csv`, `episode4_metric_summary.json`, `episode4_gripper_trace.csv`, sensitivity/confusion/phase/chunk summaries, latency summaries, diagnosis files and five figures.
