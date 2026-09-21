# Phase7 Priority2 Report

## Scope

Priority2 reviewed Priority1 outputs, audited correction action chunk availability, ran feasible action-head-only hidden alignment baselines, and prepared matched camera shift / letterbox decomposition inputs.

## Correction Action Chunk Audit

Existing correction artifacts are mostly metric-only. Full L1/RMSE/Huber/cosine evaluation for those exact saved correction methods requires corrected action chunks or action-head rerun.

## Action-Head-Only Alignment

Best held-out method by action L2:

```text
progress_phase_shift = 0.066064
```

This result is `VERIFIED_ACTION_HEAD_ONLY`, not full image-to-action inference.

## Matched Camera Shift

Prepared conditions:

- raw original
- raw 24 px right shift
- P4 letterbox 4 px right shift matched to 24/1280
- P4 letterbox 24 px right shift legacy control

Full-forward status: `GPU_REQUIRED`.

## Letterbox Decomposition

Prepared conditions A0-A8 covering direct resize, black/gray/mean padding, replicated background, center crop, top and bottom alignment.

Full-forward status: `GPU_REQUIRED`.

## Global Distribution Alignment Baselines

Action-head-only baselines include global mean shift, diagonal mean-variance alignment, progress/phase/progress-phase shifts, PCA mean shift, oracle PCA delta projection, wrong-direction, and random matched-norm controls.

Deployability labels are saved in `04_global_vs_policy_alignment/deployability_table.csv`.

## Final State

```text
COMPLETED_PRIORITY2
PRIORITY3_STARTED: false
PRIORITY4_STARTED: false
```
