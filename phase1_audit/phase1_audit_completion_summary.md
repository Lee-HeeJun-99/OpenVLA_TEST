# Phase 1 Audit Completion Summary

Date: 2026-09-15 KST

## Completion Criterion

Phase 1 first-pass audit is considered complete because each core item has been assigned a status:

- `VERIFIED`
- `EXISTING RESULT`
- `UNVERIFIED`
- `INVALID / RE-RUN REQUIRED`
- `PLANNED`

Not all items are verified. The purpose of this pass is to identify which existing results can be reused and which claims are blocked.

## Reusable Existing Baseline

The existing 5-episode paired dataset can be reused for bundle-policy offline method development.

Reusable scope:

- 5 Real episodes
- 5 matching Sim replay episodes
- 225 paired frames
- Bundle OFT `oftplus_h5_vision`
- Instruction: `Pick up the orange cube.`
- Existing feature/action manifests
- Existing representation metrics
- New LHJ action-gap/correction metrics

Allowed claim:

> For the existing 5-episode bundle-policy offline dataset, Real/Sim observations show measurable representation/action discrepancies, and leave-one-episode-out progress-conditioned Real→Sim correction reduces both latent discrepancy and offline final Action Gap.

Not allowed:

> This improves current ROS real robot deployment.

> This proves camera/geometry/lighting as the cause.

> This generalizes to unseen layouts/trajectories.

## Key Verified Items

- 5-episode frame/step counts and visual consistency.
- Real commanded metadata consistency.
- Sim replay condition consistency.
- Replay uses home-relative joint delta from Real source steps.
- Sim target vs actual joint after snap is near exact.
- Bundle policy identity:
  - `oftplus_h5_vision`
  - K=5
  - vision-only
  - center crop enabled
- Feature tensor shapes.
- Representation metrics for existing pairs.
- LOO representation progress shift.
- OFT action normalization path.
- LOO policy-relevant action-hidden progress shift:
  - raw action chunk mean L2: `0.370262`
  - hidden-shift action chunk mean L2: `0.072325`
  - relative reduction: `80.47%`

## High-Impact Unverified Or Invalid Items

### 1. Current ROS deployment mismatch

Status: `INVALID / RE-RUN REQUIRED`

Current ROS runtime config uses:

- `oftplus_h5_proprio`
- different checkpoint
- different preprocessing/crop settings

The LHJ analysis uses:

- bundle `oftplus_h5_vision`
- center crop enabled

Impact:

- Current LHJ results cannot be used as final evidence for current ROS real deployment.

### 2. Real physical geometry and table history

Status: `UNVERIFIED`

Impact:

- Geometry-cause attribution is blocked.

### 3. Real camera intrinsic/extrinsic

Status: `UNVERIFIED`

Impact:

- Camera-cause attribution is blocked.

### 4. Real measured TCP/EEF trajectory

Status: `UNVERIFIED`

Real 5-episode data stores planned/commanded pose, not measured feedback pose.

Impact:

- Physical TCP tracking equivalence is not established.

### 5. Direct Real TCP vs Sim tool0 comparison

Status: `INVALID / RE-RUN REQUIRED`

Reason:

- Frame definitions and orientation conventions are not reconciled.

## Phase 2 Decision

Proceed with two separated tracks:

### Track A - Bundle Offline Research Track

Purpose:

- Continue method development for policy-relevant representation correction.

Use:

- Existing 5-episode bundle-policy baseline.
- Existing LHJ Level 1/Level 2 offline evidence.

Next:

- Evaluate correction collapse/variance preservation.
- Add token/action-sensitive direction analysis.
- If possible, test held-out layout/trajectory when more data exists.

### Track B - Real Deployment Track

Purpose:

- Prepare claims for real robot policy testing.

Required before claims:

- Decide target deployment policy:
  - bundle `oftplus_h5_vision`, or
  - current ROS `oftplus_h5_proprio`.
- Align preprocessing/policy identity.
- Regenerate feature/action metrics for that exact runtime path.
- Obtain or explicitly mark missing Real camera/geometry/frame calibration.

## Next Work Items

1. For Track A:
   - run over-correction/collapse checks on corrected action-hidden features.
   - analyze whether hidden-shift preserves variance/pairwise structure.
2. For Track B:
   - prepare a runtime alignment checklist.
   - decide whether to switch ROS to bundle vision-only policy or rerun analysis for proprio policy.
3. Do not run Real robot motion without explicit user approval.
