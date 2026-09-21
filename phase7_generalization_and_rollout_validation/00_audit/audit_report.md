# Phase7 Priority1 Audit Report

## Scope

This audit covers the existing Phase1-Phase6 offline `oftplus_h5_vision` 225-pair dataset and associated saved feature/action outputs.

## Key Findings

- Action dimension is 7.
- Indices 0-2 are treated as translation deltas.
- Indices 3-5 are treated as rotation representation components.
- Index 6 is gripper closedness.
- Action chunks have length 5.
- Stored OFTRuntime `response['actions']` are treated as policy decoded actions after model unnormalization path.
- `action_head.output` feature tensors are not the same as final decoded `response['actions']`.
- Gripper binary threshold is not a verified GT threshold; Phase7 uses 0.5 only for Real/Sim disagreement.
- Existing Real/Sim action metrics are disagreement metrics, not GT accuracy metrics.

## Blocking / Caveat Items

- No verified measured real robot motion is available in the 225-pair offline data.
- No verified GT action exists for deciding whether Real or Sim policy output is correct.
- Direct TCP/EEF physical error remains blocked by coordinate convention audit.
- P4+C2 non-additivity is valid for the implemented image-space conditions but not a physically matched camera-shift conclusion.
