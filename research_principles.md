# A0509 VLA Sim-to-Real Research Principles

Date: 2026-09-15 KST

## Core Hypothesis: Policy-Relevant Representation Gap

Representation Gap itself is not the final research contribution. It is expected that Real and Sim images produce different internal representations when passed through the same VLA.

The central question is:

> Which part of the measured Representation Gap actually changes downstream policy action prediction?

Definitions:

- `Representation Gap`: Difference between internal representations generated from paired Real and Sim observations.
- `Policy-Relevant Representation Gap`: The subset/direction/component of Representation Gap that meaningfully affects downstream action prediction.

Important rule:

```text
Representation Gap magnitude != Policy relevance
```

A large latent distance may be action-insensitive. A small latent distance may still lie along an action-sensitive direction and produce a large Action Gap.

## Additional Research Questions

RQ8. Does representation similarity imply policy similarity?

- RQ8-1. Do frames with larger Representation Gap also show larger Action Gap?
- RQ8-2. If Representation Gap is reduced, does Action Gap also decrease?
- RQ8-3. Which representation layer is most connected to Action Gap?
- RQ8-4. Which latent directions or tokens are action-sensitive?
- RQ8-5. Is policy-sensitive representation alignment more effective than whole-distribution alignment?

## Representation Gap To Action Gap

For each paired frame:

```text
z_real_i = representation(real_i)
z_sim_i  = representation(sim_i)

D_repr_i = D(z_real_i, z_sim_i)

a_real_i = Policy(z_real_i)
a_sim_i  = Policy(z_sim_i)

D_action_i = D(a_real_i, a_sim_i)
```

Analyze the relationship between `D_repr` and `D_action` by frame, episode, phase, and progress. Use Spearman/rank correlation when appropriate.

Correlation is not causality.

## Layer-Wise Policy Relevance

Do not ask only:

> Where is the Real/Sim representation gap largest?

Ask:

> Which layer's Real/Sim gap is most related to action discrepancy?

Candidate layers:

- `vision_backbone.output`
- `projector.output`
- multimodal representation
- action-facing hidden representation

Raw L2 across different layers is not directly comparable.

## Action Sensitivity

Representation directions are not equally important. A perturbation direction is policy-relevant only if it changes action:

```text
z' = z + delta
Delta a = Policy(z + delta) - Policy(z)
```

The Real-to-Sim difference vector:

```text
delta_RS = z_sim - z_real
```

should be tested for alignment with action-sensitive directions before claiming policy relevance.

## Correction Success Levels

LEVEL 1 - Representation:

```text
D(z_corrected, z_sim) < D(z_real, z_sim)
```

Claim allowed:

> latent discrepancy decreased.

LEVEL 2 - Action:

```text
D(a_corrected, a_sim) < D(a_real, a_sim)
```

and ideally:

```text
D(a_corrected, a_reference) < D(a_real, a_reference)
```

Claim allowed:

> offline action discrepancy decreased.

LEVEL 3 - Real Performance:

Corrected policy improves real-world task performance.

Claim allowed:

> representation correction contributed to Real-world policy performance improvement.

## Correction Generalization

Correction estimation data must be separate from final evaluation data.

For the initial 5-episode setting, use Leave-One-Episode-Out:

- train correction on 4 episodes
- test on the remaining episode
- repeat 5 folds

The test episode's Sim representation must not be used to estimate its own correction.

## Distribution vs Pairwise vs Policy Alignment

Separate:

- `Distribution Alignment`: Real/Sim distributions become statistically closer.
- `Pairwise Alignment`: each corresponding Real/Sim state becomes closer.
- `Policy Alignment`: action prediction becomes closer or better.

MMD reduction alone does not prove correspondence or policy improvement.

## Over-Correction / Collapse Checks

Correction must not destroy representation structure.

Check:

- feature variance
- covariance structure
- pairwise distance structure
- episode separation
- phase/progress structure
- token variance

MMD decreasing while representation diversity collapses is not a successful alignment.

## Token-Level Analysis

Do not rely only on pooled representations.

Audit:

- token count
- token ordering
- spatial/patch relation
- token norm
- token-wise cosine/L2
- token variance

Do not assume token index maps directly to a physical object until the model implementation is checked.

## Environment Attribution

Representation Gap exists, but cause is not obvious. Controlled ablations should connect:

```text
Environment factor
-> Observation Gap
-> Representation Gap
-> Action Gap
```

Candidate factors:

- camera
- geometry
- lighting
- background
- temporal/control
- preprocessing

## Policy-Relevant Alignment

The goal is not perfect Real environment reconstruction. The goal is to match the observation/representation structure that the policy uses for action.

Prioritize alignment by `Policy Impact`, not by physical difference magnitude alone.

## Final Evidence Chain

The final study should answer:

1. What differs between Real and Sim?
2. Which differences affect observations?
3. Which observation differences remain in representation?
4. Which representation differences affect actions?
5. Can environment alignment or representation correction reduce policy-relevant differences?
6. Does Action Gap decrease?
7. Does this hold on held-out conditions?
8. Does Real task performance improve?

Final target chain:

```text
Environment Gap
-> Observation Gap
-> Representation Gap
-> Policy-Relevant Representation Gap
-> Action Gap
-> Real Performance
```

Representation Gap itself is not the final result.
