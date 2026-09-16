# Action-Hidden Real→Sim Direction Perturbation

Experiment:
action_hidden_states.input Real→Sim direction perturbation.

Purpose:
Test whether the Real→Sim hidden-state difference is merely a large latent displacement or an action-sensitive direction for the `oftplus_h5_vision` action head.

Hypothesis:
If Δh_RS = h_sim - h_real contains policy-relevant components, perturbing h_real along Δh_RS should move the predicted action toward the Sim action more strongly than same-norm random or orthogonal controls.

Input:
- 225 Real/Sim paired frames from the verified 5-episode offline dataset.
- Feature: `action_hidden_states.input`, shape `(1, 35, 4096)`.
- Policy scope: `oftplus_h5_vision`, checkpoint step 28560.
- Action head checkpoint: `runtime_state/oft_mixed480_step28560_merged/action_head--28560_checkpoint.pt`.

Method:
For each pair, compute `Δh_RS = h_sim - h_real`, then evaluate `h(α) = h_real + αδ` through the action head for `α = [-0.5, 0, 0.25, 0.5, 0.75, 1.0, 1.5]`.

Control:
Two same-norm controls were used for each pair: a random direction and an orthogonal direction relative to Δh_RS.

Metrics:
- Gap to Sim action: chunk mean L2, chunk max L2.
- Change from Real action: chunk mean L2.
- First-action translation L2, rotation L2, gripper absolute gap.
- Overall, episode-wise, phase-wise, and progress-compatible frame outputs.

Result:
| Direction / α | Gap to Sim chunk mean L2 | Change from Real chunk mean L2 | Mean gap reduction ratio |
|---|---:|---:|---:|
| raw Real action, α=0 | 0.370245 | 0.000964 | -0.015244 |
| Real→Sim, α=0.5 | 0.177333 | 0.193181 | 0.452190 |
| Real→Sim, α=1.0 | 0.000664 | 0.370302 | 0.870652 |
| Real→Sim, α=1.5 | 0.065443 | 0.435074 | 0.497537 |
| random, α=1.0 | 0.365349 | 0.008559 | -0.037308 |
| orthogonal, α=1.0 | 0.364505 | 0.008480 | -0.036604 |

At α=1.0, Real→Sim perturbation reduced the action gap to nearly zero because it reconstructs the Sim hidden-state input to the same action head. The important control result is that same-norm random and orthogonal directions changed the action only weakly and did not reduce the gap.

Phase-wise Real→Sim α=1:
| Planner phase | Frames | Gap to Sim chunk mean L2 | Mean gap reduction ratio |
|---|---:|---:|---:|
| alignment | 85 | 0.000288 | 0.999501 |
| descent_to_grasp | 40 | 0.000939 | 0.995910 |
| grasp_close | 50 | 0.000821 | 0.754536 |
| hold | 10 | 0.000192 | 0.999696 |
| lift | 40 | 0.001110 | 0.584471 |

Interpretation:
This is direct offline Level-2 evidence that the Real→Sim difference at `action_hidden_states.input` includes an action-sensitive direction. The result is not explained by perturbation norm alone, because same-norm random and orthogonal directions produced much smaller action changes.

Status:
VERIFIED offline Level-2 evidence for the existing `oftplus_h5_vision` 225-pair dataset.

Limitation:
This does not identify which subcomponents/tokens are sensitive yet. It also does not prove environment causality or Real robot performance improvement. The α=1 Real→Sim result is a sanity check of action-head consistency, not deployable correction by itself because paired Sim hidden states are unavailable at deployment time.

Next decision:
Proceed to correction decomposition: estimate action-sensitive vs action-null components and compare full shift, sensitive-only, null-only, random, and no-correction conditions under the same action-dimension and phase breakdown.
