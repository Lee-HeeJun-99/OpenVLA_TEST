# Priority2 Global vs Policy Alignment

Status: `VERIFIED_ACTION_HEAD_ONLY`

This analysis used stored `action_hidden_states.input` features from P0 and reran the action head only.

Best method by held-out episode action L2:

```text
progress_phase_shift (0.066064)
```

Deployability labels are saved in `deployability_table.csv`.

Limit: this is not full image-to-action inference.
