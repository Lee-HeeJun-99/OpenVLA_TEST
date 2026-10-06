# Stationary pendant cross-check form

This form is read-only. Do not jog, Home, actuate the gripper, change tool/TCP, change
mode, or acknowledge/reset alarms while collecting it.

| Item | Operator entry |
|---|---|
| Date/time | 2026-10-02 |
| Operator | user/local operator |
| Robot physically stationary | `operator_confirmed` (reported during read-only check) |
| Active TCP/tool name | `Tool_v1` |
| TCP offset translation | `[0, 0, 0]` |
| TCP offset translation unit | pendant convention; reported zero on all axes |
| TCP offset orientation A/B/C | `[0, 0, 0]` |
| TCP offset orientation unit/order shown | pendant convention; reported zero on all axes |
| Current TCP x/y/z | `[307.10, -12.75, 722.39] mm` |
| Current TCP A/B/C | displayed as `Rx/Ry/Rz = [179.74, -155.72, 3.30] deg` |
| Current pose reference frame | presumed base from current pendant screen; needs explicit screen/reference confirmation |
| Display identifies flange/TCP/tool-tip | active TCP `Tool_v1`; zero offset makes it coincide with flange in configuration |
| Gripper model | |
| Mounting adapter | |
| Gripper feedback sensor present | `yes / no / unknown` |
| Feedback signal/topic/DI mapping | |
| Pendant screenshot path | not provided; manually transcribed by operator |
| Notes | |

Computed comparison reference (same stationary session):

```text
link_6/base_link position = [0.3065574091, -0.0121740200, 0.7232624636] m
```

Do not compare orientation numerically until the pendant A/B/C convention and active
TCP transform are recorded.

## Offline comparison

- Pendant minus computed FK position: `[+0.543, -0.576, -0.872] mm`
- Position L2 difference: `1.178 mm`
- Computed Doosan-equivalent ZYZ: `[0.1063, 155.6999, -176.4572] deg`
- Pendant representation: `[179.74, -155.72, 3.30] deg`
- Matrix/geodesic difference if the pendant values use the audited Doosan ZYZ
  convention: `0.177°`

Raw Euler triples are non-unique; the orientation comparison uses rotation matrices.
This is a one-pose stationary cross-check, not continuous measured TCP feedback.
