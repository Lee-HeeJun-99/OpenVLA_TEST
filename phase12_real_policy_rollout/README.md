# Phase 12 Real policy rollout

Current state: `BLOCKED_REQUIRES_LOCAL_OPERATOR_AND_HARDWARE`.

This directory currently contains static audit, offline-only safety contracts, config and mock tests. It does not contain an authorized motion launcher. Run offline tests only:

```bash
/home/ubuntu/a0509_vla_linux_field_bundle_20260903/environment/a6000_ubuntu22_py310/bin/python -m unittest discover -s /home/ubuntu/a0509_vla_linux_field_bundle_20260903/lhj/phase12_real_policy_rollout/tests -v
```
