# Work log — remote offline integration

- Confirmed robot workspace HEAD `86eaa9632d651eb907332334d02f32c1461850d7`; protected dirty collection/runtime trees.
- Read collection/control/logging paths; classified trajectory as scripted reference.
- Audited episode 4 metadata and all 45 recorded steps.
- Implemented ROS-free Reference Command extractor and schema.
- Generated 45 episode 4 Reference Commands; maximum difference from existing `steps_with_actions.jsonl` was exactly 0.0.
- Documented collector rotation conversion and unresolved native Doosan convention.
- Extended explicit timestamp/clock-domain policy.
- Added a zero-node Shadow launch draft and fail-closed safety configuration.
- Audited prediction-only server code. The local operator subsequently ran both servers: OpenVLA returned 45 H=1 predictions and OFT returned nine K=5 chunks with zero inference errors and no command delivery.
- Generated episode 4 reference-command-relative per-step, phase and overall metrics. Reference close step is 26; OpenVLA first close is 23 and OFT first close is 1.
- Audited the complete gripper path from OFT action head and dataset statistics through the HTTP response, Phase 10 adapters, and the Real runtime action adapter/hardware-output selection.
- Auto-selected the newest valid, non-fixture, error-free 45-step OpenVLA and OFT recorded-input results; preserved older failed candidates unchanged.
- Generated a 45-step gripper trace, threshold/polarity sweep, overall/phase/chunk confusion summaries, latency samples/statistics and five diagnostic figures.
- Verified OFT K=5 expansion: inference frames 0,5,...,40 map to steps 0:4,5:9,...,40:44 with no duplicates, omissions or off-by-one shift.
- Classified the step-1 finding as `H. MULTIPLE_CONTRIBUTING_FACTORS`: genuine early high-closedness model outputs under dataset semantics, plus opposite Real-runtime polarity, hysteretic thresholds and chunk-0-only consumption.
- Added copied-semantics unit coverage for runtime polarity, thresholds, state retention, repeated-command suppression and chunk ordering.
- Ran 18 offline tests successfully with bundle Python 3.10.
- Did not query ROS graph, connect hardware, launch runtime, publish commands, or call services/actions.
