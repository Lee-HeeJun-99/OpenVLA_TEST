# matched15 sim reference

- source: `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/validation/evidence/vanilla/success_profile_s1_balanced_step8130_matched15_k1_seed9406_15ep.json`
- IK: sim_home seed 후 직전 해 seed의 연속 branch. 통계는 연속 pose 전이만 사용하며 home seed→첫 관측 정렬은 제외.
- 전체 관절 성분 |Δq| p99/max: `3.3825° / 5.0182°`.
- 10° 후보가 관측 최대보다 작은가: `False`.
- 제외한 home seed→첫 pose 정렬 max: `33.0519°`.

| joint | p99 | max |
|---|---:|---:|
| J1 | 0.9916° | 1.1747° |
| J2 | 2.2812° | 2.4773° |
| J3 | 3.7015° | 5.0182° |
| J4 | 0.4906° | 1.5447° |
| J5 | 3.4924° | 4.0026° |
| J6 | 3.5178° | 3.7818° |

## Cube positions relative to episode home tool0 origin

단위 mm. 표는 sim world≈robot base 축이며 원점만 첫 tool0 pose로 옮겼다. tool0 자체 축 표현도 JSON에 저장했다.

| episode | target | cube | base-axis relative xyz mm |
|---:|---|---|---|
| 1 | red | red | [376.62, -57.27, -609.03] |
| 1 | red | yellow | [433.09, -183.52, -609.03] |
| 1 | red | blue | [281.38, -203.95, -609.03] |
| 2 | yellow | red | [244.03, 119.56, -609.04] |
| 2 | yellow | yellow | [335.67, -44.76, -609.04] |
| 2 | yellow | blue | [389.04, -137.80, -609.04] |
| 3 | blue | red | [317.58, -75.70, -609.03] |
| 3 | blue | yellow | [294.85, 187.91, -609.03] |
| 3 | blue | blue | [234.61, 86.17, -609.03] |
| 4 | red | red | [275.27, 38.93, -609.04] |
| 4 | red | yellow | [404.92, -0.92, -609.04] |
| 4 | red | blue | [441.00, -200.09, -609.04] |
| 5 | yellow | red | [381.71, -98.85, -609.05] |
| 5 | yellow | yellow | [333.09, 193.64, -609.05] |
| 5 | yellow | blue | [424.04, -224.57, -609.05] |
| 6 | blue | red | [460.74, 49.31, -608.98] |
| 6 | blue | yellow | [271.35, -88.33, -608.98] |
| 6 | blue | blue | [234.54, 85.86, -608.98] |
| 7 | red | red | [303.05, -112.08, -609.04] |
| 7 | red | yellow | [385.73, 101.08, -609.04] |
| 7 | red | blue | [416.52, -193.58, -609.04] |
| 8 | yellow | red | [304.94, -148.63, -609.00] |
| 8 | yellow | yellow | [320.37, -3.73, -609.00] |
| 8 | yellow | blue | [433.89, -90.89, -609.00] |
| 9 | blue | red | [356.24, -33.10, -609.04] |
| 9 | blue | yellow | [249.92, -103.86, -609.04] |
| 9 | blue | blue | [234.97, 86.24, -609.04] |
| 10 | red | red | [235.62, 25.66, -609.03] |
| 10 | red | yellow | [379.73, -103.56, -609.03] |
| 10 | red | blue | [452.15, 192.47, -609.03] |
| 11 | yellow | red | [458.44, -192.40, -609.04] |
| 11 | yellow | yellow | [351.88, -33.31, -609.04] |
| 11 | yellow | blue | [399.83, 61.40, -609.04] |
| 12 | blue | red | [453.43, 160.29, -609.01] |
| 12 | blue | yellow | [295.45, 205.36, -609.01] |
| 12 | blue | blue | [271.27, -89.34, -609.01] |
| 13 | red | red | [300.89, -224.43, -609.05] |
| 13 | red | yellow | [244.18, -29.52, -609.05] |
| 13 | red | blue | [440.80, -123.02, -609.05] |
| 14 | yellow | red | [254.26, 117.90, -609.04] |
| 14 | yellow | yellow | [356.19, -130.35, -609.04] |
| 14 | yellow | blue | [470.00, 35.87, -609.04] |
| 15 | blue | red | [271.90, 163.15, -609.04] |
| 15 | blue | yellow | [288.71, -129.45, -609.04] |
| 15 | blue | blue | [332.97, -221.93, -609.04] |

## Images

Evidence paths are Windows D:\codex_staging paths absent from this bundle. Saved images are related-sim proxies and must not be represented as exact matched15 frames.

- `reports/sim_reference/sim_proxy_initial_01_224.jpg` ← `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/lhj/phase9_sim_policy_shadow_replay/01_sim_policy_rollout_episode4_sim_baseline/episode4_sim_baseline_redprompt_k5_seed9406_1ep_frames/episode_001_red_initial.jpg` (proxy)
- `reports/sim_reference/sim_proxy_initial_02_224.jpg` ← `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/lhj/phase9_sim_policy_shadow_replay/01_sim_policy_rollout/phase9_fixed_layout_sim_policy_k1_seed20260922_1ep_frames/episode_001_red_initial.jpg` (proxy)
- `reports/sim_reference/sim_proxy_initial_03_224.jpg` ← `/home/ubuntu/a0509_vla_linux_field_bundle_20260903/lhj/phase9_sim_policy_shadow_replay/01_sim_policy_rollout_episode4/episode4_fixed_layout_oft_k5_k5_seed20260923_1ep_frames/episode_001_red_initial.jpg` (proxy)

## Exact matched15 layout render inputs (episodes 3, 6, 9)

`exact_matched15_layout=true`는 각 `pose.json`의 `cube_world_pose.position_m`이 evidence의
`sampled_layout_m` 원문이며, 평가 코드와 동일하게 source USD spawn quaternion을 yaw로
사용한다는 뜻이다. 상대좌표로 다시 계산하지 않았다. evidence 결과 자체에는 yaw와
초기 관절 배열이 없으므로, yaw는 동일 USD spawn pose, 관절은 평가 fallback인
`isaac_sim/config/a0509_home_pose.json`에서 가져왔다. 물리 정착 뒤 evidence가 기록한
`initial_cube_positions_m`도 pose JSON에 별도 보존했다.

| episode | exact layout | pose/source | policy 1280×720 | resize-naive 224 | overview | status |
|---:|---|---|---|---|---|---|
| 3 | true | [pose.json](episode03/pose.json), [renderer](episode03/render_episode.py) | 미생성 | 미생성 | 미생성 | [GPU 차단](episode03/render_status.json) |
| 6 | true | [pose.json](episode06/pose.json), [renderer](episode06/render_episode.py) | 미생성 | 미생성 | 미생성 | [선행 차단](episode06/render_status.json) |
| 9 | true | [pose.json](episode09/pose.json), [renderer](episode09/render_episode.py) | 미생성 | 미생성 | 미생성 | [선행 차단](episode09/render_status.json) |

### Environment and invocation

- Isaac Sim: `/home/ubuntu/Downloads/isaac-sim-standalone-5.1.0-linux-x86_64`,
  `5.1.0-rc.19+release.26219.9c81211b.gl`.
- scene: `validation/a0509_project/USD/version_1/a0509_version_1_scaled_85x60.usda`.
- evaluator: `isaac_sim/a0509_control/vla_validation.py`, wrapper
  `isaac_sim/run_oft_validation.py`, application `isaac_sim/a0509_control_app.py`.
- phase9 rollout command/reference: `lhj/phase9_sim_policy_shadow_replay/README.md`.
- source evaluator uses RayTracedLighting, camera
  `/World/a0509/link_6/tool0/left_Camera`, 1280×720. The 224 image is bicubic
  resize-naive with no crop.
- retry command (GPU driver is available):
  `/home/ubuntu/Downloads/isaac-sim-standalone-5.1.0-linux-x86_64/python.sh reports/sim_reference/episode03/render_episode.py`
  (repeat for episode06 and episode09).

### Render blocker

Isaac Sim is installed, but the 2026-11-13 episode 3 attempt could not enumerate a GPU:
`NVML_ERROR_DRIVER_NOT_LOADED`, `No device could be created`, and
`GPU Foundation is not initialized`; `nvidia-smi` also could not communicate with the driver.
No PNG was produced. Episodes 6 and 9 were therefore not launched and no proxy image was
mislabelled as an exact render. When rendering succeeds, the renderer/lighting are intended to
match evaluation (`RayTracedLighting` and the same source USD); the overview camera and its
annotations are new observation-only additions and are not part of training/evaluation.
