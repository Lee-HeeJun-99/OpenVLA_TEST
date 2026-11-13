#!/usr/bin/env python3
"""Render one frozen matched15 layout without changing source USD or evaluation code."""
from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path


def arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--pose-json", type=Path, required=True)
    parser.add_argument("--warmup", type=int, default=90)
    return parser.parse_args()


ARGS = arguments()
POSE_PATH = ARGS.pose_json.expanduser().resolve()
OUT = POSE_PATH.parent
POSE = json.loads(POSE_PATH.read_text(encoding="utf-8"))

from isaacsim import SimulationApp

APP = SimulationApp(
    {
        "headless": True,
        "renderer": "RayTracedLighting",
        "open_usd": POSE["scene_usd"],
        "create_new_stage": False,
        "width": 1280,
        "height": 720,
    }
)


def yaw_deg(wxyz) -> float:
    w, x, y, z = (float(v) for v in wxyz)
    return math.degrees(math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))


def save_rgba(camera, path: Path) -> None:
    import numpy as np
    from PIL import Image

    rgba = camera.get_rgba()
    if rgba is None:
        raise RuntimeError(f"camera returned no image: {camera.prim_path}")
    array = np.asarray(rgba)
    if array.dtype != np.uint8:
        array = np.clip(array * (255.0 if array.max() <= 1.0 else 1.0), 0, 255).astype(np.uint8)
    Image.fromarray(array[:, :, :3], "RGB").save(path)


try:
    import numpy as np
    import omni.timeline
    import omni.usd
    from isaacsim.core.prims import SingleArticulation, SingleRigidPrim
    from isaacsim.core.utils.viewports import set_camera_view
    from isaacsim.sensors.camera import Camera
    from PIL import Image, ImageDraw
    from pxr import Gf, Sdf, UsdGeom

    stage = omni.usd.get_context().get_stage()
    session = stage.GetSessionLayer()
    stage.SetEditTarget(session)
    timeline = omni.timeline.get_timeline_interface()

    robot = SingleArticulation(
        prim_path=POSE["robot_prim"], name="matched15_reference_robot", reset_xform_properties=False
    )
    cubes = {
        color: SingleRigidPrim(path, name=f"matched15_{color}", reset_xform_properties=False)
        for color, path in POSE["cube_prims"].items()
    }
    timeline.play()
    for _ in range(5):
        APP.update()
    robot.initialize()
    for cube in cubes.values():
        cube.initialize()

    arm_names = [f"joint_{index}" for index in range(1, 7)]
    arm_indices = np.asarray([robot.get_dof_index(name) for name in arm_names], dtype=np.int64)
    home = np.asarray(POSE["initial_robot_joint_positions_rad"], dtype=np.float64)
    robot.set_joint_positions(home, joint_indices=arm_indices)
    robot.set_joint_velocities(np.zeros(6), joint_indices=arm_indices)

    resolved = {}
    for color, cube in cubes.items():
        _, spawn_quat = cube.get_world_pose()
        spawn_quat = np.asarray(spawn_quat, dtype=np.float64)
        cube.set_world_pose(
            position=np.asarray(POSE["cube_world_pose"][color]["position_m"], dtype=np.float64),
            orientation=spawn_quat,
        )
        cube.set_linear_velocity(np.zeros(3))
        cube.set_angular_velocity(np.zeros(3))
        resolved[color] = {
            "quaternion_wxyz": spawn_quat.tolist(),
            "yaw_deg": yaw_deg(spawn_quat),
            "source": "spawn orientation read from the same source USD, as vla_validation._place_layout does",
        }

    # Update transforms without advancing physics: preserve the exact commanded layout.
    for _ in range(max(1, ARGS.warmup)):
        APP.update()

    policy = Camera(
        prim_path=POSE["policy_camera_prim"], frequency=5.0, resolution=(1280, 720)
    )
    policy.initialize()

    overview_path = "/World/Matched15ReferenceOverviewCamera"
    UsdGeom.Camera.Define(stage, Sdf.Path(overview_path))
    set_camera_view(
        eye=np.array([0.35, -0.001, 2.45], dtype=np.float64),
        target=np.array([0.35, 0.0, 0.82], dtype=np.float64),
        camera_prim_path=overview_path,
    )
    overview = Camera(prim_path=overview_path, frequency=5.0, resolution=(1280, 720))
    overview.initialize()
    for _ in range(max(30, ARGS.warmup)):
        APP.update()

    policy_raw = OUT / "policy_camera_1280x720.png"
    save_rgba(policy, policy_raw)
    Image.open(policy_raw).convert("RGB").resize((224, 224), Image.Resampling.BICUBIC).save(
        OUT / "policy_camera_resize_naive_224.png"
    )
    overview_raw = OUT / "overview_raw.png"
    save_rgba(overview, overview_raw)

    tool_prim = stage.GetPrimAtPath(POSE["tool0_prim"])
    tool_matrix = UsdGeom.Xformable(tool_prim).ComputeLocalToWorldTransform(0)
    tool_xyz = np.asarray(tool_matrix.ExtractTranslation(), dtype=np.float64)
    table_z = float(min(v["position_m"][2] for v in POSE["cube_world_pose"].values()) - 0.035)
    base_xyz = np.asarray(
        UsdGeom.Xformable(stage.GetPrimAtPath(POSE["robot_prim"])).ComputeLocalToWorldTransform(0).ExtractTranslation(),
        dtype=np.float64,
    )
    points = np.vstack(
        [
            [base_xyz[0], base_xyz[1], table_z + 0.003],
            [base_xyz[0] + 0.18, base_xyz[1], table_z + 0.003],
            [base_xyz[0], base_xyz[1] + 0.18, table_z + 0.003],
            [tool_xyz[0], tool_xyz[1], table_z + 0.003],
        ]
    )
    pixels = np.asarray(overview.get_image_coords_from_world_points(points), dtype=np.float64)
    image = Image.open(overview_raw).convert("RGB")
    draw = ImageDraw.Draw(image)
    origin, x_end, y_end, tool_px = [tuple(map(float, point[:2])) for point in pixels]
    draw.line([origin, x_end], fill=(255, 60, 60), width=7)
    draw.line([origin, y_end], fill=(60, 255, 80), width=7)
    draw.ellipse([tool_px[0] - 9, tool_px[1] - 9, tool_px[0] + 9, tool_px[1] + 9], outline=(40, 180, 255), width=5)
    draw.text((x_end[0] + 5, x_end[1]), "base +x", fill=(255, 60, 60))
    draw.text((y_end[0] + 5, y_end[1]), "base +y", fill=(60, 255, 80))
    draw.text((tool_px[0] + 12, tool_px[1]), "home tool0 projection", fill=(40, 180, 255))
    image.save(OUT / "overview_annotated.png")

    POSE["resolved_cube_spawn_orientation"] = resolved
    POSE["render_result"] = {
        "isaac_sim_version": "5.1.0-rc.19+release.26219.9c81211b.gl",
        "renderer": "RayTracedLighting",
        "policy_resolution": [1280, 720],
        "resize_naive": {"resolution": [224, 224], "method": "PIL bicubic, no crop"},
        "overview_eye_world_m": [0.35, -0.001, 2.45],
        "overview_target_world_m": [0.35, 0.0, 0.82],
        "home_tool0_world_position_m": tool_xyz.tolist(),
        "home_tool0_table_projection_world_m": [float(tool_xyz[0]), float(tool_xyz[1]), table_z],
        "physics_advanced_after_placement": False,
    }
    POSE_PATH.write_text(json.dumps(POSE, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
finally:
    try:
        timeline.stop()
    except Exception:
        pass
    APP.close()
