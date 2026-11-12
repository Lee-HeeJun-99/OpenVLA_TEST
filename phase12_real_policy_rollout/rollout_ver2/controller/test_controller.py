import json
import tempfile
import unittest
from dataclasses import replace
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from controller.action import ActionInterpreter
from controller.config import load_config
from controller.geometry import ToolTransform, apply_action
from controller.gripper import ContinuousGripper
from controller.precheck import (EndpointStreak, GripperStateTracker, action_reasons,
                                 starting_home_reasons,joint_envelope_status)
from controller.publisher import DryRunPublisher
from controller.trajectory import minimum_jerk, SegmentGenerator,SegmentPreflightError
from controller.stop_reasons import IKStopReason
from kinematics import Pose, PyKDLKinematics

STATS = "/home/ubuntu/a0509_vla_linux_field_bundle_20260903/models/vanilla_s1_balanced_step8130/dataset_statistics.json"


class ControllerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.interpreter = ActionInterpreter(STATS)
        cls.tool = ToolTransform.tool0_real()

    def test_action_binary_and_endpoint_values(self):
        raw = self.interpreter.endpoint_max_values.copy(); raw[6] = 0.5
        a = self.interpreter.parse(raw)
        self.assertEqual(a.gripper_binary, 1)
        self.assertEqual(a.gripper_target, 0.5)
        self.assertTrue(np.all(a.endpoint_max[:6]))
        self.assertAlmostEqual(self.interpreter.endpoint_max_values[5], 0.0649145657)

    def test_tool_transform_round_trip(self):
        pose = Pose(np.array([0.3, -0.1, 0.7]), Rotation.from_euler("xyz", [1, 2, 3], degrees=True).as_matrix())
        recovered = self.tool.tool_to_link(self.tool.link_to_tool(pose))
        np.testing.assert_allclose(recovered.matrix, pose.matrix, atol=1e-12)

    def test_rotation_frames(self):
        pose = Pose(np.zeros(3), Rotation.from_euler("x", 45, degrees=True).as_matrix())
        omega = np.array([0, 0, 0.1])
        left = apply_action(pose, np.zeros(3), omega, "world_left")
        right = apply_action(pose, np.zeros(3), omega, "tool_right")
        self.assertGreater(np.linalg.norm(left.rotation - right.rotation), 1e-3)

    def test_minimum_jerk_boundaries(self):
        np.testing.assert_allclose(minimum_jerk(np.array([0.0, 1.0])), [0, 1])
        eps = 1e-6
        self.assertLess(abs((minimum_jerk(eps)-minimum_jerk(0))/eps), 1e-8)

    def test_endpoint_streak_and_no_clamp(self):
        config = replace(load_config(), endpoint_policy="stop")
        tracker = EndpointStreak(); raw = self.interpreter.endpoint_max_values.copy()
        action = self.interpreter.parse(raw)
        for _ in range(3): counts = tracker.update(action)
        reasons = action_reasons(action, np.array([0.3, 0, 0.5]), config, counts)
        self.assertTrue(any(x.startswith("ENDPOINT_TOKEN_STREAK") for x in reasons))
        np.testing.assert_array_equal(action.raw, raw)

    def test_gripper_endpoint_is_not_arm_streak(self):
        config = load_config(); tracker = EndpointStreak()
        raw = np.zeros(7); raw[6] = self.interpreter.endpoint_min_values[6]
        action = self.interpreter.parse(raw)
        for _ in range(config.endpoint_streak_limit + 1):
            counts = tracker.update(action)
        self.assertFalse(np.any(counts))
        self.assertFalse(any(x.startswith("ENDPOINT_TOKEN_STREAK") for x in
                             action_reasons(action, [0.3, 0, 0.5], config, counts)))

    def test_gripper_records_transitions_only(self):
        tracker = GripperStateTracker()
        self.assertFalse(tracker.update(0))
        self.assertFalse(tracker.update(0))
        self.assertTrue(tracker.update(1))
        self.assertFalse(tracker.update(1))

    def test_endpoint_default_is_log_only(self):
        config = load_config(); tracker = EndpointStreak()
        action = self.interpreter.parse(self.interpreter.endpoint_max_values)
        for _ in range(config.endpoint_streak_limit):
            counts = tracker.update(action)
        self.assertFalse(any(x.startswith("ENDPOINT_TOKEN_STREAK") for x in
                             action_reasons(action, [0.3, 0, 0.5], config, counts)))

    def test_continuous_gripper_matches_sim_slew(self):
        gripper = ContinuousGripper(60.0, 0.6)
        gripper.set_target(1.0)
        values = gripper.advance(12)
        self.assertAlmostEqual(values[0], 1/36)
        self.assertAlmostEqual(values[-1], 12/36)
        gripper.set_target(0.1)
        self.assertLess(gripper.advance(1)[0], values[-1])
        with self.assertRaisesRegex(ValueError, "OUT_OF_RANGE"):
            gripper.set_target(1.1)

    def test_ik_stop_reasons_are_distinct(self):
        self.assertNotEqual(IKStopReason.TARGET_UNREACHABLE,
                            IKStopReason.JOINT_LIMIT)
        self.assertNotEqual(IKStopReason.SINGULARITY,
                            IKStopReason.ALGORITHM_LIMIT)

    def test_segment_endpoint_and_dynamics(self):
        config = load_config()
        kin = PyKDLKinematics(); gen = SegmentGenerator(kin, config)
        q = np.deg2rad([-170, -10, -35, 15, -80, 20]); start = kin.fk(q)
        target = Pose(start.position_m + [0.001, 0, 0], start.rotation)
        segment = gen.generate(start, target, q, use_candidate_limits=False)
        np.testing.assert_allclose(kin.fk(segment.joints_rad[-1]).position_m,
                                   target.position_m, atol=2e-8)
        self.assertTrue(np.all(segment.max_velocity_rad_s <= config.max_joint_velocity_rad_s))
        self.assertTrue(np.all(segment.max_acceleration_rad_s2 <= config.max_joint_acceleration_rad_s2))

    def test_segment_is_not_returned_before_full_preflight(self):
        config=replace(load_config(),jacobian_min_singular_value=999.0)
        kin=PyKDLKinematics();gen=SegmentGenerator(kin,config)
        q=load_config().real_home_joint_rad;start=kin.fk(q)
        with self.assertRaises(SegmentPreflightError):
            gen.generate(start,start,q,use_candidate_limits=False)

    def test_dry_run_publisher_only_writes_file(self):
        class Dummy:
            times_s=np.array([0.0]); joints_rad=np.zeros((1,6))
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/"log.jsonl"; DryRunPublisher(path).publish_segment(0,Dummy())
            row=json.loads(path.read_text()); self.assertTrue(row["dry_run"])

    def test_start_home_check_is_jointwise_and_does_not_wrap(self):
        config=load_config()
        self.assertEqual(starting_home_reasons(config.real_home_joint_rad,config),[])
        self.assertEqual(starting_home_reasons(config.sim_home_joint_rad,config),
                         ["START_NOT_REAL_HOME:J6"])

    def test_sim_real_mount_action_trajectory_invariance(self):
        config=load_config();kin=PyKDLKinematics()
        sim_tool=ToolTransform.tool0_sim();real_tool=ToolTransform.tool0_real()
        q_sim=config.sim_home_joint_rad.copy()
        q_real=q_sim.copy();q_real[5]+=config.gripper_mount_offset_j6_rad
        sim_pose=sim_tool.link_to_tool(kin.fk(q_sim))
        real_pose=real_tool.link_to_tool(kin.fk(q_real))
        np.testing.assert_allclose(real_pose.matrix,sim_pose.matrix,atol=2e-8)
        actions=[([.001,0,0],[0,0,np.deg2rad(.2)]),
                 ([0,-.001,.0005],[np.deg2rad(.1),0,0]),
                 ([0,0,.001],[0,np.deg2rad(-.15),0])]
        for dp,omega in actions:
            sim_pose=apply_action(sim_pose,dp,omega,"world_left")
            real_pose=apply_action(real_pose,dp,omega,"world_left")
            np.testing.assert_allclose(real_pose.position_m,sim_pose.position_m,atol=1e-12)
            np.testing.assert_allclose(real_pose.rotation,sim_pose.rotation,atol=1e-12)
            sim_link=sim_tool.tool_to_link(sim_pose)
            real_link=real_tool.tool_to_link(real_pose)
            expected_real_rotation=(Rotation.from_matrix(sim_link.rotation)*
                                    Rotation.from_euler("z",180,degrees=True)).as_matrix()
            np.testing.assert_allclose(real_link.position_m,sim_link.position_m,atol=1e-12)
            np.testing.assert_allclose(real_link.rotation,expected_real_rotation,atol=1e-12)

    def test_joint_envelope_is_log_only_measurement(self):
        config=load_config();q=config.real_home_joint_rad.copy()
        result=joint_envelope_status(q,config);self.assertFalse(result['outside'])
        # matched15 continuous branch extends real-basis J6 well below the
        # Isaac225-only minimum (-34.44 deg); the union must include it.
        q[5]=np.deg2rad(-100);result=joint_envelope_status(q,config)
        self.assertFalse(result['outside'])
        q[4]=np.deg2rad(-114);result=joint_envelope_status(q,config)
        self.assertTrue(result['outside']);self.assertEqual(result['joints'],[5])

    def test_joint_envelope_is_exact_union_of_recorded_sources(self):
        path=Path(__file__).with_name("controller_config.json")
        raw=json.loads(path.read_text())
        envelope=raw["sim_joint_envelope_real_basis_deg"]
        sources=list(envelope["sources"].values())
        expected_min=np.min([source["min"] for source in sources],axis=0)
        expected_max=np.max([source["max"] for source in sources],axis=0)
        np.testing.assert_allclose(envelope["min"],expected_min,atol=1e-5)
        np.testing.assert_allclose(envelope["max"],expected_max,atol=1e-5)
        self.assertEqual(envelope["combination"],"per_joint_union")
        self.assertEqual(envelope["policy"],"log_only")


if __name__ == "__main__":
    unittest.main()
