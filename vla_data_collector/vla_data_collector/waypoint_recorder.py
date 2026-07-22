from __future__ import annotations

import json
from pathlib import Path

import rclpy
from rclpy.node import Node

from .doosan_interface import DoosanInterface


DEFAULT_PLAN = [
    ("home", 1),
    ("approach", 1),
    ("pre_grasp", 1),
    ("grasp", 0),
    ("lift", 0),
]


class WaypointRecorder(Node):
    """
    Record TCP waypoints after the operator manually moves the robot.

    This node does not enable hand-guiding automatically.
    Enable hand-guiding using the approved Doosan procedure, move the robot,
    then press Enter for each waypoint.
    """

    def __init__(self) -> None:
        super().__init__("waypoint_recorder", namespace="dsr01")

        self.declare_parameter("output_path", "waypoints/cube_pick.json")
        self.declare_parameter("robot_id", "dsr01")
        self.declare_parameter("robot_model", "a0509")

        self.output_path = Path(str(self.get_parameter("output_path").value))
        robot_id = str(self.get_parameter("robot_id").value)
        robot_model = str(self.get_parameter("robot_model").value)

        self.robot = DoosanInterface(
            node=self,
            robot_id=robot_id,
            robot_model=robot_model,
        )
        self.waypoints: list[dict] = []

    def record(self, name: str, gripper_state: int) -> None:
        pose = self.robot.get_tcp_pose().as_array().tolist()
        item = {
            "name": name,
            "tcp_pose": pose,
            "gripper": int(gripper_state),
        }
        self.waypoints.append(item)
        self.get_logger().info(
            f"Recorded {name}: pose={pose}, gripper={gripper_state}"
        )

    def save(self) -> None:
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "instruction": "pick up the cube",
            "coordinate_frame": "base",
            "position_unit": "meter",
            "rotation_unit": "radian",
            "gripper_convention": {
                "0": "closed",
                "1": "open",
            },
            "waypoints": self.waypoints,
        }
        self.output_path.write_text(
            json.dumps(payload, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
        self.get_logger().info(f"Saved: {self.output_path}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WaypointRecorder()

    try:
        print("\n[주의] 승인된 절차로 Hand Guiding을 활성화하십시오.")
        print("각 위치로 로봇을 직접 이동한 뒤 Enter를 누르십시오.\n")

        for name, gripper_state in DEFAULT_PLAN:
            state_text = "OPEN(1)" if gripper_state == 1 else "CLOSE(0)"
            input(f"[{name}] 위치로 이동 후 Enter — gripper={state_text}: ")
            node.record(name, gripper_state)

        node.save()

    except KeyboardInterrupt:
        node.get_logger().warning("Waypoint recording cancelled.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
