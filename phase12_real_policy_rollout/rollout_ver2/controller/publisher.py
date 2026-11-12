from __future__ import annotations

from abc import ABC, abstractmethod
import json
from pathlib import Path


class Publisher(ABC):
    """Transport-neutral output interface; no ROS implementation is provided."""

    @abstractmethod
    def publish_segment(self, segment_id, segment):
        raise NotImplementedError


class DryRunPublisher(Publisher):
    def __init__(self, output_path):
        self.output_path = Path(output_path)
        self.output_path.parent.mkdir(parents=True, exist_ok=True)

    def publish_segment(self, segment_id, segment):
        with self.output_path.open("a") as stream:
            for time_s, joints in zip(segment.times_s, segment.joints_rad):
                stream.write(json.dumps({"dry_run": True, "segment": int(segment_id),
                                         "time_s": float(time_s),
                                         "joint_position_deg": [float(x) for x in __import__('numpy').rad2deg(joints)]},
                                        separators=(",", ":")) + "\n")

    def publish_gripper(self, segment_id, times_s, commands):
        with self.output_path.open("a") as stream:
            for time_s, command in zip(times_s, commands):
                stream.write(json.dumps({"dry_run": True,
                                         "segment": int(segment_id),
                                         "time_s": float(time_s),
                                         "gripper_command": float(command)},
                                        separators=(",", ":")) + "\n")
