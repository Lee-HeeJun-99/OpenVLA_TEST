from __future__ import annotations
from dataclasses import dataclass
import json
from pathlib import Path
import threading
import time
import numpy as np


def _multi_array_rows(value):
    """Convert ROS Float64MultiArray[] or plain nested test lists to JSON rows."""
    rows=[]
    for row in value:
        data=getattr(row,"data",row)
        rows.append(list(data))
    return rows


@dataclass(frozen=True)
class FeedbackSnapshot:
    received_monotonic: float
    joint_rad: np.ndarray
    joint_velocity_rad_s: np.ndarray
    tool0_real_pose: object
    robot_state: int | None
    brake_state: tuple
    robot_state_received_monotonic: float | None
    joint_state_received_monotonic: float | None

    def log_record(self):
        return {"frame":"tool0_real","received_monotonic":self.received_monotonic,
                "joint_position_rad":self.joint_rad.tolist(),
                "joint_velocity_rad_s":self.joint_velocity_rad_s.tolist(),
                "tool0_real_position_m":self.tool0_real_pose.position_m.tolist(),
                "tool0_real_rotation_matrix":self.tool0_real_pose.rotation.tolist(),
                "robot_state":self.robot_state,"brake_state":list(self.brake_state)}


class RobotFeedback:
    def __init__(self, kinematics, tool_transform, stale_s=0.1, clock=time.monotonic,
                 robot_state_logger=None):
        self.kin = kinematics; self.tool = tool_transform
        self.stale_s = float(stale_s); self.clock = clock
        self.robot_state_logger = robot_state_logger
        self.lock = threading.RLock(); self._joint = None; self._robot = None
        self.subscriptions = []

    @classmethod
    def from_ros_node(cls, node, kinematics, tool_transform, config, clock=time.monotonic,
                      robot_state_logger=None):
        from sensor_msgs.msg import JointState
        from dsr_msgs2.msg import RobotState
        from rclpy.qos import qos_profile_sensor_data
        instance=cls(kinematics,tool_transform,
                     stale_s=config["feedback_stale_s_candidate"],clock=clock,
                     robot_state_logger=robot_state_logger)
        instance.subscriptions.append(node.create_subscription(
            JointState,config["joint_states_topic_unverified"],
            instance.joint_state_callback,qos_profile_sensor_data))
        instance.subscriptions.append(node.create_subscription(
            RobotState,config["robot_state_topic_unverified"],
            instance.robot_state_callback,qos_profile_sensor_data))
        return instance

    def joint_state_callback(self, msg):
        now = self.clock(); names = list(msg.name)
        mapping = {name: i for i, name in enumerate(names)}
        indices = [mapping[f"joint_{i}"] for i in range(1, 7)]
        q = np.asarray([msg.position[i] for i in indices], dtype=float)
        v = np.asarray([msg.velocity[i] for i in indices], dtype=float)
        if not np.all(np.isfinite(q)) or not np.all(np.isfinite(v)):
            raise ValueError("nonfinite JointState")
        with self.lock: self._joint = (now, q, v)

    def robot_state_callback(self, msg):
        now = self.clock()
        record={"host_received_monotonic":now,
                "robot_state":int(msg.robot_state),
                "current_posj":list(getattr(msg,"current_posj",())),
                "current_velj":list(getattr(msg,"current_velj",())),
                "target_posj":list(getattr(msg,"target_posj",())),
                "current_posx":list(getattr(msg,"current_posx",())),
                "current_tool_posx":list(getattr(msg,"current_tool_posx",())),
                "rotation_matrix":_multi_array_rows(getattr(msg,"rotation_matrix",())),
                "f_current_pos_world":_multi_array_rows(getattr(msg,"f_current_pos_world",())),
                "actual_bk":list(getattr(msg,"actual_bk",())),
                "joint_err":list(getattr(msg,"joint_err",())),
                "task_err":list(getattr(msg,"task_err",())),
                "sync_time":getattr(msg,"sync_time",None)}
        if self.robot_state_logger is not None:
            self.robot_state_logger.write(record)
        with self.lock:
            self._robot = (now, int(msg.robot_state),
                           tuple(getattr(msg, "actual_bk", ())))

    def snapshot(self, now=None):
        now = self.clock() if now is None else float(now)
        with self.lock:
            if self._joint is None:
                raise RuntimeError("JOINT_FEEDBACK_MISSING")
            jt, q, v = self._joint
            rt, state, brakes = self._robot if self._robot is not None else (None, None, ())
        received = min(jt, rt) if rt is not None else jt
        return FeedbackSnapshot(received, q.copy(), v.copy(),
            self.tool.link_to_tool(self.kin.fk(q)), state, brakes, rt, jt)

    def stale_reasons(self, now=None):
        now = self.clock() if now is None else float(now); reasons=[]
        with self.lock: joint=self._joint; robot=self._robot
        if joint is None or now-joint[0] > self.stale_s: reasons.append("JOINT_STATE_STALE")
        if robot is None or now-robot[0] > self.stale_s: reasons.append("ROBOT_STATE_STALE")
        return reasons


class FeedbackJsonlLogger:
    """Explicit logger for real feedback; every pose is labelled tool0_real."""
    def __init__(self,path):
        self.path=Path(path);self.path.parent.mkdir(parents=True,exist_ok=True)

    def write(self,snapshot):
        row=snapshot.log_record()
        with self.path.open("a") as stream:
            stream.write(json.dumps(row,separators=(",",":"))+"\n")
        return row


class RobotStateFullRateLogger:
    """Writes every RobotState callback for stop-test FK↔posx delta analysis."""
    def __init__(self,path):
        self.path=Path(path);self.path.parent.mkdir(parents=True,exist_ok=True)

    def write(self,record):
        with self.path.open("a") as stream:
            stream.write(json.dumps(record,separators=(",",":"))+"\n")
        return record
