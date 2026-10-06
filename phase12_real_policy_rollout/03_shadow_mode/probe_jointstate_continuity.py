#!/usr/bin/env python3
"""Passive JointState continuity probe; creates no publishers or service clients."""

import argparse
import json
import math
import statistics
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from read_only_state_adapter import FeedbackReadinessGate


CANONICAL = tuple(f"joint_{index}" for index in range(1, 7))


class Probe(Node):
    def __init__(self, topic: str, reliable: bool) -> None:
        super().__init__("phase12_jointstate_continuity_probe")
        self.source_ns = []
        self.receive_monotonic_ns = []
        self.receive_ros_ns = []
        self.orders = set()
        self.invalid_position = 0
        self.invalid_velocity = 0
        self.missing_joint = 0
        self.effort_unsupported = 0
        self.first_receive_wall_monotonic = None
        self.gap_events = []
        self.gate = FeedbackReadinessGate()
        self.feedback_ready_wall_monotonic = None
        self.measurement_source_ns = []
        self.measurement_receive_monotonic_ns = []
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1000,
            reliability=ReliabilityPolicy.RELIABLE if reliable else ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(JointState, topic, self.callback, qos)

    def callback(self, message: JointState) -> None:
        if self.first_receive_wall_monotonic is None:
            self.first_receive_wall_monotonic = time.monotonic()
        receive_monotonic_ns = time.monotonic_ns()
        receive_ros_ns = self.get_clock().now().nanoseconds
        source_ns = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        if self.source_ns:
            source_gap_ns = source_ns - self.source_ns[-1]
            receive_gap_ns = receive_monotonic_ns - self.receive_monotonic_ns[-1]
            if source_gap_ns >= 50_000_000 or receive_gap_ns >= 50_000_000:
                self.gap_events.append(
                    {
                        "previous_local_receive_index": len(self.source_ns) - 1,
                        "current_local_receive_index": len(self.source_ns),
                        "previous_source_timestamp_ns": self.source_ns[-1],
                        "current_source_timestamp_ns": source_ns,
                        "source_gap_ns": source_gap_ns,
                        "source_gap_seconds": source_gap_ns / 1e9,
                        "previous_receive_monotonic_ns": self.receive_monotonic_ns[-1],
                        "current_receive_monotonic_ns": receive_monotonic_ns,
                        "receive_gap_ns": receive_gap_ns,
                        "receive_gap_seconds": receive_gap_ns / 1e9,
                    }
                )
        names = list(message.name)
        indices = {name: index for index, name in enumerate(names)}
        self.orders.add(tuple(names))
        self.source_ns.append(source_ns)
        self.receive_monotonic_ns.append(receive_monotonic_ns)
        self.receive_ros_ns.append(receive_ros_ns)

        missing = any(name not in indices for name in CANONICAL)
        self.missing_joint += int(missing)
        if not missing:
            self.invalid_position += int(
                len(message.position) != len(names)
                or not all(math.isfinite(float(message.position[indices[name]])) for name in CANONICAL)
            )
            self.invalid_velocity += int(
                len(message.velocity) != len(names)
                or not all(math.isfinite(float(message.velocity[indices[name]])) for name in CANONICAL)
            )
        self.effort_unsupported += int(
            len(message.effort) != len(names)
            or not all(math.isfinite(float(value)) for value in message.effort)
        )
        sample_valid = not missing and self.invalid_position == 0 and self.invalid_velocity == 0
        gate_status = self.gate.observe(
            source_ns=source_ns,
            receive_monotonic_ns=receive_monotonic_ns,
            sample_valid=sample_valid,
        )
        if gate_status['feedback_ready']:
            if self.feedback_ready_wall_monotonic is None:
                self.feedback_ready_wall_monotonic = time.monotonic()
            self.measurement_source_ns.append(source_ns)
            self.measurement_receive_monotonic_ns.append(receive_monotonic_ns)
        else:
            self.feedback_ready_wall_monotonic = None
            self.measurement_source_ns.clear()
            self.measurement_receive_monotonic_ns.clear()


def series_stats(values):
    gaps = [(right - left) / 1e9 for left, right in zip(values, values[1:])]
    span = (values[-1] - values[0]) / 1e9 if len(values) > 1 else 0.0
    return {
        "span_seconds": span,
        "rate_hz": (len(values) - 1) / span if span > 0 else 0.0,
        "median_gap_seconds": statistics.median(gaps) if gaps else None,
        "max_gap_seconds": max(gaps) if gaps else None,
        "gap_ge_50ms": sum(gap >= 0.05 for gap in gaps),
        "gap_ge_100ms": sum(gap >= 0.1 for gap in gaps),
        "gap_ge_500ms": sum(gap >= 0.5 for gap in gaps),
        "gap_ge_1s": sum(gap >= 1.0 for gap in gaps),
        "duplicate": sum(right == left for left, right in zip(values, values[1:])),
        "nonmonotonic": sum(right < left for left, right in zip(values, values[1:])),
        "largest_gaps": sorted(gaps, reverse=True)[:10],
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/dsr01/joint_states")
    parser.add_argument("--duration", type=float, default=60.0)
    parser.add_argument("--discovery-timeout", type=float, default=30.0)
    parser.add_argument("--best-effort", action="store_true")
    parser.add_argument("--output")
    arguments = parser.parse_args()

    rclpy.init()
    probe = Probe(arguments.topic, reliable=not arguments.best_effort)
    started = time.monotonic()
    try:
        while rclpy.ok():
            now = time.monotonic()
            if probe.first_receive_wall_monotonic is None:
                if now - started >= arguments.discovery_timeout:
                    break
            elif (probe.feedback_ready_wall_monotonic is not None and
                  now - probe.feedback_ready_wall_monotonic >= arguments.duration):
                break
            elif (probe.feedback_ready_wall_monotonic is None and
                  now - probe.first_receive_wall_monotonic >= arguments.discovery_timeout):
                break
            rclpy.spin_once(probe, timeout_sec=0.1)
    finally:
        elapsed = time.monotonic() - started
        result = {
            "classification": "PASSIVE_JOINTSTATE_CONTINUITY_PROBE",
            "topic": arguments.topic,
            "requested_duration_seconds": arguments.duration,
            "subscription_reliability": "BEST_EFFORT" if arguments.best_effort else "RELIABLE",
            "wall_elapsed_seconds": elapsed,
            "first_receive_delay_seconds": (
                probe.first_receive_wall_monotonic - started
                if probe.first_receive_wall_monotonic is not None
                else None
            ),
            "discovery_timeout_seconds": arguments.discovery_timeout,
            "feedback_gate": {
                "state": probe.gate.state,
                "feedback_ready": probe.gate.ready,
                "stable_duration_seconds": probe.gate.stable_duration_ns / 1e9,
                "warmup_max_gap_seconds": probe.gate.warmup_max_gap_ns / 1e9,
                "runtime_stale_gap_seconds": probe.gate.runtime_stale_gap_ns / 1e9,
                "measurement_message_count": len(probe.measurement_source_ns),
                "measurement_source": series_stats(probe.measurement_source_ns),
                "measurement_receive_monotonic": series_stats(probe.measurement_receive_monotonic_ns),
            },
            "message_count": len(probe.source_ns),
            "source_ros_timestamp": series_stats(probe.source_ns),
            "receive_monotonic_timestamp": series_stats(probe.receive_monotonic_ns),
            "receive_ros_timestamp": series_stats(probe.receive_ros_ns),
            "position_invalid": probe.invalid_position,
            "velocity_invalid": probe.invalid_velocity,
            "missing_joint": probe.missing_joint,
            "effort_unsupported": probe.effort_unsupported,
            "joint_orders": sorted(probe.orders),
            "gap_events_ge_50ms": probe.gap_events,
            "publisher_created": False,
            "service_client_created": False,
            "command_issued": False,
        }
        payload = json.dumps(result, indent=2)
        print(payload)
        if arguments.output:
            with open(arguments.output, "x", encoding="utf-8") as stream:
                stream.write(payload + "\n")
                stream.flush()
        probe.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
