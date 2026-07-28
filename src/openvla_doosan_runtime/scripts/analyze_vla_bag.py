#!/usr/bin/env python3
from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import hashlib
from pathlib import Path
from typing import Iterable

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def _vector(values: Iterable[float]) -> tuple[float, ...]:
    return tuple(float(value) for value in values)


def _same_action(left: tuple[float, ...], right: tuple[float, ...], epsilon: float) -> bool:
    return np.allclose(left, right, rtol=0.0, atol=epsilon)


def _topic_reader(bag_path: Path):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )
    topic_types = {
        topic.name: topic.type
        for topic in reader.get_all_topics_and_types()
    }
    message_types = {
        topic: get_message(type_name)
        for topic, type_name in topic_types.items()
    }
    return reader, message_types


def analyze_bag(bag_path: Path, epsilon: float, max_image_hashes: int) -> None:
    reader, message_types = _topic_reader(bag_path)

    counts: Counter[str] = Counter()
    raw_actions: list[tuple[float, ...]] = []
    target_poses: list[tuple[float, ...]] = []
    current_poses: list[tuple[float, ...]] = []
    image_hashes: list[str] = []
    statuses: defaultdict[str, Counter[str]] = defaultdict(Counter)
    rosout_matches: list[str] = []

    while reader.has_next():
        topic, serialized, _timestamp = reader.read_next()
        counts[topic] += 1

        msg_type = message_types.get(topic)
        if msg_type is None:
            continue
        msg = deserialize_message(serialized, msg_type)

        if topic == "/vla/raw_action":
            raw_actions.append(_vector(msg.data))
        elif topic == "/vla/target_pose":
            target_poses.append(_vector(msg.data))
        elif topic == "/doosan/current_pose":
            current_poses.append(_vector(msg.data))
        elif topic == "/vla/image_rgb" and len(image_hashes) < max_image_hashes:
            image_hashes.append(hashlib.sha256(bytes(msg.data)).hexdigest()[:16])
        elif topic in {
            "/vla/model_status",
            "/vla/action_status",
            "/vla/episode_status",
            "/doosan/status",
        }:
            statuses[topic][str(msg.data)] += 1
        elif topic == "/rosout":
            text = str(msg.msg)
            if (
                "inference_ok:" in text
                or "repeated_action_detected" in text
                or "action_rejected:" in text
            ):
                rosout_matches.append(f"{msg.name}: {text}")

    print(f"bag: {bag_path}")
    print("topic_counts:")
    for topic, count in sorted(counts.items()):
        print(f"  {topic}: {count}")

    if raw_actions:
        unique_actions = Counter(raw_actions)
        longest_repeat = 1
        current_repeat = 1
        for previous, current in zip(raw_actions, raw_actions[1:]):
            if _same_action(previous, current, epsilon):
                current_repeat += 1
            else:
                longest_repeat = max(longest_repeat, current_repeat)
                current_repeat = 1
        longest_repeat = max(longest_repeat, current_repeat)

        print("raw_action:")
        print(f"  count: {len(raw_actions)}")
        print(f"  unique_exact: {len(unique_actions)}")
        print(f"  longest_repeat_epsilon_{epsilon}: {longest_repeat}")
        for action, count in unique_actions.most_common(5):
            print(f"  common x{count}: {list(action)}")

    if image_hashes:
        print("image_rgb:")
        print(f"  hashed: {len(image_hashes)}")
        print(f"  unique_hashes: {len(set(image_hashes))}")

    for label, poses in (("current_pose", current_poses), ("target_pose", target_poses)):
        if not poses:
            continue
        array = np.asarray(poses, dtype=np.float64)
        xyz = array[:, :3]
        print(label + ":")
        print(f"  count: {len(poses)}")
        print(f"  xyz_min: {xyz.min(axis=0).tolist()}")
        print(f"  xyz_max: {xyz.max(axis=0).tolist()}")

    if statuses:
        print("statuses:")
        for topic, counter in sorted(statuses.items()):
            print(f"  {topic}:")
            for value, count in counter.most_common(5):
                print(f"    x{count}: {value}")

    if rosout_matches:
        print("rosout_matches:")
        for line in rosout_matches[-20:]:
            print(f"  {line}")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag_path", type=Path)
    parser.add_argument("--epsilon", type=float, default=1.0e-12)
    parser.add_argument("--max-image-hashes", type=int, default=100)
    args = parser.parse_args()

    analyze_bag(
        args.bag_path.expanduser().resolve(),
        epsilon=args.epsilon,
        max_image_hashes=args.max_image_hashes,
    )


if __name__ == "__main__":
    main()
