#!/usr/bin/python3
"""Read-only camera QoS diagnostic; creates only Image subscriptions."""
import argparse
import json
import time

import numpy as np


def percentile(values, q):
    array = np.asarray(values, dtype=float)
    return float(np.percentile(array, q)) if array.size else None


def run_case(rclpy, image_type, qos, label, topic, duration_s, minimal_callback=False):
    node = rclpy.create_node("v2_camera_qos_diag_" + label)
    received = []
    callback_ms = []
    converted_bytes = []

    def callback(msg):
        started = time.perf_counter()
        host_time = time.monotonic()
        if minimal_callback:
            rgb = None
        else:
            h, w = int(msg.height), int(msg.width)
            channels = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4}.get(msg.encoding)
            if channels is None:
                raise ValueError("unsupported camera encoding: " + str(msg.encoding))
            rows = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, int(msg.step))
            image = rows[:, :w * channels].reshape(h, w, channels)
            if msg.encoding in ("bgr8", "bgra8"):
                order = [2, 1, 0] + ([3] if channels == 4 else [])
                image = image[..., order]
            rgb = np.ascontiguousarray(image[..., :3])
        received.append(host_time)
        converted_bytes.append(None if rgb is None else int(rgb.nbytes))
        callback_ms.append((time.perf_counter() - started) * 1000.0)

    sub = node.create_subscription(image_type, topic, callback, qos)
    start = time.monotonic()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    try:
        while time.monotonic() - start < duration_s:
            executor.spin_once(timeout_sec=0.05)
    finally:
        executor.remove_node(node)
        node.destroy_subscription(sub)
        node.destroy_node()
    elapsed = time.monotonic() - start
    intervals = np.diff(np.asarray(received)) * 1000.0 if len(received) > 1 else np.array([])
    return {
        "qos": label,
        "duration_s": elapsed,
        "received_frames": len(received),
        "receive_rate_hz": len(received) / elapsed,
        "reception_percent_of_30hz": 100.0 * len(received) / (30.0 * elapsed),
        "interval_ms": {
            "p50": percentile(intervals, 50), "p99": percentile(intervals, 99),
            "max": float(np.max(intervals)) if intervals.size else None,
        },
        "callback_including_bgra_to_rgb_ms": {
            "p50": percentile(callback_ms, 50), "p99": percentile(callback_ms, 99),
            "max": max(callback_ms) if callback_ms else None,
        },
        "interval_over_100ms_count": int(np.sum(intervals > 100.0)),
        "interval_over_1s_count": int(np.sum(intervals > 1000.0)),
        "minimal_callback":bool(minimal_callback),
        "converted_rgb_bytes": converted_bytes[-1] if converted_bytes else None,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/zed/zed_node/rgb/color/rect/image")
    parser.add_argument("--duration", type=float, default=20.0)
    parser.add_argument("--output", required=True)
    parser.add_argument("--modified-only",action="store_true")
    args = parser.parse_args()
    import rclpy
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from sensor_msgs.msg import Image
    rclpy.init()
    try:
        cases = [
            (qos_profile_sensor_data, "sensor_data_best_effort"),
            (QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                        reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.VOLATILE), "reliable_depth1_volatile"),
        ]
        if args.modified_only:cases=cases[1:]
        result = {"topic": args.topic, "measurement_order": [x[1] for x in cases], "cases": []}
        for qos, label in cases:
            result["cases"].append(run_case(rclpy, Image, qos, label, args.topic,args.duration,args.modified_only))
        with open(args.output, "w", encoding="utf-8") as stream:
            json.dump(result, stream, indent=2)
            stream.write("\n")
        print(json.dumps(result, indent=2))
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
