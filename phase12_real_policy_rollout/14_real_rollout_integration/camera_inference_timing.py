"""Selected-frame freshness at request start, not response completion."""
import math

# Operator-requested receive-silence watchdog. Separate from the unchanged
# 0.5s selected-frame age gate at inference start.
CAMERA_STREAM_TIMEOUT_S = 1.0


def camera_stream_lost(received, now):
    return received is None or now - received >= CAMERA_STREAM_TIMEOUT_S


def inference_timing(received, started, finished, *, source_stamp=None,
                     ros_start=None, ros_end=None):
    if not all(math.isfinite(x) for x in (received, started, finished)) or not received <= started <= finished:
        raise ValueError('invalid_camera_monotonic_timing')
    return dict(frame_receive_monotonic=received,
                inference_start_monotonic=started, inference_end_monotonic=finished,
                camera_age_at_inference_start=started-received,
                camera_age_at_inference_end=finished-received,
                inference_latency=finished-started,
                source_age_at_inference_start=None if source_stamp is None or ros_start is None else ros_start-source_stamp,
                source_age_at_inference_end=None if source_stamp is None or ros_end is None else ros_end-source_stamp)
