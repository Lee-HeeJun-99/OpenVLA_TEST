"""Selected-frame freshness at request start, not response completion."""
import math


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
