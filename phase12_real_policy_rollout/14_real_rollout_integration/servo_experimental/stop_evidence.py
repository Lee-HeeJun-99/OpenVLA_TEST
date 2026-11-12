"""Read-only stop proof: ACK + fresh STANDBY + post-request clean joint window."""
import math
from settle_window import ExactZeroSettleWindow


def evaluate_stop(*, ack, requested_at, robot_state, state_received_at, joint_rows, now):
    evidence = dict(confirmed=False, additional_dispatch_allowed=False,
                    comparison_clock='HOST_MONOTONIC_RECEIVE_TIME',
                    ack=ack is True, criterion='ACK_AND_FRESH_STANDBY_AND_POST_STOP_EXACT_ZERO_WINDOW')
    try:
        if ack is not True:raise ValueError('stop_ack_not_successful')
        if any(t is None for t in (requested_at, state_received_at, now)):
            raise ValueError('missing_stop_timestamps')
        if not all(type(t) in (float, int) and math.isfinite(t) for t in (requested_at, state_received_at, now)):
            raise ValueError('invalid_stop_timestamps')
        if not requested_at <= state_received_at <= now or now-state_received_at >= .5:
            raise ValueError('stop_state_stale_or_before_request')
        if (robot_state.get('success') is not True or type(robot_state.get('robot_state')) is not int
                or robot_state.get('robot_state') != 1):
            raise ValueError('final_robot_not_STANDBY')
        window = ExactZeroSettleWindow(); ready = False
        for row in joint_rows:
            if row['receive'] >= requested_at:ready = window.sample(row)
        if not ready or not 0 <= now-window.last['receive'] < .1:
            raise ValueError('post_stop_stationary_window_not_confirmed')
        evidence.update(confirmed=True, reason=None, stationary_window=window.evidence())
    except (ValueError, TypeError, KeyError, RuntimeError) as exc:
        evidence['reason'] = str(exc)
    return evidence
