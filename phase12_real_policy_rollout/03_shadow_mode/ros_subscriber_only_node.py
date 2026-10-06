"""ROS2 subscriber-only skeleton. Intentionally defines no publisher/client/action client."""
from __future__ import annotations

def assert_safe_flags(config, validate_startup):
    validate_startup(config)

def subscriber_spec(config):
    """Return passive subscription declarations for a local operator launch wrapper."""
    return {
      'camera':config['topics']['camera'],
      'joint_state':config['topics']['joint_state'],
      'measured_tcp_pose':config['topics']['measured_tcp_pose'],
      'safety_state':config['topics'].get('safety_state'),
    }

# Live ROS construction is deliberately deferred. Gate 1 observed none of the
# required topics, so instantiating subscriptions would not validate real data.
