import importlib.util
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('adapter',ROOT/'03_shadow_mode/read_only_state_adapter.py')
adapter=importlib.util.module_from_spec(spec);spec.loader.exec_module(adapter)

def feed(gate,count,start_source=0,start_receive=0,period=10_000_000):
    result=None
    for index in range(count):
        result=gate.observe(source_ns=start_source+index*period,
                            receive_monotonic_ns=start_receive+index*period)
    return result

def test_warmup_requires_continuous_two_seconds():
    gate=adapter.FeedbackReadinessGate()
    assert not feed(gate,200)['feedback_ready']
    assert feed(gate,2,start_source=2_000_000_000,start_receive=2_000_000_000)['feedback_ready']

def test_initial_discovery_gap_restarts_warmup():
    gate=adapter.FeedbackReadinessGate()
    feed(gate,10)
    status=gate.observe(source_ns=3_100_000_000,receive_monotonic_ns=3_100_000_000)
    assert not status['feedback_ready']
    assert status['exclusion_reason']=='initial_discovery_warmup'
    assert feed(gate,202,start_source=3_110_000_000,start_receive=3_110_000_000)['feedback_ready']

def test_runtime_stale_revokes_ready():
    gate=adapter.FeedbackReadinessGate()
    assert feed(gate,202)['feedback_ready']
    status=gate.observe(source_ns=4_000_000_000,receive_monotonic_ns=4_000_000_000)
    assert not status['feedback_ready']
    assert status['exclusion_reason']=='runtime_stale_feedback'

def test_duplicate_timestamp_never_ready():
    gate=adapter.FeedbackReadinessGate()
    feed(gate,202)
    status=gate.observe(source_ns=2_010_000_000,receive_monotonic_ns=2_020_000_000)
    assert not status['feedback_ready']
    assert status['exclusion_reason']=='invalid_feedback'
