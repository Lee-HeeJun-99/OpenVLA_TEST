import math
from pathlib import Path
import numpy as np
import pytest
import yaml
from speedl_experimental.controller import SpeedLShadowController

LIMITS = yaml.safe_load((Path(__file__).resolve().parents[2]/'01_configs/safety_limits.yaml').read_text())


def test_units_direction_and_slew():
    c = SpeedLShadowController(LIMITS)
    last = np.zeros(6)
    for i in range(10):
        r = c.candidate([400, 0, 500, 0, 0, 0], [410, 0, 490, 30, 0, 0], now=i*.05, source_time=0)
        v = np.array(r['vel'])
        assert v[0] > 0 and v[2] < 0 and v[5] > 0
        for s, a in [(slice(0, 3), 20), (slice(3, 6), 20)]:
            assert np.linalg.norm(v[s]) <= 20+1e-9
            assert np.linalg.norm(v[s]-last[s]) <= a*.05+1e-9
        assert not r['command_issued'] and not r['physical_rollout_ready']
        assert r['time'] == .1
        last = v


def test_equivalent_zyz_does_not_command_large_rotation():
    r = SpeedLShadowController(LIMITS).candidate([400, 0, 500, 0, 155, -176], [400, 0, 500, 180, -155, 4], now=0, source_time=0)
    assert np.linalg.norm(r['vel'][3:]) < 1e-9


@pytest.mark.parametrize('target,now,source', [
    ([math.nan, 0, 500, 0, 0, 0], 0, 0),
    ([999, 0, 500, 0, 0, 0], 0, 0),
    ([400, 0, 500, 0, 0, 0], 2, 0),
    ([400, 0, 500, 0, 0, 0], 0, 1)])
def test_fault_latches_inhibit(target, now, source):
    c = SpeedLShadowController(LIMITS)
    with pytest.raises(ValueError):
        c.candidate([400, 0, 500, 0, 0, 0], target, now=now, source_time=source)
    assert c.aborted
    with pytest.raises(RuntimeError):
        c.candidate([400, 0, 500, 0, 0, 0], [400, 0, 500, 0, 0, 0], now=0, source_time=0)


def test_tick_timeout_and_no_publish_after_abort():
    c = SpeedLShadowController(LIMITS)
    c.candidate([400, 0, 500, 0, 0, 0], [400, 0, 500, 0, 0, 0], now=0, source_time=0)
    with pytest.raises(ValueError, match='tick_gap'):
        c.candidate([400, 0, 500, 0, 0, 0], [400, 0, 500, 0, 0, 0], now=.1, source_time=0)
