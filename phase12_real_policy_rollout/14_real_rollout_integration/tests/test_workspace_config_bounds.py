"""Offline equality/boundary tests for policy and periodic tracker workspace."""
from pathlib import Path
import pytest
import yaml
from run_real_rollout import SafetyPipeline
from workspace_contract import PHASE12_DATA_DERIVED_WORKSPACE as W
from servo_experimental.controller import ServoShadowController


LIMITS=yaml.safe_load((Path(__file__).resolve().parents[2]/'01_configs/safety_limits.yaml').read_text())


@pytest.mark.parametrize('axis,lo,hi',[(0,0.,.8),(1,-.8,.8),(2,-.05,1.)])
def test_boundaries_match_tracker(axis,lo,hi):
    assert W.minimum_m[axis]==lo
    assert W.maximum_m[axis]==hi
    tracker=ServoShadowController([400,0,500,0,0,0],LIMITS)
    for value in (lo,hi):
        p=[.4,0,.5];p[axis]=value
        assert W.inspect(p).accepted
        tracker._workspace([v*1000 for v in p])
    for value in (lo-1e-6,hi+1e-6):
        p=[.4,0,.5];p[axis]=value
        assert not W.inspect(p).accepted
        with pytest.raises(ValueError,match='workspace_rejection'):
            tracker._workspace([v*1000 for v in p])


def test_endpoint_rejection_remains_active_inside_raw_step_gate():
    pipeline=SafetyPipeline('openvla',operator_confirmed_initial_open=False,arm_only=True,
                            translation_endpoint_delta=True)
    row=pipeline.runtime.evaluate([.002,0,0,0,0,0,1],action_id='edge',
        source_monotonic=1.,now_monotonic=1.,current_position_m=(.799,0,.5),
        current_abc_deg=(0,0,0),phase='alignment',arm_only=True)
    assert 'workspace_violation' in row['technical_blockers']
    assert not row['technical_valid']
