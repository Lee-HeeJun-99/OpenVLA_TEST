"""Offline raw 8-degree gate; default paths and angular dynamics unchanged."""
import math
from pathlib import Path
import pytest
import yaml
from run_real_rollout import SafetyPipeline


def evaluate(angle,limit=8.):
    pipeline=SafetyPipeline('openvla',operator_confirmed_initial_open=False,arm_only=True,
                            translation_endpoint_delta=True,raw_rotation_step_limit_deg=limit)
    return pipeline.runtime.evaluate([0,0,0,0,0,math.radians(angle),1],
        action_id='test',source_monotonic=1.,now_monotonic=1.,
        current_position_m=(.4,0,.5),current_abc_deg=(0,0,0),phase='alignment',arm_only=True)


@pytest.mark.parametrize('angle,accepted',[(4.149291218,True),(7.9,True),(8.,True),(8.1,False)])
def test_boundary(angle,accepted):
    row=evaluate(angle)
    assert row['technical_valid']==accepted
    assert row['effective_raw_rotation_limit_deg']==8.
    assert math.degrees(abs(row['limited_canonical_action'][5])) <= .8+1e-9


def test_default_and_config():
    assert 'raw_rotation_step_limit' in evaluate(4.1,4.)['technical_blockers']
    root=Path(__file__).resolve().parents[1]
    assert yaml.safe_load((root/'configs/servo_episode.yaml').read_text())['raw_rotation_step_limit_deg']==8.
    assert yaml.safe_load((root.parent/'01_configs/safety_limits.yaml').read_text())['rotation_step_deg']==4.


@pytest.mark.parametrize('limit',[0,-1,float('nan'),float('inf'),180])
def test_invalid_limits(limit):
    with pytest.raises(ValueError):evaluate(1,limit)
