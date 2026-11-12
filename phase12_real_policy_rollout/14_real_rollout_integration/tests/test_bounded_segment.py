"""Offline geometry/invariant tests. Draft physical runner is never called."""
import math
from pathlib import Path
import numpy as np
import pytest
import yaml
from scipy.spatial.transform import Rotation
from servo_experimental.bounded_segment import normalized_zyz,CommandSegmentInterpolator
from servo_experimental.reference_replay import pose_distance
from draft_servo_C_wrap_test import candidate_pose,prepare_geometry,validate_site_contract


LIMITS=yaml.safe_load((Path(__file__).resolve().parents[2]/'01_configs/safety_limits.yaml').read_text())


@pytest.mark.parametrize('b',[0,1e-9,-1e-9,180,-180,179.999999,-179.999999,-155])
def test_normalized_equivalent_angles_reconstruct(b):
    previous=np.array([179.,b,179.])
    for c in (179.,180.,181.,182.,-179.,-180.,-181.,540.):
        r=Rotation.from_euler('ZYZ',[179.,b,c],degrees=True)
        abc=normalized_zyz(r,previous)
        assert np.all(abc>=-180) and np.all(abc<=180)
        assert (Rotation.from_euler('ZYZ',abc,degrees=True).inv()*r).magnitude()<1e-6
        previous=abc


def test_clamped_segment_budget_and_hold():
    tracker=CommandSegmentInterpolator([400,0,500,10,-155,179],LIMITS)
    tracker.set_action([.003,.004,0,0,0,math.radians(3),0],0.,0.,period=.5)
    rows=[tracker.tick(t) for t in np.linspace(0,1.,41)]
    assert abs(rows[-1]['cumulative_translation_mm']-5)<1e-6
    assert abs(rows[-1]['cumulative_rotation_deg']-3)<1e-6
    assert pose_distance(rows[20]['pos'],rows[-1]['pos'])[0]<1e-10
    assert pose_distance(rows[20]['pos'],rows[-1]['pos'])[1]<1e-8


def test_retarget_uses_last_command_without_jump():
    tracker=CommandSegmentInterpolator([400,0,500,0,-155,179],LIMITS)
    tracker.set_action([.01,0,0,0,0,.03,0],0.,0.,period=1.)
    for t in np.linspace(0,.25,6):old=tracker.tick(float(t))['pos']
    tracker.set_action([-.002,0,0,0,0,-.01,0],.3,.3)
    first=tracker.tick(.3)['pos']
    assert pose_distance(old,first)[0]<1e-9
    assert pose_distance(old,first)[1]<1e-8
    assert tracker.segment['duration']==.3


def test_random_preemptions_preserve_budget():
    rng=np.random.default_rng(19)
    tracker=CommandSegmentInterpolator([400,0,500,179,-155,179],LIMITS)
    now=0.
    for _ in range(50):
        action=np.r_[rng.normal(size=3)*.001,rng.normal(size=3)*.01,.5]
        tracker.set_action(action,now,now,period=.5)
        for j in range(7):
            row=tracker.tick(now+j*.05)
            assert row['cumulative_translation_mm']<=np.linalg.norm(action[:3])*1000+1e-6
            assert row['cumulative_rotation_deg']<=np.linalg.norm(action[3:6])*180/math.pi+1e-6
        now+=.4


def test_workspace_and_stale_still_inhibit():
    tracker=CommandSegmentInterpolator([799,0,500,0,0,0],LIMITS)
    with pytest.raises(ValueError,match='workspace'):tracker.set_action([.002,0,0,0,0,0,0],0.,0.)
    with pytest.raises(RuntimeError,match='inhibited'):tracker.tick(.1)
    tracker=CommandSegmentInterpolator([400,0,500,0,0,0],LIMITS)
    tracker.set_action([.001,0,0,0,0,0,0],0.,0.)
    with pytest.raises(ValueError,match='stale'):tracker.tick(1.21)


def test_tick_gap_and_invalid_action_latch():
    tracker=CommandSegmentInterpolator([400,0,500,0,0,0],LIMITS)
    tracker.set_action([.001,0,0,0,0,0,0],0.,0.)
    tracker.tick(0.)
    with pytest.raises(ValueError,match='tick_gap'):tracker.tick(.101)
    tracker=CommandSegmentInterpolator([400,0,500,0,0,0],LIMITS)
    with pytest.raises(ValueError):tracker.set_action([float('nan'),0,0,0,0,0,0],0.,0.)
    assert tracker.closed


def reviewed_fake_contract():
    # Geometry fixture only, NOT a site approval or hardware limit evidence.
    return dict.fromkeys(('reviewed_for_this_test','safe_pose_verified','controller_joint_limits_verified',
        'stop_path_verified','tcp_offset_verified','servo_transport_profile_verified'),True)|dict(
        safe_joint_deg=[0]*6,joint_lower_deg=[-90]*6,joint_upper_deg=[90]*6,
        tcp_offset_mm_zyz_deg=[0]*6,joint_margin_deg=5.,following_translation_mm=2.,
        following_rotation_deg=1.,minimum_abs_sin_B=.1,controller_version='FAKE',evidence_source='UNIT_TEST_ONLY')


@pytest.mark.parametrize('direction,initial_c',[(1,179.),(-1,-179.)])
def test_draft_modes_same_physical_rotation_low_speed(direction,initial_c):
    contract=reviewed_fake_contract()
    initial=prepare_geometry([400,0,500,10,-155,initial_c],direction,contract)
    previous=initial[3:]
    peak=0.
    for t in np.linspace(0,8,161):
        n,w=candidate_pose(initial,t,direction,'normalized',previous)
        u,_=candidate_pose(initial,t,direction,'unwrapped',previous)
        assert pose_distance(n,u)[1]<1e-8
        assert all(abs(a)<=180 for a in n[3:])
        assert n[:3]==initial[:3].tolist()
        previous=n[3:];peak=max(peak,np.linalg.norm(w))
    assert peak<1.
    assert abs(pose_distance(initial,n)[1]-3.)<1e-8
    assert abs(u[5])>180.


def test_draft_refuses_unreviewed_stop_and_joint_margin():
    contract=reviewed_fake_contract();contract['stop_path_verified']=False
    with pytest.raises(PermissionError):validate_site_contract(contract)
    contract=reviewed_fake_contract();contract['safe_joint_deg'][4]=89
    with pytest.raises(ValueError,match='joint_limit'):validate_site_contract(contract)
