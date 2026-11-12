"""No ROS: fake evidence does not approve hardware execution."""
import copy
from pathlib import Path
import math
import numpy as np
import pytest
import yaml
from servo_experimental.live_segment import LiveCommandSegment, require_release, stream_planning_limits
from servo_experimental.reference_replay import pose_distance

ROOT=Path(__file__).resolve().parents[1]
LIMITS=yaml.safe_load((ROOT.parent/'01_configs/safety_limits.yaml').read_text())
CONFIG=yaml.safe_load((ROOT/'configs/servo_episode.yaml').read_text())
FAKE=dict(normalized_euler_controller_verified=True, physical_stop_verified=True,
    controller_joint_protection_verified=True, following_translation_mm=10.,
    following_rotation_deg=3., feedback_max_age_s=.5)

def test_double_stream_profile_preserves_other_thresholds():
    original=copy.deepcopy(LIMITS)
    effective=stream_planning_limits(LIMITS,CONFIG)
    for key in CONFIG['stream_planning']:
        assert effective[key]==2*LIMITS[key]==40.
    for key in set(LIMITS)-set(CONFIG['stream_planning']):
        assert effective[key]==LIMITS[key]
    assert LIMITS==original
    assert CONFIG['servo_transport']['velocity_mm_s_deg_s']==[40.,40.]
    assert CONFIG['servo_transport']['acceleration_mm_s2_deg_s2']==[80.,80.]

def tracker():
    return LiveCommandSegment([400,0,500,0,-155,179],LIMITS,
        transport=CONFIG['servo_transport'],protection=FAKE)

def test_repository_release_blocks_before_ros_or_prompts(tmp_path,monkeypatch):
    import start_servo_episode_confirmed as launcher
    monkeypatch.setattr('builtins.input',lambda *a:pytest.fail('must not prompt'))
    with pytest.raises(PermissionError,match='servo_release_blocked'):
        launcher.run(tmp_path/'not_created')
    assert not (tmp_path/'not_created').exists()

def test_release_does_not_accept_unknown_or_nonfinite():
    for key in FAKE:
        config=copy.deepcopy(FAKE);config[key]=None
        with pytest.raises(PermissionError):require_release(config)

def test_quintic_no_overshoot_or_dynamic_excess():
    t=tracker();initial=t.pose()
    t.set_action([.001,0,0,0,0,math.radians(1),0],0.,0.)
    for now in np.arange(0,1.2,.05):
        row=t.tick(float(now));t.mark_published(float(now),row['pos'])
        assert row['cumulative_translation_mm']<=1+1e-6
        assert row['cumulative_rotation_deg']<=1+1e-6
        assert max(abs(v) for v in row['pos'][3:])<=180
    assert pose_distance(initial,t.pose())[0]==pytest.approx(1)

def test_feedback_inhibits_and_age_not_rebased():
    t=tracker();t.set_action([.001,0,0,0,0,0,0],0.,0.)
    with pytest.raises(RuntimeError,match='following_error'):
        t.verify_feedback(dict(pose=[450,0,500,0,-155,179],received=.1),.1)
    with pytest.raises(RuntimeError,match='inhibited'):t.tick(.2)
    t=tracker();t.set_action([.001,0,0,0,0,0,0],0.,.5)
    with pytest.raises(ValueError,match='stale'):t.tick(1.21)

def test_retarget_queues_without_resetting_moving_profile():
    t=tracker();t.set_action([.001,0,0,0,0,math.radians(1),0],0.,0.)
    for now in np.arange(0,.25,.05):t.tick(float(now))
    before=copy.deepcopy(t.segment)
    state=t.set_action([-.001,0,0,0,0,-math.radians(1),0],.25,.25)
    assert state['state']=='QUEUED_LATEST_UNTIL_REST'
    np.testing.assert_array_equal(t.segment['start'],before['start'])
    # Brake starts at the last command, with exactly inherited derivatives.
    u=(.2-before['activated'])/before['duration']
    c=before['profile_coeff']
    assert t.segment['braking']
    assert t.segment['brake_initial_progress']==pytest.approx(np.polynomial.polynomial.polyval(u,c))
    assert t.segment['brake_initial_velocity']==pytest.approx(np.polynomial.polynomial.polyval(u,np.polynomial.polynomial.polyder(c))/before['duration'])
    assert t.segment['brake_initial_acceleration']==pytest.approx(np.polynomial.polynomial.polyval(u,np.polynomial.polynomial.polyder(c,2))/before['duration']**2)
    seen=[]
    for now in np.arange(.25,1.2,.05):
        row=t.tick(float(now));seen.append(row)
    assert any(row['source_monotonic']==.25 for row in seen)
    assert not t.closed
    for row in seen:
        assert np.linalg.norm(row['planning_acceleration'][:3])<=LIMITS['translation_acceleration_mm_s2']+1e-6
        assert np.linalg.norm(row['planning_acceleration'][3:])<=LIMITS['rotation_acceleration_deg_s2']+1e-6

def test_pending_is_latest_only_and_source_age_not_reset():
    t=tracker();t.set_action([.001,0,0,0,0,0,0],0.,0.)
    t.tick(0.);t.tick(.05)
    t.set_action([.001,0,0,0,0,0,0],.1,.1)
    t.set_action([-.001,0,0,0,0,0,0],.11,.11)
    assert t.pending[1]==.11 and t.replaced_pending==1
    activated=[]
    for now in np.arange(.15,1.2,.05):
        row=t.tick(float(now));activated.append(row['source_monotonic'])
    assert .1 not in activated and .11 in activated
    assert all(row_source in (0.,.11) for row_source in activated)

def test_rest_boundary_new_arrival_supersedes_queued_goal():
    t=tracker();t.set_action([.001,0,0,0,0,0,0],0.,0.)
    t.tick(0.);t.tick(.05)
    t.set_action([.001,0,0,0,0,0,0],.1,.1)
    for now in np.arange(.1,.6,.05):
        row=t.tick(float(now))
        if t.segment['profile_done']:
            break
    now=float(now)+.01
    t.set_action([-.001,0,0,0,0,0,0],now,now)
    assert t.pending is None
    assert t.segment['source']==now

def test_c2_brake_matches_boundary_derivatives_and_stays_in_budget():
    t=tracker();action=[.002,.001,0,0,0,.015,0]
    t.set_action(action,0.,0.)
    for now in np.arange(0,.3,.05):t.tick(float(now))
    t.set_action([-v for v in action],.3,.3)
    s=t.segment
    assert s['braking']
    c=s['profile_coeff'];duration=s['duration']
    assert c[0]==pytest.approx(s['brake_initial_progress'])
    assert c[1]/duration==pytest.approx(s['brake_initial_velocity'])
    assert 2*c[2]/duration**2==pytest.approx(s['brake_initial_acceleration'])
    assert np.polynomial.polynomial.polyval(1,np.polynomial.polynomial.polyder(c))/duration==pytest.approx(0,abs=1e-9)
    assert np.polynomial.polynomial.polyval(1,np.polynomial.polynomial.polyder(c,2))/duration**2==pytest.approx(0,abs=1e-8)
    assert s['brake_endpoint_fraction']<=1
    for now in np.arange(.3,1.2,.05):
        row=t.tick(float(now))
        assert row['cumulative_translation_mm']<=t.segment['translation_budget']+1e-6
        assert row['cumulative_rotation_deg']<=t.segment['rotation_budget']+1e-6

def test_random_direction_changes_with_irregular_ticks_remain_bounded():
    rng=np.random.default_rng(87);t=tracker();next_input=0.;now=0.
    for _ in range(240):
        if now>=next_input:
            action=np.r_[rng.uniform(-.00015,.00015,3),rng.uniform(-.001,.001,3),0.]
            t.set_action(action,now,now)
            next_input=now+float(rng.uniform(.22,.32))
        row=t.tick(now)
        assert row['cumulative_translation_mm']<=t.segment['translation_budget']+1e-6
        assert row['cumulative_rotation_deg']<=t.segment['rotation_budget']+1e-6
        assert np.linalg.norm(row['planning_velocity'][:3])<=LIMITS['translation_velocity_mm_s']+1e-6
        assert np.linalg.norm(row['planning_velocity'][3:])<=LIMITS['rotation_velocity_deg_s']+1e-6
        assert np.linalg.norm(row['planning_acceleration'][:3])<=LIMITS['translation_acceleration_mm_s2']+1e-6
        assert np.linalg.norm(row['planning_acceleration'][3:])<=LIMITS['rotation_acceleration_deg_s2']+1e-6
        t.mark_published(now,row['pos'])
        now+=float(rng.uniform(.045,.065))
    assert not t.closed
    assert t.braking_plans>0

def test_deadline_budget_does_not_rebase_age_or_rewrite_action():
    t=tracker();action=np.array([.02,0,0,0,0,.03,0.])
    saved=action.copy();t.set_action(action,0.,.4)
    np.testing.assert_array_equal(action,saved)
    assert t.segment['source']==0.
    assert t.segment['duration']<=.6+1e-9
    assert 0<t.segment['planned_execution_fraction']<1
    assert t.segment['target'][0]==pytest.approx(420.)
    for now in np.arange(.4,1.15,.05):
        row=t.tick(float(now))
        assert np.linalg.norm(row['planning_acceleration'][:3])<=LIMITS['translation_acceleration_mm_s2']+1e-6
        assert row['source_monotonic']==0.
    assert t.segment['profile_done']
    with pytest.raises(ValueError,match='stale'):t.tick(1.21)

def test_too_late_action_never_gets_new_source_time():
    t=tracker()
    with pytest.raises(ValueError,match='insufficient_fresh_action_horizon'):
        t.set_action([.001,0,0,0,0,0,0],0.,1.)
    assert t.closed

def test_active_source_stale_during_wait_still_inhibits():
    t=tracker();t.set_action([.02,0,0,0,0,0,0],0.,0.)
    for now in np.arange(0,1.4,.05):
        if abs(now-.2)<1e-9:t.set_action([-.001,0,0,0,0,0,0],.2,.2)
        t.tick(float(now))
    with pytest.raises(ValueError,match='stale'):t.tick(1.41)
    assert t.closed
