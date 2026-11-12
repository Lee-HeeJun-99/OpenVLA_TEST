import copy
from pathlib import Path
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
import yaml
from speedl_experimental.joint_safety import JointRangePredictor, ProtectedSpeedLBoundary, CANONICAL_JOINTS
from speedl_experimental.controller import SpeedLShadowController
from servo_experimental.stop_evidence import evaluate_stop
from servo_experimental.live_guard import StreamGuard

ROOT=Path(__file__).resolve().parents[1]
CONFIG=yaml.safe_load((ROOT/'configs/joint_safety.yaml').read_text())
URDF=ROOT.parents[2]/'validation/a0509_project/urdf/a0509.urdf'
Q=np.deg2rad([-182.55277,-1.07182,-41.00585,1.25662,-113.50009,1.49975])


def row(q=None):
    return dict(names=list(CANONICAL_JOINTS),position=(Q if q is None else q).tolist(),
                receive=1.,source=1.,valid=True,phase='RUNTIME_READY',source_gap=.01,receive_gap=.01)


def predictor(config=None):return JointRangePredictor(URDF, config or CONFIG)


def candidate():return dict(vel=[0]*6,acc=[20.,20.],time=.1,source_monotonic=1.)


def test_geometric_jacobian_matches_independent_finite_difference():
    model=predictor().model;t,j=model.matrix_and_jacobian(Q)
    for i in range(6):
        q=Q.copy();q[i]+=1e-7;t2,_=model.matrix_and_jacobian(q)
        v=(t2[:3,3]-t[:3,3])/1e-7
        w=(Rotation.from_matrix(t2[:3,:3])*Rotation.from_matrix(t[:3,:3]).inv()).as_rotvec()/1e-7
        np.testing.assert_allclose(j[:,i],np.r_[v,w],atol=2e-7)


def test_order_by_name_no_wrap_and_unknown_controller_blocks():
    p=predictor();r=row();idx=[2,5,0,4,1,3]
    r['names']=[r['names'][i] for i in idx];r['position']=[r['position'][i] for i in idx]
    d=p.assess(r,[0]*6,now=1)
    assert d['allowed'] and not d['physical_dispatch_allowed']
    assert d['predicted_min_deg'][0] < -180
    assert 'controller_limits_verified' in d['live_blockers']


def test_predicts_j5_crossing_before_observed_limit():
    p=predictor();q=Q.copy();q[4]=np.deg2rad(-134.99)
    _,j=p.model.matrix_and_jacobian(q);dq=np.array([0,0,0,0,-.1,0])
    v=j@dq;twist=np.r_[v[:3]*1000,np.rad2deg(v[3:])]
    d=p.assess(row(q),twist,now=1)
    assert not d['allowed'] and 'joint_5' in d['reason']
    assert d['predicted_min_deg'][4] < -135


def test_measured_violation_stale_gap_and_singular_reject():
    p=predictor();q=Q.copy();q[4]=np.deg2rad(-137.5)
    assert not p.assess(row(q),[0]*6,now=1)['allowed']
    assert 'stale' in p.assess(row(),[0]*6,now=1.5)['reason']
    r=row();r['source_gap']=.1
    assert 'gap' in p.assess(r,[0]*6,now=1)['reason']
    assert 'singular' in p.assess(row(np.zeros(6)),[1,0,0,0,0,0],now=1)['reason']


def test_local_ik_inside_and_outside_limits():
    p=predictor();t,_=p.model.matrix_and_jacobian(Q)
    pose=np.r_[t[:3,3]*1000,Rotation.from_matrix(t[:3,:3]).as_euler('ZYZ',degrees=True)]
    assert p.assess(row(),[0]*6,now=1,target_pose=pose)['allowed']
    q=Q.copy();q[4]=np.deg2rad(-134.99)
    qbad=q.copy();qbad[4]=np.deg2rad(-136)
    t,_=p.model.matrix_and_jacobian(qbad)
    pose=np.r_[t[:3,3]*1000,Rotation.from_matrix(t[:3,:3]).as_euler('ZYZ',degrees=True)]
    assert not p.assess(row(q),[0]*6,now=1,target_pose=pose)['allowed']


def test_speedl_protected_candidate_integrates_joint_rejection():
    limits=yaml.safe_load((ROOT.parent/'01_configs/safety_limits.yaml').read_text())
    c=SpeedLShadowController(limits);q=Q.copy();q[4]=np.deg2rad(-137.5)
    r=c.protected_candidate([400,0,500,0,0,0],[401,0,500,0,0,0],now=1,source_time=1,
        jointstate=row(q),joint_predictor=predictor())
    assert c.aborted and not r['joint_protection']['allowed']
    assert r['joint_protection']['rejection_stage']=='measured_joint_bounds'
    assert not r['command_issued']


def test_unverified_contract_logs_and_dispatches_zero_forever():
    logs=[];calls=[];stops=[]
    g=ProtectedSpeedLBoundary(predictor(),logs.append,calls.append,lambda:stops.append(1))
    c=candidate()
    with pytest.raises(ValueError,match='unverified'):g.submit(row(),c,now=1)
    assert logs and not calls and not stops
    with pytest.raises(RuntimeError):g.submit(row(),c,now=1)


def test_fake_verified_contract_violation_after_dispatch_stops_once():
    cfg=copy.deepcopy(CONFIG)
    # Synthetic test flags, never a real controller evidence artifact.
    for k in ('controller_limits_verified','physical_speedl_contract_verified','stop_latency_bound_verified'):cfg[k]=True
    logs=[];calls=[];stops=[];g=ProtectedSpeedLBoundary(predictor(cfg),logs.append,calls.append,lambda:stops.append(1))
    c=candidate()
    g.submit(row(),c,now=1);assert len(calls)==1
    q=Q.copy();q[4]=np.deg2rad(-136)
    with pytest.raises(ValueError):g.submit(row(q),c,now=1)
    g.abort('second_abort');assert stops==[1] and len(calls)==1
    g.confirm_stop(dict(confirmed=False))
    with pytest.raises(RuntimeError):g.submit(row(),c,now=1)


def test_logger_fault_never_sends():
    def bad(_):raise OSError('disk_full')
    calls=[];g=ProtectedSpeedLBoundary(predictor(),bad,calls.append,lambda:None)
    with pytest.raises(OSError):g.submit(row(),candidate(),now=1)
    assert not calls and g.closed


def test_logger_stall_rechecked_before_fake_dispatch():
    cfg=copy.deepcopy(CONFIG)
    for k in ('controller_limits_verified','physical_speedl_contract_verified','stop_latency_bound_verified'):cfg[k]=True
    t=[0.];calls=[]
    def slow(_):t[0]=.1
    g=ProtectedSpeedLBoundary(predictor(cfg),slow,calls.append,lambda:None,clock=lambda:t[0])
    with pytest.raises(ValueError,match='stall'):g.submit(row(),candidate(),now=1)
    assert not calls and g.closed and g.events[-1]['reason']=='speedl_pre_dispatch_stall'


@pytest.mark.parametrize('field,value', [('time',float('nan')),('time',5.),('acc',[999,999]),('vel',[1000,0,0,0,0,0])])
def test_invalid_speedl_transport_inhibits_without_send(field,value):
    c=candidate();c[field]=value;calls=[]
    g=ProtectedSpeedLBoundary(predictor(),lambda _:None,calls.append,lambda:None)
    with pytest.raises(ValueError):g.submit(row(),c,now=1)
    assert not calls and g.closed


def stopped_rows():
    return [dict(receive=1+i*.01,source=1+i*.01,valid=True,velocity=[0]*6) for i in range(25)]


def proof(**changes):
    kw=dict(ack=True,requested_at=1.,robot_state=dict(success=True,robot_state=1),
            state_received_at=1.24,joint_rows=stopped_rows(),now=1.25)
    kw.update(changes);return evaluate_stop(**kw)


def test_stop_proof_fresh_stationary_then_no_resume():
    assert proof()['confirmed']
    calls=[];g=StreamGuard(lambda:calls.append('stop'))
    g.publish(lambda:calls.append('send'));g.finish()
    g.confirm_stop(ack=True,requested_at=1.,robot_state=dict(success=True,robot_state=1),
        state_received_at=1.24,joint_rows=stopped_rows(),now=1.25)
    with pytest.raises(RuntimeError):g.publish(lambda:calls.append('bad'))
    assert calls==['send','stop']


@pytest.mark.parametrize('changes',[
    dict(ack=False),dict(robot_state=dict(success=True,robot_state=10)),
    dict(state_received_at=.9),dict(state_received_at=0),dict(joint_rows=[]),
    dict(joint_rows=[dict(receive=1.1,source=1.1,valid=True,velocity=[0]*6)]),
    dict(now=2),dict(joint_rows=[dict(receive=1.24,source=1.24,valid=True,velocity=[.01]*6)])])
def test_stop_unknown_ack_only_or_motion_never_confirms(changes):
    assert not proof(**changes)['confirmed']


def test_stop_failure_latches_first_reason_and_never_retries():
    calls=[];g=StreamGuard(lambda:calls.append('stop'))
    g.publish(lambda:None);g.finish('robot_abnormal_state:10')
    evidence=g.confirm_stop(ack=False,requested_at=1.,robot_state={},state_received_at=None,joint_rows=[],now=2.)
    assert not evidence['confirmed'] and g.reason=='robot_abnormal_state:10'
    g.finish('again');assert calls==['stop']
    with pytest.raises(RuntimeError):g.publish(lambda:calls.append('bad'))
