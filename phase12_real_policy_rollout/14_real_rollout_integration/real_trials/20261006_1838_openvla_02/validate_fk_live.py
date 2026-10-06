"""Read-only live FK candidate and continuity gate, never a measured TCP."""
import sys,time,json,math,hashlib
from pathlib import Path
ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT.parents[2]/'03_shadow_mode'))
from jointstate_flange_fk import A0509FlangeFK,reorder_joint_state
import rclpy
from sensor_msgs.msg import JointState
from rclpy.qos import qos_profile_sensor_data
URDF=Path('/home/ubuntu/robot_ws/src/doosan-robot2/dsr_description2/urdf/a0509.urdf')
def main():
    fk=A0509FlangeFK(URDF);rclpy.init();node=rclpy.create_node('phase12_fk_gate_validation')
    rows=[];start=time.monotonic()
    def callback(m):
        now=time.monotonic();stamp=m.header.stamp.sec+m.header.stamp.nanosec/1e9
        row=dict(receive_monotonic=now,source_ros=stamp,receive_wall=time.time(),names=list(m.name),
            position=list(m.position),velocity=list(m.velocity),command_issued=False,executed_action=None,robot_delivered_command=None)
        try:
            positions=reorder_joint_state(m.name,m.position)
            if len(m.velocity)!=len(m.name) or not all(math.isfinite(v) for v in m.velocity):raise ValueError('invalid_velocity')
            row['fk']=fk.compute(positions);row['valid']=True
        except (ValueError,KeyError) as exc:row.update(valid=False,reason=str(exc))
        rows.append(row)
    node.create_subscription(JointState,'/dsr01/joint_states',callback,qos_profile_sensor_data)
    while time.monotonic()-start<60:rclpy.spin_once(node,timeout_sec=.02)
    now=time.monotonic();rg=[b['receive_monotonic']-a['receive_monotonic'] for a,b in zip(rows,rows[1:])]
    sg=[b['source_ros']-a['source_ros'] for a,b in zip(rows,rows[1:])]
    rate=(len(rows)-1)/(rows[-1]['receive_monotonic']-rows[0]['receive_monotonic']) if len(rows)>1 else 0
    gate=len(rows)>1 and rate>=95 and max(rg)<.1 and max(sg)<.1 and all(r['valid'] for r in rows) and all(s>0 for s in sg) and now-rows[-1]['receive_monotonic']<=.5
    report=dict(classification='READ_ONLY_FK_CANDIDATE_VALIDATION',duration_s=now-start,count=len(rows),
        first_delay_s=rows[0]['receive_monotonic']-start if rows else None,steady_receive_hz=rate,
        max_receive_gap_s=max(rg) if rg else None,max_source_gap_s=max(sg) if sg else None,
        source_gaps_ge_100ms=sum(g>=.1 for g in sg),source_regressions=sum(g<0 for g in sg),
        latest_age_s=now-rows[-1]['receive_monotonic'] if rows else None,
        jointstate_gate='PASS' if gate else 'FAIL',invalid_count=sum(not r['valid'] for r in rows),
        latest_fk_candidate=rows[-1].get('fk') if rows else None,urdf_sha256=hashlib.sha256(URDF.read_bytes()).hexdigest(),
        tcp_gate='BLOCKED_PENDING_CURRENT_OPERATOR_TOOL_AND_POSE_CROSSCHECK',motion_authorized=False,
        robot_commands=0,executed_action=None,robot_delivered_command=None)
    for name,data in [('fk_live_samples.json',rows),('fk_live_validation.json',report)]:
        with (ROOT/name).open('x') as f:json.dump(data,f,indent=2)
    print(json.dumps(report,indent=2));node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
