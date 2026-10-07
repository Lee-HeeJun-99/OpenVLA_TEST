"""Recorded prediction-only batch verification. Physical automation forbidden."""
import argparse,json,sys,time
from pathlib import Path
ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT.parent/'14_real_rollout_integration'))
from rollout_batch_summary import RolloutBatch
from run_real_rollout import Controller
from real_sink import RealDoosanCommandSink,Authorization
from integrated_logger import FsyncJsonlLogger
from jointstate_runtime_startup import JointStateStartup

def simulated_readiness(generation):
    # Explicitly virtual clock fixtures, not current live stream PASS artifacts.
    state=JointStateStartup(0)
    for n in range(1003):
        t=.01+n*.01
        state.sample(dict(receive=t,source=t,header_age=0.,valid=True))
    return {**state.status(10.03),'generation':generation,'first_fresh_after_rearm':True,'warmup_seconds':10.02,'source':'MOCK_CLOCK_FIXTURE'}

def run(input_path,output,target,maximum):
    fixtures=json.loads(Path(input_path).read_text());transport_calls=[]
    def execute(trial):
        fixture=fixtures[(trial.trial_id-1)%len(fixtures)]
        path=Path(output)/f'trial_{trial.trial_id:03d}'/'controller.jsonl'
        def no_transport():transport_calls.append(True);raise AssertionError('dry_run_transport_created')
        sink=RealDoosanCommandSink(no_transport,Authorization())
        with FsyncJsonlLogger(path) as logger:
            controller=Controller('openvla','short_horizon',sink,logger,dry_run=True,initial_open=True,trial=trial)
            for index,row in enumerate(fixture['predictions']):
                obs=dict(receive_monotonic=1+index*.2,tcp_m_abc=[.4,0,.5,10,20,30],phase='alignment',
                         robot_state_ok=True,camera_ok=True,joint_state_ok=True,tcp_ok=True,model_ok=True,communication_ok=True)
                obs.update(row.get('observation',{}))
                if row.get('runtime_error'):
                    controller.abort(row['runtime_error']);break
                controller.step(row['action'],obs,inference_time=obs['receive_monotonic'],chunk_id=str(index),chunk_index=0)
                if controller.aborted:break
        return fixture.get('task_success')
    result=RolloutBatch(target,maximum).run(simulated_readiness,execute,output)
    result['transport_calls']=len(transport_calls)
    result['prediction_provenance']='SYNTHETIC_FIXTURE_NOT_CHECKPOINT_OUTPUT'
    (Path(output)/'batch_summary.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(result,indent=2));return result

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--recorded-input',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--target-valid-rollouts',type=int,default=20);p.add_argument('--max-attempts',type=int,default=30)
    a=p.parse_args();run(a.recorded_input,a.output,a.target_valid_rollouts,a.max_attempts)
