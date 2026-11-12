"""Offline CLI serialization regression; no ROS imports or physical transport."""
import json
import replay_live_command_segment as replay


def test_replay_writes_json_summary(tmp_path,monkeypatch,capsys):
    trial=tmp_path/'input';trial.mkdir()
    prediction=dict(step=0,camera_timing=dict(inference_start_monotonic=0.),
        current_controller_pose=[400,0,500,0,-155,179],
        action_pipeline=dict(limited_canonical_action=[.001,0,0,0,0,0,0]))
    command=dict(pos=[400,0,500,0,-155,179],requested_monotonic=.4,source_monotonic=0.)
    (trial/'predictions.jsonl').write_text(json.dumps(prediction)+'\n')
    (trial/'servo_commands.jsonl').write_text(json.dumps(command)+'\n')
    output=tmp_path/'output'
    monkeypatch.setattr('sys.argv',['replay','--trial',str(trial),'--output',str(output)])
    replay.main()
    summary=json.loads((output/'replay_summary.json').read_text())
    assert summary['physical_commands']==0 and not summary['transport_created']
    assert type(summary['deadline_limited_segments']) is int
    assert summary['first_inhibit'] is None
