"""Prediction preparation and deferred artifacts. No motion/IO capability."""
import base64
import hashlib
import io
import json
import math
import os
import time
from urllib.request import Request,urlopen


def select_fresh_frame(snapshot,check,*,clock=time.monotonic,sleep=time.sleep):
    deadline=clock()+2
    while clock()<deadline:
        check();selected=snapshot()
        if selected and 0<=clock()-selected[1]<=.01:return selected
        sleep(.001)
    raise TimeoutError('fresh_camera_timeout')


def predict_frame(selected,ros_now,check,*,clock=time.monotonic,request_open=urlopen):
    from live_observation import camera_rgb_image
    from camera_inference_timing import inference_timing
    msg,received=selected
    if msg.encoding!='bgra8' or [msg.width,msg.height]!=[1280,720]:raise ValueError('camera_contract')
    rgb=camera_rgb_image(msg);buf=io.BytesIO();rgb.save(buf,format='JPEG',quality=95);jpeg=buf.getvalue()
    req=Request('http://127.0.0.1:8766/predict',data=json.dumps(dict(image_jpeg_base64=base64.b64encode(jpeg).decode(),instruction='Pick up the orange cube.')).encode(),headers={'Content-Type':'application/json'})
    ros_start=ros_now();t0=clock()
    if not 0<=t0-received<.5:raise ValueError('camera_stale_at_inference_start')
    check()
    with request_open(req,timeout=1.2) as response:prediction=json.load(response)
    t1=clock();check();raw=prediction['action']
    if len(raw)!=7 or not all(math.isfinite(v) for v in raw):raise ValueError('invalid_model_action')
    timing=inference_timing(received,t0,t1,source_stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9,ros_start=ros_start,ros_end=ros_now())
    return dict(msg=msg,rgb=rgb,jpeg=jpeg,prediction=prediction,raw=raw,timing=timing,t0=t0,t1=t1)


def warm_up(predict,check,save,*,max_action_age_s,settle_s):
    """Three discarded fresh-frame requests, before any physical sink exists.

    First request is recorded separately, not claimed to be a server cold start.
    Require both subsequent requests to leave room for the existing settle gate.
    This is a preflight check, never a replacement for actual dispatch age checks.
    """
    rows=[];last_stamp=None
    for i in range(3):
        check();sample=predict();stamp=sample['msg'].header.stamp
        key=(stamp.sec,stamp.nanosec)
        if key==last_stamp:raise RuntimeError('warmup_duplicate_camera_frame')
        last_stamp=key
        row=dict(request=i,phase='PREFLIGHT_WARMUP',first_request_this_preflight=i==0,
                 raw_action=sample['raw'],camera_timing=sample['timing'],source_stamp=list(key),
                 command_requested=False,command_issued=False,excluded_from_episode=True)
        rows.append(row);save('model_warmup.json',dict(status='RUNNING',samples=rows))
    ready=all(r['camera_timing']['inference_latency']+settle_s<max_action_age_s for r in rows[1:])
    save('model_warmup.json',dict(status='PASS' if ready else 'FAIL',samples=rows,
        existing_action_age_limit_s=max_action_age_s,existing_settle_s=settle_s,
        physical_commands=0,actual_dispatch_gate_still_required=True))
    if not ready:raise RuntimeError('model_warmup_insufficient_action_age_budget')


def archive_request_jpeg(path,jpeg):
    """Persist the already encoded model input before dispatch; no recompression."""
    try:
        with path.open('xb') as f:
            f.write(jpeg);f.flush();os.fsync(f.fileno())
    except OSError as exc:
        raise RuntimeError('logger_failure:model_input_archive:'+str(exc)) from exc


def deferred_artifacts(output,step,msg,rgb,jpeg):
    """Only called after Controller.step returns/raises, including rejection."""
    start=time.monotonic()
    rgb.save(output/f'camera_{step:03d}.png')
    return dict(source_image_sha256=hashlib.sha256(bytes(msg.data)).hexdigest(),
        model_input_sha256=hashlib.sha256(jpeg).hexdigest(),
        deferred_artifact_duration_s=time.monotonic()-start)
