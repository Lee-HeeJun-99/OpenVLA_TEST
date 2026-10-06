"""Prediction-only health verification. No ROS imports or robot capability."""
import argparse,base64,io,json,math,time
from pathlib import Path
from urllib.request import Request,urlopen
import yaml
from PIL import Image
from model_health import validate_runtime_health
ROOT=Path(__file__).resolve().parent

def validate_identity(health,model):
    config=yaml.safe_load((ROOT/'configs'/f'real_{model}.yaml').read_text())
    if config['checkpoint'] not in str(health.get('checkpoint','')):raise ValueError('checkpoint_mismatch')
    required=dict(model=model,variant=config['variant'],action_dim=7,chunk_size=config['chunk_size'],
        requires_proprio=False,center_crop=model=='oft',server_version='phase12-health-v1')
    for key,value in required.items():
        if health.get(key)!=value:raise ValueError('model_contract:'+key)
    validate_runtime_health(health,health)
    p=health['preprocessing']
    processor_path=ROOT.parents[2]/('models/vanilla_s1_balanced_step8130/processor/preprocessor_config.json' if model=='openvla'
        else 'runtime_state/oft_mixed480_step28560_merged/preprocessor_config.json')
    training_processor=json.loads(processor_path.read_text())
    for key in ('input_sizes','means','stds','interpolations','image_resize_strategy'):
        if p['processor'].get(key)!=training_processor.get(key):raise ValueError('training_preprocessing_mismatch:'+key)
    instruction='clean_instruction_then_build_inference_prompt' if model=='openvla' else 'lower_whitespace_strip_terminal_punctuation'
    if p['instruction_handling']!=instruction:raise ValueError('instruction_handling_mismatch')
    if p['color_order']!='RGB' or p['source_dtype']!='uint8' or p['tensor_dtype']!='bfloat16':raise ValueError('pixel_contract')
    if model=='openvla' and p.get('crop_bottom_fraction')!=0.:raise ValueError('crop_bottom_mismatch')
    if model=='oft' and (p.get('center_crop') is not True or p.get('center_crop_area_scale')!=.9):raise ValueError('center_crop_mismatch')
    return config

def verify(model,url,expected,image):
    started=time.monotonic()
    with urlopen(url+'/health',timeout=3) as response:health=json.load(response)
    config=validate_identity(health,model)
    validate_runtime_health(health,expected or health)
    rgb=Image.open(image).convert('RGB');buffer=io.BytesIO();rgb.save(buffer,format='JPEG')
    req=Request(url+'/predict',data=json.dumps({'image_jpeg_base64':base64.b64encode(buffer.getvalue()).decode(),
        'instruction':config['instruction']}).encode(),headers={'Content-Type':'application/json'})
    prediction_started=time.monotonic()
    with urlopen(req,timeout=10) as response:prediction=json.load(response)
    if prediction.get('fixture') is True:raise ValueError('fixture_not_gpu_runtime_validation')
    actions=prediction.get('actions') if model=='oft' else [prediction.get('action')]
    if not isinstance(actions,list) or len(actions)!=config['chunk_size']:raise ValueError('prediction_chunk_shape')
    for action in actions:
        if not isinstance(action,list) or len(action)!=7 or not all(isinstance(v,(float,int)) and math.isfinite(v) for v in action):raise ValueError('prediction_action_shape')
    return dict(status='MODEL_HEALTH_PASS',health=health,prediction_latency_s=time.monotonic()-prediction_started,
        total_latency_s=time.monotonic()-started,executed_action=None,robot_delivered_command=None)

def main():
    p=argparse.ArgumentParser();p.add_argument('--model',choices=['openvla','oft'],required=True)
    p.add_argument('--url');p.add_argument('--expected-contract',type=Path)
    p.add_argument('--image',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();config=yaml.safe_load((ROOT/'configs'/f'real_{a.model}.yaml').read_text())
    result=verify(a.model,a.url or config['server_url'],json.loads(a.expected_contract.read_text()) if a.expected_contract else None,a.image)
    with a.output.open('x') as f:json.dump(result,f,indent=2)
    print(result['status'])
if __name__=='__main__':main()
