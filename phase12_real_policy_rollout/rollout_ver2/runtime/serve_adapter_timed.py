#!/usr/bin/env python3
"""Timing-enabled OpenVLA server kept inside rollout_ver2.

The bundle's original serve_adapter.py is read-only.  This entry point imports
its model/runtime definitions and adds response metadata without modifying the
action array or its serialization.
"""
from __future__ import annotations
import argparse,base64,importlib.util,io,json,sys,threading,time
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
from pathlib import Path
import numpy as np

ROLLOUT_ROOT=Path(__file__).resolve().parents[1]
if str(ROLLOUT_ROOT) not in sys.path:sys.path.insert(0,str(ROLLOUT_ROOT))
from runtime.server_timing import serialize_action_unchanged

BUNDLE_ROOT=Path(__file__).resolve().parents[4]
SOURCE=BUNDLE_ROOT/"runtime/vanilla-support/serve_adapter.py"
SOURCE_DIR=SOURCE.parent
for dependency in (SOURCE_DIR,BUNDLE_ROOT/"runtime/openvla-oft"):
    if str(dependency) not in sys.path:sys.path.insert(0,str(dependency))
spec=importlib.util.spec_from_file_location("a0509_original_serve_adapter",SOURCE)
original=importlib.util.module_from_spec(spec);spec.loader.exec_module(original)
torch=original.torch;Image=original.Image


class TimedAdapterRuntime(original.AdapterRuntime):
    def predict(self,image,instruction,decode_s=0.0):
        preprocess_started=time.perf_counter()
        clean=original.clean_instruction(instruction)
        image=original.crop_image_bottom(image.convert("RGB"),self.crop_bottom_fraction)
        inputs=self.processor(original.build_inference_prompt(clean),image).to(
          "cuda:0",dtype=torch.bfloat16)
        preprocess_s=time.perf_counter()-preprocess_started
        with self.lock,torch.inference_mode():
            torch.cuda.synchronize();torch.cuda.reset_peak_memory_stats()
            forward_started=time.perf_counter()
            action=self.predictor.predict_action(**inputs,unnorm_key=original.DATASET_KEY,
              do_sample=False)
            torch.cuda.synchronize();forward_s=time.perf_counter()-forward_started
            self.request_count+=1
            allocated=torch.cuda.memory_allocated()/1024**2
            reserved=torch.cuda.memory_reserved()/1024**2
            peak=torch.cuda.max_memory_allocated()/1024**2
        detokenize_started=time.perf_counter()
        if isinstance(action,tuple):action=action[0]
        serialized_action=serialize_action_unchanged(action)
        detokenize_s=time.perf_counter()-detokenize_started
        return {"action":serialized_action,"chunk_size":1,"variant":"openvla_token",
          "instruction":clean,"inference_seconds":forward_s,"request_index":self.request_count,
          "model":"openvla","checkpoint":str(self.bundle),
          "server_timing":{"decode_s":float(decode_s),"preprocess_s":preprocess_s,
            "forward_s":forward_s,"detokenize_s":detokenize_s,
            "vram_allocated_mb":allocated,"vram_reserved_mb":reserved,"vram_peak_mb":peak}}


def make_handler(runtime):
    class Handler(BaseHTTPRequestHandler):
        server_version="A0509OpenVLATimed/1.0"
        def write_json(self,status,payload):
            encoded=json.dumps(payload,separators=(",", ":")).encode()
            self.send_response(status);self.send_header("Content-Type","application/json")
            self.send_header("Content-Length",str(len(encoded)));self.end_headers();self.wfile.write(encoded)
        def do_GET(self):
            self.write_json(200,runtime.status()) if self.path=="/health" else self.write_json(404,{"error":"not_found"})
        def do_POST(self):
            if self.path!="/predict":self.write_json(404,{"error":"not_found"});return
            try:
                decode_started=time.perf_counter();length=int(self.headers.get("Content-Length","0"))
                if length<=0 or length>5*1024*1024:raise ValueError("request body must be between 1 byte and 5 MiB")
                body=json.loads(self.rfile.read(length));data=base64.b64decode(body["image_jpeg_base64"],validate=True)
                with Image.open(io.BytesIO(data)) as source:image=source.convert("RGB")
                decode_s=time.perf_counter()-decode_started
                self.write_json(200,runtime.predict(image,str(body["instruction"]),decode_s))
            except Exception as error:self.write_json(400,{"error":type(error).__name__,"message":str(error)})
        def log_message(self,fmt,*args):print("A0509_VLA_HTTP "+(fmt%args),flush=True)
    return Handler


def main():
    p=argparse.ArgumentParser();p.add_argument("--bundle",type=Path,required=True)
    p.add_argument("--model-id",default="openvla/openvla-7b");p.add_argument("--host",default="127.0.0.1")
    p.add_argument("--port",type=int,default=8766);p.add_argument("--local-files-only",action="store_true")
    a=p.parse_args();runtime=TimedAdapterRuntime(a.bundle,a.model_id,a.local_files_only)
    server=ThreadingHTTPServer((a.host,a.port),make_handler(runtime))
    print("A0509_VLA_SERVER_READY="+json.dumps({**runtime.status(),"timing_enabled":True,
      "url":f"http://{a.host}:{a.port}"}),flush=True)
    try:server.serve_forever(.2)
    finally:server.server_close()

if __name__=="__main__":main()
