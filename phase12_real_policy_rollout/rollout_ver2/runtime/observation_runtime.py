from __future__ import annotations
import json
from pathlib import Path
import time
import numpy as np
from controller.observation import preprocess_observation


class ObservationRuntime:
    def __init__(self, client, interpreter, output_dir, clock=time.monotonic):
        self.client=client; self.interpreter=interpreter; self.output=Path(output_dir)
        self.output.mkdir(parents=True,exist_ok=True); self.clock=clock; self.index=0

    def predict(self, rgb, instruction, observation_metadata=None):
        started=self.clock()
        jpeg,_=preprocess_observation(np.asarray(rgb,dtype=np.uint8))
        preprocessed=self.clock()
        image_path=self.output/f"model_input_{self.index:06d}.jpg"
        image_path.write_bytes(jpeg)
        image_written=self.clock()
        response=self.client.predict_jpeg(jpeg,instruction)
        http_completed=self.clock()
        action=self.interpreter.parse(response["action"])
        parsed=self.clock()
        row={"step":self.index,"received_monotonic":self.clock(),
             "image":str(image_path),"instruction":instruction,
             "raw_action":action.raw.tolist(),
             "arm_endpoint_min":action.endpoint_min[:6].tolist(),
             "arm_endpoint_max":action.endpoint_max[:6].tolist()}
        if "server_timing" in response:row["server_timing"]=dict(response["server_timing"])
        if response.get("model") is not None:row["server_model"]=response["model"]
        if response.get("checkpoint") is not None:row["server_checkpoint"]=response["checkpoint"]
        row["processing_timing"]={"started_monotonic":started,
          "preprocess_completed_monotonic":preprocessed,
          "image_write_completed_monotonic":image_written,
          "http_completed_monotonic":http_completed,"parse_completed_monotonic":parsed,
          "preprocess_s":preprocessed-started,"image_write_s":image_written-preprocessed,
          "http_s":http_completed-image_written,"parse_s":parsed-http_completed,
          "total_s":parsed-started}
        if observation_metadata is not None:row["observation_timing"]=dict(observation_metadata)
        with (self.output/"predictions.jsonl").open("a") as stream:
            stream.write(json.dumps(row,separators=(",",":"))+"\n")
        self.last_record=row;self.index+=1; return action
