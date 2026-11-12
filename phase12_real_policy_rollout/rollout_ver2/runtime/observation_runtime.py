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

    def predict(self, rgb, instruction):
        jpeg,_=preprocess_observation(np.asarray(rgb,dtype=np.uint8))
        image_path=self.output/f"model_input_{self.index:06d}.jpg"
        image_path.write_bytes(jpeg)
        response=self.client.predict_jpeg(jpeg,instruction)
        action=self.interpreter.parse(response["action"])
        row={"step":self.index,"received_monotonic":self.clock(),
             "image":str(image_path),"instruction":instruction,
             "raw_action":action.raw.tolist(),
             "arm_endpoint_min":action.endpoint_min[:6].tolist(),
             "arm_endpoint_max":action.endpoint_max[:6].tolist()}
        with (self.output/"predictions.jsonl").open("a") as stream:
            stream.write(json.dumps(row,separators=(",",":"))+"\n")
        self.index+=1; return action

