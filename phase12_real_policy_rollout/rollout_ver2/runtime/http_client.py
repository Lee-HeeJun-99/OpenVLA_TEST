import base64,json
from urllib.request import Request,urlopen


class PredictClient:
    def __init__(self,url="http://127.0.0.1:8766",timeout_s=30.0):
        self.url=url.rstrip("/");self.timeout_s=float(timeout_s)
    def predict_jpeg(self,jpeg,instruction):
        body=json.dumps({"image_jpeg_base64":base64.b64encode(jpeg).decode("ascii"),
                         "instruction":instruction}).encode()
        request=Request(self.url+"/predict",data=body,method="POST",
                        headers={"Content-Type":"application/json"})
        with urlopen(request,timeout=self.timeout_s) as response:return json.load(response)
