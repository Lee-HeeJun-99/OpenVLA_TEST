import json

import json_numpy
import numpy as np
import requests
from PIL import Image


json_numpy.patch()

SERVER_URL = "http://127.0.0.1:8000/act"
IMAGE_PATH = "/home/ubuntu/test_image.jpg"

# dataset_statistics.json의 최상위 키와 같아야 합니다.
UNNORM_KEY = "doosan_a0509"

image = Image.open(IMAGE_PATH).convert("RGB")
image = np.asarray(image, dtype=np.uint8)

payload = {
    "image": image,
    "instruction": "pick up the cube",
    "unnorm_key": UNNORM_KEY,
}

print("이미지 크기:", image.shape)
print("OpenVLA 추론 요청 전송 중...")

response = requests.post(
    SERVER_URL,
    json=payload,
    timeout=120,
)

print("HTTP 상태 코드:", response.status_code)

if response.status_code != 200:
    print("서버 응답:")
    print(response.text)
    raise SystemExit(1)

action = response.json()

print("OpenVLA 출력:")
print(action)
print("Action 배열:", np.asarray(action))
print("Action shape:", np.asarray(action).shape)