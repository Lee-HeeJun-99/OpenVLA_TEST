import io
import json
import unittest

import numpy as np
import torch
from PIL import Image
from torchvision.transforms import functional as TVF
from torchvision.transforms import InterpolationMode

from controller.observation import (PROCESSOR_CONFIG, preprocess_observation,
                                    processor_tensor)

IMAGE = "/home/ubuntu/a0509_vla_linux_field_bundle_20260903/validation/preflight/input.jpg"


class ObservationContractTests(unittest.TestCase):
    def test_sim_client_and_rollout_jpeg_and_tensor_are_identical(self):
        rgb = np.asarray(Image.open(IMAGE).convert("RGB"), dtype=np.uint8)

        # Independent reference spelling of VLAInferenceClient.predict.
        reference_buffer = io.BytesIO()
        Image.fromarray(rgb, mode="RGB").save(
            reference_buffer, format="JPEG", quality=92, optimize=False)
        reference_jpeg = reference_buffer.getvalue()
        with Image.open(io.BytesIO(reference_jpeg)) as source:
            reference_rgb = source.convert("RGB")

        rollout_jpeg, rollout_tensor = preprocess_observation(rgb)
        self.assertEqual(reference_jpeg, rollout_jpeg)

        config = json.loads(PROCESSOR_CONFIG.read_text())
        refs = []
        for resize, crop, norm in zip(config["tvf_resize_params"],
                                      config["tvf_crop_params"],
                                      config["tvf_normalize_params"]):
            image = TVF.resize(reference_rgb, resize["size"],
                               InterpolationMode.BICUBIC,
                               antialias=resize["antialias"])
            image = TVF.center_crop(image, crop["output_size"])
            refs.append(TVF.normalize(TVF.to_tensor(image), norm["mean"],
                                      norm["std"], inplace=False))
        reference_tensor = torch.vstack(refs)
        self.assertTrue(torch.equal(reference_tensor, rollout_tensor))
        self.assertEqual(tuple(rollout_tensor.shape), (6, 224, 224))

    def test_processor_has_no_geometric_input_center_crop(self):
        config = json.loads(PROCESSOR_CONFIG.read_text())
        self.assertEqual(config["image_resize_strategy"], "resize-naive")
        self.assertEqual(config["tvf_resize_params"][0]["size"], [224, 224])
        self.assertEqual(config["tvf_crop_params"][0]["output_size"], [224, 224])


if __name__ == "__main__":
    unittest.main()
