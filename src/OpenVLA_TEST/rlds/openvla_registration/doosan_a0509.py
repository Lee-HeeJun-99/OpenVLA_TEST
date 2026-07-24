"""OpenVLA registration snippets for the Doosan A0509 RLDS dataset.

Copy the relevant pieces into the matching files in your OpenVLA checkout.
This file is intentionally not imported by the collector package.
"""


CONFIGS_PY_SNIPPET = """
"doosan_a0509": {
    "image_obs_keys": {
        "primary": "image",
        "secondary": None,
        "wrist": None,
    },
    "depth_obs_keys": {
        "primary": None,
        "secondary": None,
        "wrist": None,
    },
    "state_obs_keys": ["EEF_state", None, "gripper_state"],
    "state_encoding": StateEncoding.POS_EULER,
    "action_encoding": ActionEncoding.EEF_POS,
},
"""


TRANSFORMS_PY_SNIPPET = """
def doosan_a0509_dataset_transform(trajectory):
    trajectory["language_instruction"] = trajectory["language_instruction"]
    return trajectory
"""

TRANSFORM_REGISTRY_SNIPPET = """
"doosan_a0509": doosan_a0509_dataset_transform,
"""

MIXTURES_PY_SNIPPET = """
"doosan_a0509_single": [
    ("doosan_a0509", 1.0),
],
"""
