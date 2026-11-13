"""Pure helpers shared by the timed server and /usr/bin/python3 tests."""
import numpy as np


def serialize_action_unchanged(action):
    action_array=np.asarray(action,dtype=np.float64)
    while action_array.ndim>1 and action_array.shape[0]==1:action_array=action_array[0]
    if action_array.shape!=(7,):
        raise ValueError(f"Expected one H1 action with shape (7,), got {action_array.shape}")
    return action_array.tolist()
