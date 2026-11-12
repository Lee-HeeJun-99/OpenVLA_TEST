import json
from pathlib import Path

CONFIG_PATH=Path(__file__).with_name("runtime_config.json")


def load_runtime_config(path=CONFIG_PATH):
    return json.loads(Path(path).read_text())
