"""Production raw-action rejection limits; these never clip or scale."""
import math
from pathlib import Path
import yaml

RAW_TRANSLATION_STEP_LIMIT_M = float(yaml.safe_load(
    (Path(__file__).resolve().parents[1]/'01_configs/safety_limits.yaml').read_text())['translation_step_m'])
if not math.isfinite(RAW_TRANSLATION_STEP_LIMIT_M) or RAW_TRANSLATION_STEP_LIMIT_M<=0:
    raise ValueError('invalid_raw_translation_limit_configuration')


def raw_translation_exceeds_limit(translation):
    values=tuple(float(v) for v in translation)
    if len(values)!=3 or not all(math.isfinite(v) for v in values):
        raise ValueError('invalid_translation')
    return math.sqrt(sum(v*v for v in values)) > RAW_TRANSLATION_STEP_LIMIT_M+1e-12
