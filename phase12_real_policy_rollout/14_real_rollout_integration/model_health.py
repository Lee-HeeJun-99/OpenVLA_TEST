"""Strict declarative health contract. Missing fields are failures, not defaults."""
def validate_health(actual,expected):
    errors=[]
    for key in ('checkpoint','variant','action_dim','chunk_size','requires_proprio','center_crop',
                'input_resolution','color_order','dtype','normalization','instruction'):
        if key not in expected:errors.append('contract_missing:'+key)
        elif actual.get(key)!=expected[key]:errors.append('health_mismatch:'+key)
    if errors:raise ValueError('|'.join(errors))
    return True
