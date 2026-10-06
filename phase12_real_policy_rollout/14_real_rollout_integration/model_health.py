"""Strict declarative health contract. Missing fields are failures, not defaults."""
def validate_health(actual,expected):
    errors=[]
    for key in ('checkpoint','variant','action_dim','chunk_size','requires_proprio','center_crop',
                'input_resolution','color_order','dtype','normalization','instruction'):
        if key not in expected:errors.append('contract_missing:'+key)
        elif actual.get(key)!=expected[key]:errors.append('health_mismatch:'+key)
    if errors:raise ValueError('|'.join(errors))
    return True

def validate_runtime_health(actual,expected):
    """Compare a versioned, audited contract, including actual processor config."""
    for key in ('server_version','model','checkpoint','variant','action_dim','chunk_size',
                'requires_proprio','center_crop','preprocessing'):
        if key not in expected or key not in actual or actual[key]!=expected[key]:
            raise ValueError('health_contract_mismatch:'+key)
    p=actual['preprocessing']
    for key in ('color_order','source_dtype','tensor_dtype','processor','instruction_handling'):
        if key not in p:raise ValueError('preprocessing_missing:'+key)
    processor=p['processor']
    if not isinstance(processor,dict) or not processor:raise ValueError('processor_config_missing')
    return True
