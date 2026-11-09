"""Validate MoveLine transport bounds; translation and rotation are distinct."""
import math


def validate_profile(velocity, acceleration):
    result=[]
    for values in (velocity,acceleration):
        pair=tuple(float(v) for v in values)
        if len(pair)!=2 or not all(math.isfinite(v) and 0<v<=cap for v,cap in zip(pair,(120.,20.))):
            raise ValueError('velocity_acceleration_contract')
        result.append(pair)
    return tuple(result)
