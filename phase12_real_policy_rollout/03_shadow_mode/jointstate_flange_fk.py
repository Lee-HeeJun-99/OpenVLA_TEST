#!/usr/bin/env python3
"""Command-free A0509 URDF FK from measured JointState positions.

The result is a computed link_6/flange pose, not controller measured TCP feedback.
No ROS publisher, service client, action client, or robot command exists here.
"""
from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Mapping, Sequence


CANONICAL_JOINTS = tuple(f"joint_{i}" for i in range(1, 7))


def _eye4():
    return [[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]]


def _mm(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def _origin(xyz, rpy):
    x, y, z = xyz; r, p, yw = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(yw), math.sin(yw)
    # URDF fixed-axis roll-pitch-yaw: Rz(yaw) Ry(pitch) Rx(roll).
    t = _eye4()
    t[0][:3] = [cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr]
    t[1][:3] = [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr]
    t[2][:3] = [-sp, cp*sr, cp*cr]
    t[0][3], t[1][3], t[2][3] = x, y, z
    return t


def _axis_angle(axis, angle):
    x, y, z = axis
    n = math.sqrt(x*x+y*y+z*z)
    if n == 0.0: raise ValueError("zero_joint_axis")
    x, y, z = x/n, y/n, z/n
    c, s, v = math.cos(angle), math.sin(angle), 1.0-math.cos(angle)
    t = _eye4()
    t[0][:3] = [x*x*v+c, x*y*v-z*s, x*z*v+y*s]
    t[1][:3] = [y*x*v+z*s, y*y*v+c, y*z*v-x*s]
    t[2][:3] = [z*x*v-y*s, z*y*v+x*s, z*z*v+c]
    return t


def _quat_wxyz(r):
    # Stable matrix-to-quaternion conversion.
    tr = r[0][0] + r[1][1] + r[2][2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2; w=.25*s; x=(r[2][1]-r[1][2])/s; y=(r[0][2]-r[2][0])/s; z=(r[1][0]-r[0][1])/s
    elif r[0][0] > r[1][1] and r[0][0] > r[2][2]:
        s=math.sqrt(1+r[0][0]-r[1][1]-r[2][2])*2; w=(r[2][1]-r[1][2])/s; x=.25*s; y=(r[0][1]+r[1][0])/s; z=(r[0][2]+r[2][0])/s
    elif r[1][1] > r[2][2]:
        s=math.sqrt(1+r[1][1]-r[0][0]-r[2][2])*2; w=(r[0][2]-r[2][0])/s; x=(r[0][1]+r[1][0])/s; y=.25*s; z=(r[1][2]+r[2][1])/s
    else:
        s=math.sqrt(1+r[2][2]-r[0][0]-r[1][1])*2; w=(r[1][0]-r[0][1])/s; x=(r[0][2]+r[2][0])/s; y=(r[1][2]+r[2][1])/s; z=.25*s
    n=math.sqrt(w*w+x*x+y*y+z*z)
    return [w/n,x/n,y/n,z/n]


@dataclass(frozen=True)
class Joint:
    name: str
    parent: str
    child: str
    xyz: tuple[float, float, float]
    rpy: tuple[float, float, float]
    axis: tuple[float, float, float]


class A0509FlangeFK:
    def __init__(self, urdf_path: str | Path):
        self.urdf_path = str(Path(urdf_path).resolve())
        root = ET.parse(self.urdf_path).getroot()
        by_name = {j.attrib['name']: j for j in root.findall('joint')}
        chain=[]
        for name in CANONICAL_JOINTS:
            e=by_name.get(name)
            if e is None or e.attrib.get('type') not in {'revolute','continuous'}: raise ValueError(f"invalid_or_missing_{name}")
            origin=e.find('origin'); axis=e.find('axis')
            xyz=tuple(float(v) for v in (origin.attrib.get('xyz','0 0 0') if origin is not None else '0 0 0').split())
            rpy=tuple(float(v) for v in (origin.attrib.get('rpy','0 0 0') if origin is not None else '0 0 0').split())
            av=tuple(float(v) for v in (axis.attrib.get('xyz','0 0 1') if axis is not None else '0 0 1').split())
            chain.append(Joint(name,e.find('parent').attrib['link'],e.find('child').attrib['link'],xyz,rpy,av))
        for a,b in zip(chain,chain[1:]):
            if a.child != b.parent: raise ValueError(f"disconnected_chain_{a.name}_{b.name}")
        if chain[0].parent != 'base_link' or chain[-1].child != 'link_6': raise ValueError('unexpected_a0509_chain_endpoints')
        self.chain=tuple(chain)

    def compute(self, positions_by_name: Mapping[str, float]):
        if set(CANONICAL_JOINTS)-set(positions_by_name): raise ValueError('missing_canonical_joint')
        q=[float(positions_by_name[n]) for n in CANONICAL_JOINTS]
        if not all(math.isfinite(v) for v in q): raise ValueError('nonfinite_joint_position')
        t=_eye4()
        for joint,angle in zip(self.chain,q):
            t=_mm(t,_origin(joint.xyz,joint.rpy)); t=_mm(t,_axis_angle(joint.axis,angle))
        rotation=[row[:3] for row in t[:3]]
        return {
            'source':'computed_from_measured_joint_state_urdf_fk',
            'pose_kind':'computed_link6_flange_not_measured_tcp',
            'reference_frame':'base_link',
            'child_frame':'link_6',
            'position_m':[t[0][3],t[1][3],t[2][3]],
            'rotation_matrix':rotation,
            'quaternion_wxyz':_quat_wxyz(rotation),
            'joint_order':list(CANONICAL_JOINTS),
            'joint_position_rad':q,
            'urdf_path':self.urdf_path,
            'tool_offset_applied':False,
            'measured_tcp_available':False,
            'valid_for_observation':True,
            'valid_as_measured_tcp':False,
            'valid_for_motion_readiness':False,
            'blocker':'UNVERIFIED_FLANGE_TO_TCP_OFFSET_AND_NO_MEASURED_TCP_CROSSCHECK',
            'command_issued':False,
            'executed_action':None,
            'robot_delivered_command':None,
        }


def reorder_joint_state(names: Sequence[str], positions: Sequence[float]):
    if len(names) != len(positions) or len(set(names)) != len(names): raise ValueError('invalid_joint_arrays')
    values=dict(zip(names,positions))
    return {name:values[name] for name in CANONICAL_JOINTS if name in values}
