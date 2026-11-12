"""Command-free Cartesian workspace candidate and validation helpers."""
from dataclasses import dataclass
import math
from typing import Sequence
from pathlib import Path
import yaml

@dataclass(frozen=True)
class WorkspaceDecision:
    accepted: bool
    reason: str|None
    command_issued: bool=False

@dataclass(frozen=True)
class CartesianWorkspace:
    minimum_m: tuple[float,float,float]
    maximum_m: tuple[float,float,float]
    provenance: str="DATA_DERIVED_AND_OPERATOR_APPROVED_2026_10_02"
    def __post_init__(self):
        if len(self.minimum_m)!=3 or len(self.maximum_m)!=3:raise ValueError('workspace_requires_xyz')
        if not all(math.isfinite(x) for x in self.minimum_m+self.maximum_m):raise ValueError('workspace_nonfinite')
        if any(lo>=hi for lo,hi in zip(self.minimum_m,self.maximum_m)):raise ValueError('workspace_invalid_order')
    def inspect(self, position_m:Sequence[float])->WorkspaceDecision:
        p=tuple(float(x) for x in position_m)
        if len(p)!=3 or not all(math.isfinite(x) for x in p):return WorkspaceDecision(False,'invalid_tcp_position')
        if any(x<lo or x>hi for x,lo,hi in zip(p,self.minimum_m,self.maximum_m)):
            return WorkspaceDecision(False,'workspace_violation')
        return WorkspaceDecision(True,None)
    def inspect_delta(self,current_m:Sequence[float],delta_m:Sequence[float])->WorkspaceDecision:
        if len(current_m)!=3 or len(delta_m)!=3:return WorkspaceDecision(False,'invalid_tcp_position')
        return self.inspect([float(a)+float(b) for a,b in zip(current_m,delta_m)])

# Legacy public name retained; bounds and provenance come from the same
# configuration used by the periodic Cartesian tracker.
_config=yaml.safe_load((Path(__file__).resolve().parents[1]/'01_configs/safety_limits.yaml').read_text())
_bounds=_config['workspace_m']
PHASE12_DATA_DERIVED_WORKSPACE=CartesianWorkspace(
    minimum_m=tuple(float(_bounds[axis][0]) for axis in ('x','y','z')),
    maximum_m=tuple(float(_bounds[axis][1]) for axis in ('x','y','z')),
    provenance=_config['workspace_provenance'])
