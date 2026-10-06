"""Offline-only candidate composition; not authorized for controller commands."""
from __future__ import annotations

def require_verified_convention(status):
    if status!='VERIFIED': raise RuntimeError('UNRESOLVED_ROTATION_CONVENTION')

def compose_candidate(current_matrix, delta_matrix, *, frame, status='UNRESOLVED'):
    require_verified_convention(status)
    if frame=='tool': return current_matrix @ delta_matrix
    if frame=='base': return delta_matrix @ current_matrix
    raise ValueError('frame must be tool or base')
