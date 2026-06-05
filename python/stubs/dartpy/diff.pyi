"""
PyTorch autograd bridge for differentiable simulation.

Attached at runtime as ``dartpy.diff`` (and ``dartpy.simulation.diff``).
Wraps a single differentiable ``World`` step as a
``torch.autograd.Function``; torch is imported lazily so importing
dartpy stays torch-free.
"""

from __future__ import annotations

__all__: list[str] = [
    "timestep",
]

def timestep(world, state, action): ...
