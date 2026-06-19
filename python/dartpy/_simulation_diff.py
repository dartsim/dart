"""PyTorch autograd bridge for differentiable experimental simulation.

This module is the pure-Python ``diff`` namespace attached to
``dartpy`` (imported as ``sx``). It wraps a single
differentiable ``World`` step as a ``torch.autograd.Function`` so a step can be
composed into a PyTorch graph:

>>> import dartpy as sx
>>> next_state = sx.diff.timestep(world, state, action)  # doctest: +SKIP
>>> next_state.sum().backward()                            # doctest: +SKIP

Torch is an optional dependency. Importing this module (and therefore
``import dartpy``) must succeed without torch installed;
``sx.diff`` always exists as an attribute. ``torch`` is imported lazily inside
``timestep`` so the import-only path stays torch-free. Calling ``timestep``
without torch raises a clear :class:`ImportError`.

The forward pass writes ``state`` and ``action`` onto the world, steps once, and
returns the next state ``x' = [q'; q̇']``. The backward pass uses the world's
reverse-mode primitive ``World.apply_step_vjp`` to pull the upstream gradient
``dL/dx'`` back to gradients with respect to ``state`` and ``action``.
"""

from __future__ import annotations

__all__ = ["timestep"]

_TORCH_REQUIRED_MESSAGE = "torch is required for sx.diff; pip install torch"


def _require_torch():
  """Import torch lazily, raising a clear error when it is unavailable."""
  try:
    import torch  # noqa: PLC0415
  except ImportError as exc:  # pragma: no cover - exercised without torch
    raise ImportError(_TORCH_REQUIRED_MESSAGE) from exc
  return torch


def _make_step_function(torch):
  """Build the ``torch.autograd.Function`` for one differentiable step.

  The function is constructed lazily (only when torch is available) so this
  module imports cleanly without torch. The world is carried through the
  context so the backward pass can call ``apply_step_vjp`` on it.
  """

  class _TimestepFunction(torch.autograd.Function):
    @staticmethod
    def forward(ctx, world, state, action):
      ctx.world = world
      state_np = state.detach().cpu().double().numpy().reshape(-1)
      action_np = action.detach().cpu().double().numpy().reshape(-1)
      world.state_vector = state_np
      world.control_vector = action_np
      world.step()
      next_state_np = world.state_vector
      return torch.as_tensor(
          next_state_np, dtype=state.dtype, device=state.device
      )

    @staticmethod
    def backward(ctx, grad_next_state):
      grad_np = grad_next_state.detach().cpu().double().numpy().reshape(-1)
      gradient = ctx.world.apply_step_vjp(grad_np)
      grad_state = torch.as_tensor(
          gradient.state,
          dtype=grad_next_state.dtype,
          device=grad_next_state.device,
      )
      grad_action = torch.as_tensor(
          gradient.control,
          dtype=grad_next_state.dtype,
          device=grad_next_state.device,
      )
      # No gradient flows to the (non-tensor) world argument.
      return None, grad_state, grad_action

  return _TimestepFunction


def timestep(world, state, action):
  """Differentiable single simulation step as a ``torch.autograd.Function``.

  Args:
    world: A differentiable ``dartpy.World`` (constructed
      with ``differentiable=True``). The world's general state/control vector
      layout must match the active differentiable family for that step.
      Rigid-only and single-multibody scenes satisfy this today; mixed
      rigid-body + multibody differentiable rollouts are rejected until
      full-world Jacobians are assembled.
    state: Torch tensor ``x = [q; q̇]`` of size ``2 * world.num_dofs``.
    action: Torch tensor ``u = τ`` of size ``world.num_efforts``.

  Returns:
    A torch tensor ``x' = [q'; q̇']`` connected to the autograd graph; calling
    ``.backward()`` populates ``state.grad`` and ``action.grad`` via the world's
    reverse-mode step VJP.

  Raises:
    ImportError: if torch is not installed.
  """
  torch = _require_torch()
  step_function = _make_step_function(torch)
  return step_function.apply(world, state, action)
