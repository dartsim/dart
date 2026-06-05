"""Public entry point for the dartpy nanobind module."""

from __future__ import annotations

import os
import sys
import importlib.machinery
from pathlib import Path
from typing import List

_PACKAGE_DIR = Path(__file__).resolve().parent


def _candidate_package_dirs() -> List[str]:
  """Return candidate directories that may contain the compiled extension."""
  roots = []
  for entry in os.environ.get("PYTHONPATH", "").split(os.pathsep):
    if not entry:
      continue
    candidate = (Path(entry).resolve() / "dartpy")
    if candidate == _PACKAGE_DIR or not candidate.is_dir():
      continue
    for suffix in importlib.machinery.EXTENSION_SUFFIXES:
      if any(candidate.glob(f"_dartpy*{suffix}")):
        roots.append(str(candidate))
        break
  roots.append(str(_PACKAGE_DIR))
  return roots


__path__ = _candidate_package_dirs()  # type: ignore[var-annotated]


from . import _dartpy as _ext  # type: ignore[attr-defined]
__version__ = getattr(_ext, "__version__", None)
from . import _layout, _naming


def _alias_extension_submodules() -> None:
  """Alias `_dartpy.*` modules to `dartpy.*` for import compatibility."""
  prefix = _ext.__name__
  package = __name__
  public_prefix = (
      prefix[: -len("._dartpy")] if prefix.endswith("._dartpy") else package
  )
  modules = sys.modules
  for internal_name, module in list(modules.items()):
    if module is None or not isinstance(internal_name, str):
      continue
    if internal_name == prefix:
      continue
    if not internal_name.startswith(f"{prefix}."):
      continue
    alias = f"{public_prefix}{internal_name[len(prefix):]}"
    if alias == package or alias in modules:
      continue
    modules[alias] = module


from ._dartpy import *  # noqa: F401,F403

_alias_extension_submodules()
_naming.install_aliases(_ext)
_layout.install_layout(sys.modules[__name__])


def _install_simulation_diff() -> None:
  """Attach the pure-Python ``diff`` (PyTorch bridge) onto ``dartpy.simulation``.

  ``dartpy.simulation`` is a C++ extension submodule (the official ECS-backed
  simulation API), so its PyTorch autograd bridge lives in a pure-Python module
  attached here as the ``diff`` attribute. This import is torch-free;
  ``dartpy.simulation.diff.timestep`` imports torch lazily, so ``import
  dartpy.simulation`` succeeds without torch and ``diff`` always exists.
  """
  # Use the RAW C++ simulation module, not the deprecated dartpy.simulation
  # legacy wrapper installed by install_layout above -- attribute access on the
  # wrapper emits a DeprecationWarning, which would fire at import time (and fail
  # under -W error).
  experimental = getattr(_ext, "simulation", None)
  if experimental is None:
    return
  from . import _simulation_experimental_diff as _diff_impl

  module_name = f"{__name__}.simulation.diff"
  diff_module = sys.modules.get(module_name)
  if diff_module is None:
    import types

    diff_module = types.ModuleType(module_name)
    diff_module.__doc__ = _diff_impl.__doc__
    diff_module.__package__ = f"{__name__}.simulation"
    diff_module.timestep = _diff_impl.timestep
    diff_module.__all__ = list(_diff_impl.__all__)
    sys.modules[module_name] = diff_module
  # Re-export the framework-neutral C++ rollout (PLAN-110 rollout item) onto the
  # ``diff`` namespace when differentiable support is compiled. The torch
  # ``timestep`` chaining bridge above stays as-is; ``rollout`` is the torch-free
  # path. Present only in DART_BUILD_DIFF=ON builds.
  for _name in ("rollout", "RolloutTrajectory"):
    _obj = getattr(experimental, _name, None)
    if _obj is not None:
      setattr(diff_module, _name, _obj)
      if _name not in diff_module.__all__:
        diff_module.__all__.append(_name)
  experimental.diff = diff_module
  # Expose the diff bridge on the canonical flat namespace too: dartpy.diff
  # (dartpy.simulation.diff still resolves via the legacy wrapper, deprecated).
  setattr(sys.modules[__name__], "diff", diff_module)


_install_simulation_diff()


def _install_world_render_bridge() -> None:
  """Attach the pure-Python ``WorldRenderBridge`` helper onto ``dartpy.gui``.

  ``WorldRenderBridge`` mirrors an ECS ``dartpy.simulation`` World into a
  parallel classic render World that the C++ Filament viewer draws (the ECS
  World has no direct viewer path). It is a supported GUI helper; because
  ``dartpy.gui`` is a C++ extension submodule, the pure-Python class is attached
  here. Requires the GUI build; skipped when ``dartpy.gui`` is absent.
  """
  gui = sys.modules.get(f"{__name__}.gui")
  if gui is None:
    return
  from . import _world_render_bridge as _bridge_impl

  gui.WorldRenderBridge = _bridge_impl.WorldRenderBridge
  # Render plumbing: the classic World root frame, used as the reference frame
  # for SimpleFrame visuals placed in a dartpy.gui.RenderWorld. The DART 7 ECS
  # Frame owns the flat dartpy.Frame name, so render code reaches the legacy
  # render frame here instead. Read from the RAW dynamics module (the
  # dartpy.dynamics legacy wrapper would emit a DeprecationWarning).
  _dyn = getattr(_ext, "dynamics", None)
  if _dyn is not None and hasattr(_dyn, "Frame"):
    gui.world_render_frame = _dyn.Frame.world


_install_world_render_bridge()


def __getattr__(name: str):
  """Expose pure-Python submodules (e.g., gui) via attribute access."""
  import importlib
  try:
    module = importlib.import_module(f"{__name__}.{name}")
  except ModuleNotFoundError as exc:  # pragma: no cover - passthrough
    raise AttributeError(name) from exc
  globals()[name] = module
  _naming.install_aliases(module)
  return module
