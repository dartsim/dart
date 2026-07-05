"""Runtime helpers to flatten the dartpy namespace and deprecate legacy modules."""

from __future__ import annotations

import importlib
import os
import sys
import types
import warnings
from typing import Iterable

_LEGACY_ENV = "DARTPY_ENABLE_LEGACY_MODULES"
_LEGACY_WARN_ENV = "DARTPY_WARN_ON_LEGACY_MODULES"

ENABLE_LEGACY_MODULES = os.environ.get(_LEGACY_ENV, "1").lower() not in (
    "0",
    "false",
    "no",
)
WARN_ON_LEGACY_MODULES = os.environ.get(_LEGACY_WARN_ENV, "0").lower() not in (
    "0",
    "false",
    "no",
)

# DART 6 submodules retained only as deprecated compatibility wrappers: their
# public symbols are flat-promoted to dartpy.*. Reaching them via the submodule
# path can emit a DeprecationWarning when DARTPY_WARN_ON_LEGACY_MODULES=1.
# `simulation` is deliberately absent here -- it is the official DART 7 ECS
# module with its own generated simulation.pyi stub, so dartpy.simulation.World /
# dartpy.simulation.diff must resolve without a deprecation warning (and not fail
# under `-W error`), even though the same names are also promoted flat as
# dartpy.World etc.
_LEGACY_MODULES: tuple[str, ...] = (
    "common",
    "math",
    "dynamics",
    "collision",
    "constraint",
    "optimizer",
    "utils",
)
# Flatten order matters: _promote_symbols is first-wins. Promote `simulation`
# FIRST so the official DART 7 ECS facade owns the canonical flat names. Three of
# its names collide with the legacy `dynamics` module -- Frame, Joint,
# ActuatorType -- and the ECS versions win: dartpy.Frame is the DART 7 ECS Frame.
# DART 6 dynamics types are not part of the public DART 7 surface; the classic
# Frame survives only as render plumbing, reached by the render bridge via
# dartpy.gui.world_render_frame(), never the flat name.
_PROMOTE_MODULES: tuple[str, ...] = ("simulation",) + tuple(
    name for name in _LEGACY_MODULES if name != "utils"
)

_WARNED: set[str] = set()


def _warn_once(module: str, replacement: str) -> None:
  if not WARN_ON_LEGACY_MODULES:
    return
  key = f"{module}->{replacement}"
  if key in _WARNED:
    return
  _WARNED.add(key)
  warnings.warn(
      f"`{module}` is deprecated and will be removed in DART 8.0; "
      f"use `{replacement}` instead.",
      DeprecationWarning,
      stacklevel=3,
  )


def _load_module(name: str):
  try:
    return importlib.import_module(name)
  except ModuleNotFoundError:
    return None


def _promote_symbols(root, module_names: Iterable[str]) -> None:
  root_name = getattr(root, "__name__", "dartpy")
  for module in module_names:
    mod = _load_module(f"{root_name}.{module}")
    if mod is None:
      continue
    for attr in dir(mod):
      if attr.startswith("_"):
        continue
      if not attr[0].isupper() and any(ch.isupper() for ch in attr):
        continue
      if hasattr(root, attr):
        continue
      try:
        setattr(root, attr, getattr(mod, attr))
      except Exception:
        continue


class _LegacyModule(types.ModuleType):
  def __init__(self, name: str, target, replacement: str):
    super().__init__(name)
    self.__dict__["_target"] = target
    self.__dict__["_replacement"] = replacement
    self.__dict__["__package__"] = name.rpartition(".")[0]
    self.__dict__["__doc__"] = getattr(target, "__doc__", None)
    for metadata in ("__file__", "__spec__", "__loader__", "__cached__"):
      if hasattr(target, metadata):
        self.__dict__[metadata] = getattr(target, metadata)
    if hasattr(target, "__path__"):
      self.__dict__["__path__"] = getattr(target, "__path__")
    if hasattr(target, "__all__"):
      self.__dict__["__all__"] = getattr(target, "__all__")

  def __getattr__(self, item):
    if item.startswith("__") and item.endswith("__"):
      return getattr(self._target, item)
    _warn_once(self.__name__, self._replacement)
    return getattr(self._target, item)

  def __dir__(self):
    return dir(self._target)


def _install_legacy_modules(root) -> None:
  if not ENABLE_LEGACY_MODULES:
    return
  root_name = getattr(root, "__name__", "dartpy")
  for module in _LEGACY_MODULES:
    mod = _load_module(f"{root_name}.{module}")
    if mod is None:
      continue
    replacement = f"{root_name}.io" if module == "utils" else root_name
    wrapper = _LegacyModule(f"{root_name}.{module}", mod, replacement)
    sys.modules[f"{root_name}.{module}"] = wrapper
    setattr(root, module, wrapper)


def _install_io_alias(root) -> None:
  root_name = getattr(root, "__name__", "dartpy")
  utils_mod = _load_module(f"{root_name}.utils")
  if utils_mod is None:
    return
  sys.modules[f"{root_name}.io"] = utils_mod
  setattr(root, "io", utils_mod)


def install_layout(root) -> None:
  """Flatten public API to dartpy + dartpy.io and deprecate legacy modules."""
  _install_io_alias(root)
  _promote_symbols(root, _PROMOTE_MODULES)
  _install_legacy_modules(root)
