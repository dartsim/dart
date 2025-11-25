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
WARN_ON_LEGACY_MODULES = os.environ.get(_LEGACY_WARN_ENV, "1").lower() not in (
    "0",
    "false",
    "no",
)

_LEGACY_MODULES: tuple[str, ...] = (
    "common",
    "math",
    "dynamics",
    "collision",
    "simulation",
    "constraint",
    "optimizer",
    "utils",
)
_PROMOTE_MODULES: tuple[str, ...] = tuple(
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
    if hasattr(target, "__path__"):
      self.__dict__["__path__"] = getattr(target, "__path__")
    if hasattr(target, "__all__"):
      self.__dict__["__all__"] = getattr(target, "__all__")

  def __getattr__(self, item):
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
