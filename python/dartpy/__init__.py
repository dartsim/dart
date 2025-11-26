"""Public entry point for the dartpy nanobind module."""

from __future__ import annotations

import os
import sys
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
    for suffix in (".so", ".pyd", ".dylib"):
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
