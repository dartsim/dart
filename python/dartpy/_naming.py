"""Runtime helpers to present a snake_case-friendly dartpy API."""

from __future__ import annotations

import functools
import inspect
import os
import re
import types
import warnings
from typing import Callable, Iterable

_SNAKE_ENV = "DARTPY_ENABLE_SNAKE_CASE"
_WARN_ENV = "DARTPY_WARN_ON_CAMELCASE"

ENABLE_SNAKE_CASE_ALIASES = os.environ.get(_SNAKE_ENV, "1").lower() not in (
    "0",
    "false",
    "no",
)
WARN_ON_CAMELCASE = os.environ.get(_WARN_ENV, "1").lower() not in (
    "0",
    "false",
    "no",
)

_WARNED: set[str] = set()


def camel_to_snake(name: str) -> str:
  """Convert CamelCase/camelCase to snake_case."""
  step1 = re.sub(r"([a-z0-9])([A-Z])", r"\1_\2", name)
  step2 = re.sub(r"([A-Z]+)([A-Z][a-z])", r"\1_\2", step1)
  return step2.replace("__", "_").lower()


def _should_alias(name: str) -> bool:
  if not name or name.startswith("_") or "__" in name:
    return False
  if "_" in name or name.isupper():
    return False
  return any(ch.isupper() for ch in name)


def _warn_once(symbol: str, replacement: str) -> None:
  key = f"{symbol}->{replacement}"
  if key in _WARNED:
    return
  _WARNED.add(key)
  warnings.warn(
      f"dartpy camelCase name `{symbol}` is deprecated; use `{replacement}`",
      DeprecationWarning,
      stacklevel=3,
  )


def _qualified_symbol(target: object, name: str) -> str:
  if inspect.isclass(target):
    module = getattr(target, "__module__", None)
    prefix = f"{module}." if module else ""
    return f"{prefix}{getattr(target, '__name__', target)}.{name}"
  if isinstance(target, types.ModuleType):
    module = getattr(target, "__name__", None)
    prefix = f"{module}." if module else ""
    return f"{prefix}{name}"
  return name


def _wrap_callable(fn: Callable, symbol: str, replacement: str) -> Callable:
  @functools.wraps(fn)
  def wrapper(*args, **kwargs):
    _warn_once(symbol, replacement)
    return fn(*args, **kwargs)

  return wrapper


def _wrap_property(prop: property, symbol: str, replacement: str) -> property:
  def _wrap_getter(getter: Callable | None):
    if getter is None:
      return None
    return _wrap_callable(getter, symbol, replacement)

  return property(
      fget=_wrap_getter(prop.fget),
      fset=_wrap_getter(prop.fset),
      fdel=_wrap_getter(prop.fdel),
      doc=prop.__doc__,
  )


def _alias_member(
    target: object, name: str, member: object, warn_on_camel: bool
) -> None:
  snake_name = camel_to_snake(name)
  if snake_name == name or hasattr(target, snake_name):
    return
  if isinstance(member, property):
    try:
      setattr(target, snake_name, member)
      if warn_on_camel:
        setattr(
            target,
            name,
            _wrap_property(
                member,
                _qualified_symbol(target, name),
                _qualified_symbol(target, snake_name),
            ),
        )
    except Exception:
      return
    return
  if inspect.isclass(member) or inspect.ismodule(member):
    return
  if not (inspect.isroutine(member) or callable(member)):
    return
  try:
    setattr(target, snake_name, member)
    if warn_on_camel:
      setattr(
          target,
          name,
          _wrap_callable(
              member,
              _qualified_symbol(target, name),
              _qualified_symbol(target, snake_name),
          ),
      )
  except Exception:
    return


def _iter_members(target: object) -> Iterable[tuple[str, object]]:
  for name in dir(target):
    if not _should_alias(name):
      continue
    try:
      member = getattr(target, name)
    except Exception:
      continue
    yield name, member


def _walk(target: object, warn_on_camel: bool, visited: set[int]) -> None:
  target_id = id(target)
  if target_id in visited:
    return
  visited.add(target_id)

  if isinstance(target, types.ModuleType):
    for name, member in _iter_members(target):
      _alias_member(target, name, member, warn_on_camel)
    for _, submodule in inspect.getmembers(target, inspect.ismodule):
      if getattr(submodule, "__name__", "").startswith(("_dartpy", "dartpy")):
        _walk(submodule, warn_on_camel, visited)
    for _, cls in inspect.getmembers(target, inspect.isclass):
      if getattr(cls, "__module__", "").startswith(("_dartpy", "dartpy")):
        _walk(cls, warn_on_camel, visited)
    return

  if inspect.isclass(target):
    for name, member in _iter_members(target):
      _alias_member(target, name, member, warn_on_camel)
    for _, cls in inspect.getmembers(target, inspect.isclass):
      if getattr(cls, "__module__", "").startswith(("_dartpy", "dartpy")):
        _walk(cls, warn_on_camel, visited)


def install_aliases(root: object, *, warn_on_camel: bool | None = None) -> None:
  """Attach snake_case aliases across dartpy modules and classes."""
  if not ENABLE_SNAKE_CASE_ALIASES:
    return
  _walk(root, WARN_ON_CAMELCASE if warn_on_camel is None else warn_on_camel, set())
