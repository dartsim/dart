#!/usr/bin/env python3
"""Shared environment sanitization for canonical repository test runners."""

from __future__ import annotations

import os
from collections.abc import Mapping


def sanitized_environment(
    prefixes: tuple[str, ...],
    source: Mapping[str, str] | None = None,
) -> dict[str, str]:
    """Return an environment without case-insensitive control prefixes."""

    environment = dict(os.environ if source is None else source)
    normalized = tuple(prefix.upper() for prefix in prefixes)
    for name in tuple(environment):
        if name.upper().startswith(normalized):
            environment.pop(name)
    return environment


def sanitized_gtest_environment(
    source: Mapping[str, str] | None = None,
) -> dict[str, str]:
    """Remove every current or future GoogleTest environment control."""

    return sanitized_environment(("GTEST_",), source)
