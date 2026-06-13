#!/usr/bin/env python3
"""Select the newest installed Visual Studio CMake generator."""

from __future__ import annotations

import json
import os
import re
import subprocess
import sys

VISUAL_STUDIO_GENERATOR = re.compile(r"^Visual Studio (?P<major>\d+) (?P<year>\d+)$")


def _load_generators() -> list[dict[str, object]]:
    result = subprocess.run(
        ["cmake", "-E", "capabilities"],
        check=True,
        stdout=subprocess.PIPE,
        text=True,
    )
    return json.loads(result.stdout).get("generators", [])


def _visual_studio_generator_key(name: str) -> tuple[int, int] | None:
    match = VISUAL_STUDIO_GENERATOR.match(name)
    if not match:
        return None
    return int(match.group("major")), int(match.group("year"))


def select_visual_studio_generator(generators: list[dict[str, object]]) -> str | None:
    candidates: list[tuple[int, int, str]] = []
    for generator in generators:
        name = generator.get("name")
        if not isinstance(name, str):
            continue

        key = _visual_studio_generator_key(name)
        if key is None:
            continue

        major, year = key
        candidates.append((major, year, name))

    if not candidates:
        return None

    return max(candidates)[2]


def main() -> int:
    override = os.environ.get("DART_WINDOWS_CMAKE_GENERATOR")
    if override:
        print(override)
        return 0

    generator = select_visual_studio_generator(_load_generators())
    if generator is None:
        print(
            "No Visual Studio CMake generator found. Install Visual Studio or "
            "set DART_WINDOWS_CMAKE_GENERATOR.",
            file=sys.stderr,
        )
        return 1

    print(generator)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
