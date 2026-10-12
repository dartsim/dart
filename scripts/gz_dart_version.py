#!/usr/bin/env python3
"""Select DART's CI-only version override for Gazebo compatibility builds."""

import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

VERSION_PATTERN = r"(?:0|[1-9][0-9]*)\.(?:0|[1-9][0-9]*)\.(?:0|[1-9][0-9]*)"


def version_override(source: Path) -> str:
    package_version = ET.parse(source / "package.xml").getroot().findtext("version", "")
    if not re.fullmatch(VERSION_PATTERN, package_version):
        raise ValueError("package.xml must contain a numeric X.Y.Z version")
    targets = re.findall(
        rf"^\|\s*`?main`?\s*\|\s*Development\s*\|\s*`?({VERSION_PATTERN})`?\s*\|\s*$",
        (source / "docs/onboarding/release-management.md").read_text(encoding="utf-8"),
        re.MULTILINE,
    )
    if len(targets) != 1:
        raise ValueError(
            "docs/onboarding/release-management.md must contain exactly one "
            "main | Development | X.Y.Z release target"
        )
    target = targets[0]
    return (
        target
        if tuple(map(int, target.split(".")))
        > tuple(map(int, package_version.split(".")))
        else ""
    )


def main() -> int:
    source = (
        Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).resolve().parents[1]
    )
    try:
        override = version_override(source)
    except (OSError, ValueError, ET.ParseError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 2
    # Always pass the cache entry, including empty, to clear an earlier override.
    print(f"-DDART_VERSION_OVERRIDE:STRING={override}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
