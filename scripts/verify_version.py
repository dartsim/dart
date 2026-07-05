#!/usr/bin/env python3
"""Verify version consistency across package.xml and built wheels."""

from __future__ import annotations

import glob
import re
import sys
import zipfile
from pathlib import Path

VERSION_PATTERN = r"^[0-9]+\.[0-9]+\.[0-9]+(?:\.(dev|alpha|beta|rc)[0-9]+)?$"


def extract_version_from_package_xml(package_xml_path: Path) -> str:
    content = package_xml_path.read_text()
    match = re.search(
        r"<version>([0-9]+\.[0-9]+\.[0-9]+(?:\.(dev|alpha|beta|rc)[0-9]+)?)</version>",
        content,
    )
    if not match:
        raise ValueError(f"Could not find version in {package_xml_path}")
    return match.group(1)


def extract_version_from_wheel(wheel_path: Path) -> str:
    with zipfile.ZipFile(wheel_path, "r") as zf:
        metadata_files = [name for name in zf.namelist() if name.endswith("/METADATA")]
        if not metadata_files:
            raise ValueError(f"No METADATA file found in {wheel_path}")

        metadata_content = zf.read(metadata_files[0]).decode("utf-8")
        match = re.search(r"^Version: (.+)$", metadata_content, re.MULTILINE)
        if not match:
            raise ValueError("Could not find Version in METADATA")
        return match.group(1).strip()


def _expand_wheels(patterns: list[str]) -> list[Path]:
    wheel_paths: list[Path] = []
    for pattern in patterns:
        matches = sorted(Path(path) for path in glob.glob(pattern))
        if matches:
            wheel_paths.extend(matches)
        else:
            wheel_paths.append(Path(pattern))
    return wheel_paths


def main(argv: list[str]) -> int:
    repo_root = Path(__file__).parent.parent
    pkg_version = extract_version_from_package_xml(repo_root / "package.xml")

    print(f"package.xml version: {pkg_version}")
    if not re.match(VERSION_PATTERN, pkg_version):
        print(f"ERROR: Invalid version format: {pkg_version}")
        print("Expected format: X.Y.Z or X.Y.Z.{dev|alpha|beta|rc}N")
        return 1

    if len(argv) == 1:
        print("No wheel paths provided; package.xml version is valid.")
        return 0

    all_match = True
    for wheel_path in _expand_wheels(argv[1:]):
        if not wheel_path.exists():
            print(f"ERROR: Wheel file not found: {wheel_path}")
            all_match = False
            continue

        try:
            wheel_version = extract_version_from_wheel(wheel_path)
        except Exception as error:
            print(f"ERROR: {wheel_path.name}: {error}")
            all_match = False
            continue

        if wheel_version == pkg_version:
            print(f"{wheel_path.name}: {wheel_version} matches package.xml")
        else:
            print(
                f"ERROR: {wheel_path.name}: {wheel_version} "
                f"does not match {pkg_version}"
            )
            all_match = False

    return 0 if all_match else 1


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
