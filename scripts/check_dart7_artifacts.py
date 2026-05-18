#!/usr/bin/env python3
"""Check whether published DART 7 package artifacts are available."""

from __future__ import annotations

import json
import sys
import urllib.request
from dataclasses import dataclass
from typing import Any
from urllib.error import HTTPError, URLError

PYPI_URL = "https://pypi.org/pypi/dartpy/json"
ANACONDA_URL = "https://api.anaconda.org/package/conda-forge/{name}"
GITHUB_RELEASES_URL = "https://api.github.com/repos/dartsim/dart/releases?per_page=100"
GITHUB_TAGS_URL = "https://api.github.com/repos/dartsim/dart/git/matching-refs/tags/v7"


@dataclass(frozen=True)
class ArtifactStatus:
    name: str
    available: bool
    detail: str
    required: bool = True


def fetch_json(url: str) -> Any:
    request = urllib.request.Request(
        url,
        headers={"User-Agent": "dart-artifact-check/1.0"},
    )
    with urllib.request.urlopen(request, timeout=30) as response:
        return json.load(response)


def major_version(version: str) -> int | None:
    if version.startswith("v"):
        version = version[1:]
    head = version.split(".", maxsplit=1)[0]
    if not head.isdigit():
        return None
    return int(head)


def is_dart7(version: str) -> bool:
    major = major_version(version)
    return major == 7


def version_key(version: str) -> tuple[int, ...]:
    if version.startswith("v"):
        version = version[1:]
    parts: list[int] = []
    for part in version.split("."):
        if not part.isdigit():
            break
        parts.append(int(part))
    return tuple(parts)


def check_pypi_dartpy() -> ArtifactStatus:
    data = fetch_json(PYPI_URL)
    if not isinstance(data, dict):
        raise TypeError("PyPI API returned unexpected data")
    releases = data.get("releases", {})
    dart7: list[str] = []
    yanked: list[str] = []

    for version, files in sorted(
        releases.items(), key=lambda item: version_key(item[0]), reverse=True
    ):
        if not is_dart7(version):
            continue
        non_yanked = [file for file in files if not file.get("yanked", False)]
        if non_yanked:
            dart7.append(f"{version} ({len(non_yanked)} file(s))")
        elif files:
            yanked.append(f"{version} ({len(files)} yanked file(s))")

    latest = data.get("info", {}).get("version", "unknown")
    if dart7:
        return ArtifactStatus(
            "PyPI dartpy",
            True,
            f"latest={latest}; non-yanked DART 7 release(s): {', '.join(dart7)}",
        )

    yanked_detail = f"; yanked DART 7 release(s): {', '.join(yanked)}" if yanked else ""
    return ArtifactStatus(
        "PyPI dartpy",
        False,
        f"latest={latest}; no non-yanked DART 7 files{yanked_detail}",
    )


def conda_versions(package: str) -> set[str]:
    data = fetch_json(ANACONDA_URL.format(name=package))
    if not isinstance(data, dict):
        raise TypeError(f"Anaconda API returned unexpected data for {package}")
    versions = {str(version) for version in data.get("versions", [])}
    for file in data.get("files", []):
        version = file.get("version")
        if version:
            versions.add(str(version))
    return versions


def check_conda_package(package: str) -> ArtifactStatus:
    versions = sorted(conda_versions(package), key=version_key, reverse=True)
    dart7 = [version for version in versions if is_dart7(version)]
    latest = versions[0] if versions else "none"
    if dart7:
        return ArtifactStatus(
            f"conda-forge {package}",
            True,
            f"latest={latest}; DART 7 version(s): {', '.join(dart7[:5])}",
        )
    return ArtifactStatus(
        f"conda-forge {package}",
        False,
        f"latest={latest}; no DART 7 version found",
    )


def check_github_release_signal() -> ArtifactStatus:
    tags_data = fetch_json(GITHUB_TAGS_URL)
    releases_data = fetch_json(GITHUB_RELEASES_URL)
    if not isinstance(tags_data, list) or not isinstance(releases_data, list):
        raise TypeError("GitHub API returned unexpected data")

    tags = []
    for item in tags_data:
        if not isinstance(item, dict):
            continue
        ref = str(item.get("ref", ""))
        tag = ref.rsplit("/", maxsplit=1)[-1]
        if is_dart7(tag):
            tags.append(tag)

    releases = []
    for item in releases_data:
        if not isinstance(item, dict):
            continue
        if item.get("draft", False):
            continue
        tag = str(item.get("tag_name", ""))
        if is_dart7(tag):
            releases.append(tag)

    if tags or releases:
        tag_detail = ", ".join(sorted(tags, key=version_key, reverse=True)) or "none"
        release_detail = (
            ", ".join(sorted(releases, key=version_key, reverse=True)) or "none"
        )
        return ArtifactStatus(
            "GitHub DART 7 release signal",
            True,
            f"remote v7 tag(s): {tag_detail}; public release(s): {release_detail}",
            required=False,
        )

    return ArtifactStatus(
        "GitHub DART 7 release signal",
        False,
        "no remote v7 tag; no public DART 7 release",
        required=False,
    )


def check_optional_github_release_signal() -> ArtifactStatus:
    try:
        return check_github_release_signal()
    except (
        HTTPError,
        URLError,
        TimeoutError,
        TypeError,
        json.JSONDecodeError,
    ) as error:
        return ArtifactStatus(
            "GitHub DART 7 release signal",
            False,
            f"could not check optional release signal: {error}",
            required=False,
        )


def main() -> int:
    try:
        statuses = [
            check_pypi_dartpy(),
            check_conda_package("dartpy"),
            check_conda_package("dartsim-cpp"),
        ]
    except (
        HTTPError,
        URLError,
        TimeoutError,
        TypeError,
        json.JSONDecodeError,
    ) as error:
        print(f"Artifact check failed: {error}", file=sys.stderr)
        return 2

    statuses.append(check_optional_github_release_signal())

    for status in statuses:
        if status.available:
            marker = "OK"
        elif status.required:
            marker = "BLOCKED"
        else:
            marker = "INFO"
        print(f"{marker}: {status.name}: {status.detail}")

    python_available = statuses[0].available or statuses[1].available
    cpp_available = statuses[2].available
    if python_available and cpp_available:
        print("DART 7 Python and C++ package artifacts are available.")
        return 0

    print("DART 7 package artifacts are still incomplete.")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
