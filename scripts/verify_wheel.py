#!/usr/bin/env python3
"""Verify the layout of dartpy wheels: the package, its extension and licenses."""

from __future__ import annotations

import glob
import sys
import zipfile
from pathlib import Path

# Compressed baselines: Linux x86_64 20,282,150 B; macOS arm64 14,083,197 B;
# Windows x64 12,597,754 B. Linux aarch64 shares the Linux budget.
WHEEL_SIZE_LIMITS = {"linux": 23_000_000, "macosx": 16_000_000, "win": 14_500_000}
TLS_NOTICES = ("curl.txt", "openssl.txt")


def _expand_wheels(patterns: list[str]) -> list[Path]:
    wheel_paths: list[Path] = []
    for pattern in patterns:
        matches = sorted(Path(path) for path in glob.glob(pattern))
        if matches:
            wheel_paths.extend(matches)
        else:
            wheel_paths.append(Path(pattern))
    return wheel_paths


def verify_wheel(wheel_path: Path) -> bool:
    print(f"Verifying wheel: {wheel_path}")

    if not wheel_path.exists():
        print(f"ERROR: Wheel file not found: {wheel_path}")
        return False

    platform_tag = wheel_path.stem.rsplit("-", 1)[-1]
    if platform_tag.startswith(("manylinux_", "linux_")):
        platform = "linux"
    elif platform_tag.startswith("macosx_"):
        platform = "macosx"
    elif platform_tag.startswith("win_"):
        platform = "win"
    else:
        print(f"ERROR: No wheel size budget for platform {platform_tag}.")
        return False
    limit = WHEEL_SIZE_LIMITS[platform]
    with zipfile.ZipFile(wheel_path, "r") as wheel:
        file_list = wheel.namelist()
        members = wheel.infolist()
    print(
        f"Wheel size: {wheel_path.stat().st_size:,} bytes compressed; "
        f"{sum(member.file_size for member in members):,} bytes unpacked."
    )
    for member in members:
        if not member.is_dir() and member.filename.startswith(
            ("dartpy.libs/", "dartpy/.dylibs/")
        ):
            print(
                f"Bundled library: {member.filename} "
                f"({member.compress_size:,} bytes compressed; "
                f"{member.file_size:,} bytes unpacked)"
            )
    ok = True
    if wheel_path.stat().st_size > limit:
        print(
            f"ERROR: {platform} wheel size {wheel_path.stat().st_size:,} bytes "
            f"exceeds the {limit:,}-byte compressed budget."
        )
        ok = False

    names = set(file_list)
    extensions = [
        name
        for name in file_list
        if name.startswith("dartpy/_core.") and name.endswith((".so", ".pyd"))
    ]
    non_metadata_files = [
        name
        for name in file_list
        if ".dist-info/" not in name and not name.endswith(".dist-info")
    ]
    dist_info = next(
        (name.partition("/")[0] for name in file_list if name.endswith("/METADATA")),
        "",
    )

    if "-cp312-abi3-" not in wheel_path.name:
        print("ERROR: Expected a cp312-abi3 wheel (DART_DARTPY_STABLE_ABI).")
        ok = False
    if not extensions:
        print("ERROR: No dartpy._core extension module found in wheel.")
        ok = False
    required_files = (
        "dartpy/__init__.py",
        "dartpy/py.typed",
        "dartpy/experimental/__init__.py",
        f"{dist_info}/licenses/LICENSE",
        f"{dist_info}/licenses/NOTICE",
        f"{dist_info}/licenses/filament/LICENSE",
    )
    # macOS/Windows keep utils-assets off until its dependency cost is approved.
    if platform == "linux" or any(
        "curl" in name.lower() for name in file_list if ".dist-info/" not in name
    ):
        required_files += tuple(
            f"{dist_info}/licenses/python/dartpy/licenses/{name}"
            for name in TLS_NOTICES
        )
    for required in required_files:
        if required not in names:
            print(f"ERROR: Wheel is missing {required}.")
            ok = False
    # auditwheel and delvewheel vendor shared libraries into dartpy.libs/;
    # delocate uses dartpy/.dylibs/ inside the package.
    stray = sorted(
        name
        for name in file_list
        if not name.startswith(("dartpy/", "dartpy.libs/", f"{dist_info}/"))
        or name.startswith("dartpy/_filament_licenses/")
    )
    if stray:
        print(
            "ERROR: Wheel contains files outside the dartpy package: "
            + ", ".join(stray)
        )
        ok = False
    if not non_metadata_files:
        print("ERROR: Wheel contains only metadata files.")
        ok = False
    if not ok:
        return False

    for extension in extensions:
        print(f"Found extension: {extension}")
    print(f"Wheel contains {len(non_metadata_files)} non-metadata files.")
    return True


def main(argv: list[str]) -> int:
    if len(argv) < 2:
        print("Usage: python scripts/verify_wheel.py <wheel_file_or_glob> [...]")
        return 1

    wheel_paths = _expand_wheels(argv[1:])
    if not wheel_paths:
        print("ERROR: No wheels found.")
        return 1

    ok = True
    for wheel_path in wheel_paths:
        ok = verify_wheel(wheel_path) and ok

    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
