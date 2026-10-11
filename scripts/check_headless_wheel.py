#!/usr/bin/env python3
"""Check the Linux wheel's C22 linkage, headless import and rendering errors."""

from __future__ import annotations

import argparse
import re
import subprocess
import tempfile
import zipfile
from pathlib import Path


def needed_libraries(dynamic_section: str) -> list[str]:
    return re.findall(r"\(NEEDED\).*?\[([^\]]+)\]", dynamic_section)


def forbidden_library(name: str) -> bool:
    # auditwheel adds a hash before the .so suffix; catch those names too.
    return name.startswith(
        ("libGL", "libEGL", "libOpenGL", "libX11", "libxcb", "libwayland-", "libvulkan")
    )


def audit_wheel(wheel: Path) -> None:
    subprocess.run(["auditwheel", "show", str(wheel)], check=True)
    count = 0
    failures = []
    with zipfile.ZipFile(wheel) as archive, tempfile.TemporaryDirectory() as tmp:
        elf = Path(tmp) / "library.so"
        for member in archive.infolist():
            with archive.open(member) as source:
                if source.read(4) != b"\x7fELF":
                    continue
                elf.write_bytes(b"\x7fELF" + source.read())
            result = subprocess.run(
                ["readelf", "-d", str(elf)],
                check=True,
                capture_output=True,
                text=True,
            )
            libraries = needed_libraries(result.stdout)
            print(f"NEEDED {member.filename}: {', '.join(libraries)}", flush=True)
            failures.extend(
                f"{member.filename}: {name}"
                for name in libraries
                if forbidden_library(name)
            )
            count += 1
    if count == 0:
        raise RuntimeError("Wheel contains no ELF objects")
    if failures:
        raise RuntimeError("Forbidden graphics dependencies: " + "; ".join(failures))
    print(f"LINKAGE PASS: {count} ELF objects; no graphics NEEDED entries", flush=True)


RUNTIME_CHECK = r'''
import ctypes
import subprocess
import sys

subprocess.run(
    [sys.executable, '-m', 'pip', 'install', '--disable-pip-version-check',
     '--root-user-action=ignore', sys.argv[1]], check=True)
for name in ('libGL.so.1', 'libEGL.so.1', 'libOpenGL.so.0', 'libX11.so.6',
             'libxcb.so.1', 'libwayland-client.so.0', 'libwayland-server.so.0',
             'libwayland-egl.so.1', 'libvulkan.so.1'):
    try:
        ctypes.CDLL(name)
    except OSError:
        continue
    raise RuntimeError(f'Container unexpectedly provides {name}')
print('CONTAINER: no GL/EGL/OpenGL/X11/xcb/Wayland/Vulkan libraries', flush=True)
import dartpy as dart
world = dart.simulation.World()
for _ in range(10):
    world.step()
assert abs(world.getTime() - 10 * world.getTimeStep()) < 1e-12
print('HEADLESS PASS: import dartpy; World.step() x10', flush=True)

failed = False
for mode, setup, packages in (
    ('capture', 'setUpViewForCapture(32, 24)', ('libvulkan1', 'mesa-vulkan-drivers')),
    ('window', 'setUpViewInWindow(0, 0, 32, 24)', ('libx11-6', 'libgl1')),
    ('window-default', 'setUpViewInWindow()', ('libx11-6', 'libgl1')),
):
    # Separate processes expose aborts and segfaults without hiding other cases.
    code = f"""
import dartpy as dart
try:
    viewer = dart.gui.Viewer()
    viewer.{setup}
    viewer.frame()
except dart.gui.RenderingUnavailableError as error:
    assert isinstance(error, RuntimeError)
    assert all(package in str(error) for package in {packages!r}), str(error)
    world = dart.simulation.World()
    for _ in range(10):
        world.step()
    print('{mode} PASS: ' + type(error).__name__ + ': ' + str(error), flush=True)
else:
    raise AssertionError('Rendering unexpectedly succeeded without a backend')
"""
    result = subprocess.run([sys.executable, '-u', '-c', code], timeout=60,
                            capture_output=True, text=True)
    print(result.stdout, end='', flush=True)
    print(result.stderr, end='', file=sys.stderr, flush=True)
    print(f'{mode}: process exit {result.returncode}', flush=True)
    failed |= (result.returncode != 0 or
               f'{mode} PASS: RenderingUnavailableError:' not in result.stdout)
if failed:
    raise SystemExit(1)
'''


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("wheel", type=Path)
    parser.add_argument("--audit-only", action="store_true", help=argparse.SUPPRESS)
    args = parser.parse_args()
    wheel = args.wheel.resolve(strict=True)
    if not re.fullmatch(
        r"dartpy-.+-cp312-abi3-manylinux_2_28_(x86_64|aarch64)\.whl", wheel.name
    ):
        parser.error("Expected a cp312-abi3-manylinux_2_28 Linux dartpy wheel")
    if args.audit_only:
        audit_wheel(wheel)
        return
    arch = wheel.name.rsplit("manylinux_2_28_", 1)[1].removesuffix(".whl")
    mount = ["-v", f"{wheel.parent}:/w:ro"]
    subprocess.run(
        [
            "docker",
            "run",
            "--rm",
            *mount,
            "-v",
            f"{Path(__file__).resolve()}:/check.py:ro",
            f"quay.io/pypa/manylinux_2_28_{arch}",
            "/opt/python/cp312-cp312/bin/python",
            "/check.py",
            f"/w/{wheel.name}",
            "--audit-only",
        ],
        check=True,
    )
    subprocess.run(
        [
            "docker",
            "run",
            "--rm",
            "-i",
            *mount,
            "python:3.12-slim",
            "python",
            "-u",
            "-",
            f"/w/{wheel.name}",
        ],
        input=RUNTIME_CHECK,
        text=True,
        check=True,
    )


if __name__ == "__main__":
    main()
