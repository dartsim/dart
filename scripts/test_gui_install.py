#!/usr/bin/env python3
"""Install, relocate, and exercise the public C++17 GUI component."""

import argparse
import hashlib
import os
import re
import subprocess
from pathlib import Path


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("build_dir", type=Path)
    args = parser.parse_args()
    build_dir = args.build_dir.resolve()
    source_dir = Path(__file__).resolve().parents[1]
    original = build_dir / "gui-install-original"
    relocated = build_dir / "gui-install-relocated"
    consumer = build_dir / "gui-installed-consumer"
    if original.exists() or relocated.exists() or consumer.exists():
        raise RuntimeError(
            "GUI install qualification requires fresh staging directories"
        )

    # dartpy installs into the active interpreter; stage only the C++ package.
    for component in ("headers", "Unspecified"):
        subprocess.run(
            [
                "cmake",
                "--install",
                str(build_dir),
                "--prefix",
                str(original),
                "--component",
                component,
            ],
            check=True,
        )
    original.rename(relocated)
    if (relocated / "include/dart/gui/detail").exists():
        raise RuntimeError("Private GUI implementation headers were installed")
    cache = (build_dir / "CMakeCache.txt").read_text()
    if "DART_USE_SYSTEM_IMGUI:BOOL=OFF" in cache:
        # Emulate a prefix that also holds another ImGui: <imgui.h> must still
        # resolve to the bundled docking headers.
        (relocated / "include/imgui.h").write_text(
            '#error "<imgui.h> resolved to a non-DART ImGui header"\n'
        )

    licenses = relocated / "share/licenses/dart/filament"
    notice_manifest = (source_dir / "cmake/DARTFilamentNotices.cmake").read_text()
    for relative, sha256 in re.findall(r'"([^"|]+)\|([a-f0-9]{64})"', notice_manifest):
        installed = (
            licenses / relative
            if relative == "LICENSE"
            else licenses / "notices" / relative
        )
        contents = installed.read_bytes()
        if relative == "LICENSE":
            contents = contents.replace(b"\r\n", b"\n")
        if hashlib.sha256(contents).hexdigest() != sha256:
            raise RuntimeError(
                f"Installed Filament notice does not match the SDK: {installed}"
            )

    environment = os.environ.copy()
    environment.pop("DISPLAY", None)
    environment.pop("WAYLAND_DISPLAY", None)
    runtime_dirs = [relocated / "bin", relocated / "lib"]
    prefix = environment.get("CONDA_PREFIX")
    prefixes = [relocated]
    if prefix:
        prefixes.extend([Path(prefix), Path(prefix) / "Library"])
        runtime_dirs.extend(
            [
                Path(prefix) / "bin",
                Path(prefix) / "lib",
                Path(prefix) / "Library/bin",
                Path(prefix) / "Library/lib",
            ]
        )
    for variable in ("PATH", "LD_LIBRARY_PATH", "DYLD_LIBRARY_PATH"):
        environment[variable] = os.pathsep.join(
            [
                *(str(directory) for directory in runtime_dirs),
                environment.get(variable, ""),
            ]
        )

    subprocess.run(
        [
            "cmake",
            "-S",
            str(source_dir / "tests/cmake/gui_consumer"),
            "-B",
            str(consumer),
            "-G",
            "Ninja",
            "-DCMAKE_BUILD_TYPE=Release",
            f"-DDART_DIR={relocated / 'share/dart/cmake'}",
            "-DCMAKE_PREFIX_PATH=" + ";".join(str(path) for path in prefixes),
        ],
        env=environment,
        check=True,
    )
    subprocess.run(
        ["cmake", "--build", str(consumer), "--parallel", "2"],
        env=environment,
        check=True,
    )
    subprocess.run(
        [
            "ctest",
            "--test-dir",
            str(consumer),
            "--output-on-failure",
            "--no-tests=error",
        ],
        env=environment,
        check=True,
    )


if __name__ == "__main__":
    main()
