#!/usr/bin/env python3
"""
CMake configuration script for DART builds.

This script handles CMake configuration with different build types and options.
It replaces the long embedded bash scripts in pixi.toml for better maintainability.
"""

import argparse
import os
import subprocess
import sys
from pathlib import Path


def get_compiler_launcher():
    """Check if sccache is available and return launcher definitions."""
    launcher_defs = []

    if not os.environ.get("CMAKE_C_COMPILER_LAUNCHER"):
        if (
            subprocess.run(
                ["which", "sccache"], capture_output=True, check=False
            ).returncode
            == 0
        ):
            launcher_defs.extend(
                [
                    "-DCMAKE_C_COMPILER_LAUNCHER=sccache",
                    "-DCMAKE_CXX_COMPILER_LAUNCHER=sccache",
                ]
            )
    else:
        c_launcher = os.environ.get("CMAKE_C_COMPILER_LAUNCHER")
        cxx_launcher = os.environ.get("CMAKE_CXX_COMPILER_LAUNCHER")
        if c_launcher:
            launcher_defs.append(f"-DCMAKE_C_COMPILER_LAUNCHER={c_launcher}")
        if cxx_launcher:
            launcher_defs.append(f"-DCMAKE_CXX_COMPILER_LAUNCHER={cxx_launcher}")

    return launcher_defs


def configure_cmake(
    build_type="Release",
    dartpy="ON",
    dart7="ON",
    dartpy7=None,
    gui_osg="ON",
    build_profile="ON",
    enable_asan="OFF",
    codecov="OFF",
    verbose="OFF",
):
    """Configure CMake for DART build."""

    # Get environment variables
    pixi_env = os.environ.get("PIXI_ENVIRONMENT_NAME", "default")
    conda_prefix = os.environ.get("CONDA_PREFIX", "")

    # Override from environment if set
    build_profile = os.environ.get("DART_BUILD_PROFILE", build_profile)
    enable_asan = os.environ.get("DART_ENABLE_ASAN", enable_asan)
    verbose = os.environ.get("DART_VERBOSE", verbose)

    # Default dartpy7 to same as dartpy if not specified
    if dartpy7 is None:
        dartpy7 = dartpy

    # Build directory
    build_dir = Path("build") / pixi_env / "cpp" / build_type

    # Get compiler launcher settings
    launcher_defs = get_compiler_launcher()

    # Build CMake command
    cmake_cmd = [
        "cmake",
        "-G",
        "Ninja",
        "-S",
        ".",
        "-B",
        str(build_dir),
        f"-DCMAKE_INSTALL_PREFIX={conda_prefix}",
        f"-DCMAKE_BUILD_TYPE={build_type}",
        f"-DCMAKE_PREFIX_PATH={conda_prefix}",
        f"-DDART_BUILD_DARTPY={dartpy}",
        f"-DDART_BUILD_DART7={dart7}",
        f"-DDART_BUILD_DARTPY7={dartpy7}",
        f"-DDART_BUILD_GUI_OSG={gui_osg}",
        f"-DDART_BUILD_PROFILE={build_profile}",
        f"-DDART_ENABLE_ASAN={enable_asan}",
        "-DDART_USE_SYSTEM_GOOGLEBENCHMARK=ON",
        "-DDART_USE_SYSTEM_GOOGLETEST=ON",
        "-DDART_USE_SYSTEM_IMGUI=ON",
        "-DDART_USE_SYSTEM_NANOBIND=OFF",
        "-DDART_USE_SYSTEM_TRACY=ON",
        f"-DDART_VERBOSE={verbose}",
    ]

    # Add pybind11 system flag (except for Windows)
    if sys.platform != "win32":
        cmake_cmd.append("-DDART_USE_SYSTEM_PYBIND11=ON")

    # Add codecov flag if enabled
    if codecov == "ON":
        cmake_cmd.append("-DDART_CODECOV=ON")

    # Add compiler launcher flags
    cmake_cmd.extend(launcher_defs)

    # Run CMake
    print(f"Configuring CMake for {build_type} build...")
    print(f"Command: {' '.join(cmake_cmd)}")

    result = subprocess.run(cmake_cmd, check=False)
    return result.returncode


def main():
    parser = argparse.ArgumentParser(description="Configure CMake for DART builds")
    parser.add_argument(
        "--build-type", default="Release", help="CMake build type (default: Release)"
    )
    parser.add_argument(
        "--dartpy",
        default="ON",
        choices=["ON", "OFF"],
        help="Build dartpy (default: ON)",
    )
    parser.add_argument(
        "--dart7", default="ON", choices=["ON", "OFF"], help="Build DART7 (default: ON)"
    )
    parser.add_argument(
        "--dartpy7",
        default=None,
        choices=["ON", "OFF"],
        help="Build dartpy7 (default: same as --dartpy)",
    )
    parser.add_argument(
        "--gui-osg",
        default="ON",
        choices=["ON", "OFF"],
        help="Build GUI OSG support (default: ON)",
    )
    parser.add_argument(
        "--build-profile",
        default="ON",
        choices=["ON", "OFF"],
        help="Enable build profiling (default: ON)",
    )
    parser.add_argument(
        "--enable-asan",
        default="OFF",
        choices=["ON", "OFF"],
        help="Enable AddressSanitizer (default: OFF)",
    )
    parser.add_argument(
        "--codecov",
        default="OFF",
        choices=["ON", "OFF"],
        help="Enable code coverage (default: OFF)",
    )
    parser.add_argument(
        "--verbose",
        default="OFF",
        choices=["ON", "OFF"],
        help="Enable verbose CMake output (default: OFF)",
    )

    args = parser.parse_args()

    return configure_cmake(
        build_type=args.build_type,
        dartpy=args.dartpy,
        dart7=args.dart7,
        dartpy7=args.dartpy7,
        gui_osg=args.gui_osg,
        build_profile=args.build_profile,
        enable_asan=args.enable_asan,
        codecov=args.codecov,
        verbose=args.verbose,
    )


if __name__ == "__main__":
    sys.exit(main())
