#!/usr/bin/env python3

# Copyright (c) 2011-2023, The DART development contributors
# All rights reserved.

import os
import platform
import shutil
import subprocess
import sys

# Get environment variables.
BUILD_TYPE = os.environ.get("BUILD_TYPE", "Release")
BUILD_DARTPY = os.environ.get("BUILD_DARTPY", "OFF")
BUILD_DOCS = os.environ.get("BUILD_DOCS", "OFF")
COMPILER = os.environ.get("COMPILER", "gcc")
CODECOV = os.environ.get("CODECOV", "OFF")
CODE_DIR = os.environ.get("CODE_DIR", os.getcwd())
IN_DOCKER = os.environ.get("IN_DOCKER", "OFF")
CHECK_FORMAT = os.environ.get("CHECK_FORMAT", "OFF")
NUM_CORES = os.environ.get("NUM_CORES", "MAX")
BUILD_EXAMPLES = os.environ.get("BUILD_EXAMPLES", "ON")
BUILD_TUTORIALS = os.environ.get("BUILD_TUTORIALS", "ON")
TEST_INSTALLATION = os.environ.get("TEST_INSTALLATION", "ON")
IN_CI = os.environ.get("IN_CI", "OFF")
ENABLE_SIMD = os.environ.get("ENABLE_SIMD", "OFF")


def get_compiler_version():
    """Get the version of the compiler specified by the COMPILER environment variable"""
    command = [COMPILER, "--version"]
    output = subprocess.run(command, text=True, stdout=subprocess.PIPE).stdout.strip()
    return output


def get_cmake_version():
    """Get the version of CMake"""
    command = ["cmake", "--version"]
    output = subprocess.run(command, text=True, stdout=subprocess.PIPE).stdout.strip()
    return output


def get_num_threads():
    """Get the number of threads to use for a parallel build"""
    if platform.system() == "Linux":
        num_available_threads = os.cpu_count()
    elif platform.system() == "Darwin":
        num_available_threads = int(
            subprocess.run(
                ["sysctl", "-n", "hw.logicalcpu"],
                text=True,
                stdout=subprocess.PIPE,
                check=True,
            ).stdout.strip()
        )
    else:
        num_available_threads = 1
        print(
            f"[WARN] {platform.system()} is not supported to detect the number of logical CPU cores."
        )

    num_threads = num_available_threads if NUM_CORES == "MAX" else int(NUM_CORES)
    if num_threads > 60:
        num_threads = 60

    return num_threads, num_available_threads


compiler_version = get_compiler_version()
cmake_version = get_cmake_version()
num_threads, num_available_threads = get_num_threads()


def print_system_info():
    """Print system information"""
    print("=====================================")
    print("\n [ SYSTEM INFO ]\n")
    print(f" IN_DOCKER: {IN_DOCKER}")
    print(
        f" OS       : {platform.system()} {platform.release()} ({platform.machine()})"
    )
    print(f" Compiler : {COMPILER} {compiler_version}")
    print(f" CMake    : {cmake_version}")
    print(f" Cores    : {num_threads} / {num_available_threads}")
    print("\n=====================================")


# Print system info
print_system_info()

# Set compilers
if COMPILER == "gcc":
    os.environ["CC"] = "gcc"
    os.environ["CXX"] = "g++"
elif COMPILER == "clang":
    os.environ["CC"] = "clang"
    os.environ["CXX"] = "clang++"
else:
    print("Info: Compiler isn't specified. Using the system default.")


def create_workspace_dir():
    """Create the workspace directory and copy DART code if running in Docker"""
    if IN_DOCKER == "ON":
        # Create workspace folder
        ws_dir = "/ws"
        os.makedirs(ws_dir, exist_ok=True)

        # Copy DART code
        source_dir = os.path.join(ws_dir, "code")
        subprocess.run(["cp", "-r", CODE_DIR, source_dir])
    else:
        source_dir = CODE_DIR

    return source_dir


# Create workspace directory and copy DART code if running in Docker
source_dir = create_workspace_dir()
os.chdir(source_dir)

# Create build directory
build_dir = os.path.join(source_dir, "build")
os.makedirs(build_dir, exist_ok=True)

# Determine the CMake install prefix option
if platform.system() == "Linux":
    install_prefix_option = "-DCMAKE_INSTALL_PREFIX=/usr/"
elif platform.system() == "Darwin":
    install_prefix_option = (
        "-DCMAKE_INSTALL_PREFIX=/usr/local/ -DCMAKE_INSTALL_RPATH=/usr/local/lib/"
    )
else:
    install_prefix_option = ""
    print(
        f"[WARN] {platform.system()} is not supported to determine the CMake install prefix option."
    )

# Set additional CMake arguments on macOS
if platform.system() == "Darwin" and shutil.which("brew") is not None:
    opencl_headers_dir = subprocess.run(
        ["brew", "--prefix", "opencl-headers"],
        text=True,
        stdout=subprocess.PIPE,
        check=True,
    ).stdout.strip()
    opencl_headers_cpp_dir = subprocess.run(
        ["brew", "--prefix", "opencl-clhpp-headers"],
        text=True,
        stdout=subprocess.PIPE,
        check=True,
    ).stdout.strip()
    cmake_args = [
        f"-DOpenCLHeaders_DIR={opencl_headers_dir}/share/cmake/OpenCLHeaders",
        f"-DOpenCLHeadersCpp_DIR={opencl_headers_cpp_dir}/share/cmake/OpenCLHeadersCpp",
    ]
else:
    cmake_args = []

print(f"CMake install prefix option: {install_prefix_option}")
print(f"Additional CMake arguments: {cmake_args}")

subprocess.run(
    [
        "cmake",
        "-S",
        source_dir,
        "-B",
        build_dir,
        "-DCMAKE_BUILD_TYPE=" + BUILD_TYPE,
        "-DDART_VERBOSE=ON",
        "-DDART_TREAT_WARNINGS_AS_ERRORS=ON",
        "-DDART_CODECOV=" + CODECOV,
        "-DDART_IN_CI=" + IN_CI,
        "-DDART_ENABLE_SIMD=" + ENABLE_SIMD,
        *cmake_args,
        install_prefix_option,
    ]
)

# Check format
if CHECK_FORMAT == "ON":
    subprocess.run(["cmake", "--build", build_dir, "--target", "check-format"])

# DART: build, test, and install
subprocess.run(
    ["cmake", "--build", build_dir, "--target", "all", "tests", "-j", str(num_threads)]
)
subprocess.run(
    ["ctest", "--output-on-failure", "-j", str(num_threads), "--test-dir", build_dir]
)

if BUILD_EXAMPLES == "ON":
    subprocess.run(
        [
            "cmake",
            "--build",
            build_dir,
            "--target",
            "all",
            "examples",
            "-j",
            str(num_threads),
        ]
    )

if BUILD_TUTORIALS == "ON":
    subprocess.run(
        [
            "cmake",
            "--build",
            build_dir,
            "--target",
            "all",
            "tutorials",
            "-j",
            str(num_threads),
        ]
    )

# dartpy: build, test, and install
if BUILD_DARTPY == "ON":
    os.environ["DART_DATA_LOCAL_PATH"] = os.path.join(source_dir, "data")
    subprocess.run(
        ["cmake", "--build", build_dir, "--target", "dartpy", "-j", str(num_threads)]
    )
    subprocess.run(["cmake", "--build", build_dir, "--target", "pytest"])
    subprocess.run(
        [
            "find",
            CODE_DIR,
            "-type",
            "d",
            "-name",
            "directory_name",
            "-exec",
            "rm",
            "-rf",
            "{}",
            "+",
        ]
    )

subprocess.run(["cmake", "--build", build_dir, "--target", "install"])

# Code coverage report generation and upload to codecov.io (only for Linux)
if CODECOV == "ON":
    print("Info: Code coverage is enabled.")

    print("Downloading codecov script...")
    subprocess.run(["curl", "-Os", "https://uploader.codecov.io/latest/linux/codecov"])
    subprocess.run(["chmod", "+x", "codecov"])
    subprocess.run(["./codecov", "--version"])
    subprocess.run(["./codecov", "-t", os.environ["CODECOV_TOKEN"]])

    print("Generating code coverage report...")

    # Capture coverage info
    subprocess.run(["lcov", "--capture", "--directory", ".", "-o", "coverage.info"])

    # Filter out system and extra files.
    # To also not include test code in coverage add them with full path to the patterns: '*/tests/*'
    subprocess.run(
        [
            "lcov",
            "--remove",
            "coverage.info",
            "/usr/*",
            "*/.deps/*",
            "*/tests/*",
            "*/examples/*",
            "*/tutorials/*",
            "--output-file",
            "coverage.info",
        ]
    )

    # Output coverage data for debugging (optional)
    subprocess.run(["lcov", "--list", "coverage.info"])

    # Uploading to CodeCov
    print("Uploading code coverage report to codecov.io...")
    subprocess.run(["./codecov", "-f", "coverage.info"])

elif TEST_INSTALLATION == "ON":
    # DART: build an C++ example using installed DART
    print("Info: Testing the installation...")
    example_dir = os.path.join(source_dir, "examples", "hello_world")
    os.makedirs(os.path.join(example_dir, "build"), exist_ok=True)
    subprocess.run(
        ["cmake", example_dir, *cmake_args], cwd=os.path.join(example_dir, "build")
    )
    subprocess.run(
        ["make", "-j", str(num_threads)], cwd=os.path.join(example_dir, "build")
    )

# TODO: Fix this
# dartpy: run a Python example using installed dartpy
# if BUILD_DARTPY == "ON":
#     print("Info: Running a Python example...")
#     print(os.environ['PYTHONPATH'])
#     py_version = f"{sys.version_info[0]}.{sys.version_info[1]}"
#     py_version_major = py_version.split('.')[0]
#     subprocess.run(['python' + py_version_major, 'setup.py', 'bdist_wheel'], cwd=source_dir)
#     wheel_file = glob.glob(os.path.join(source_dir, 'dist', '*.whl'))[0]
#     subprocess.run(['pip' + py_version_major, 'install', wheel_file])
#     subprocess.run(['python' + py_version_major, 'main.py'], cwd=os.path.join(source_dir, 'python', 'examples', 'hello_world'))
#     subprocess.run(['find', CODE_DIR, '-type', 'd', '-name', 'directory_name', '-exec', 'rm', '-rf', '{}', '+'])
