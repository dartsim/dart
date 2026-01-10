#!/usr/bin/env python3
import argparse
import os
import shlex
import subprocess
from pathlib import Path

DEFAULT_IMAGE = "altlinux/base:sisyphus"
DEFAULT_CONTAINER = "dart-altlinux"
DEFAULT_VOLUME = "dart-altlinux-work"
DEFAULT_SOURCE_DIR = "/src"
DEFAULT_WORK_DIR = "/work/dart"
DEFAULT_BUILD_DIR = "build/altlinux/cpp/Release"
DEFAULT_BUILD_TARGETS = ["tests"]
DEFAULT_TEST_REGEX = ""
DEFAULT_PACKAGES = [
    "cmake",
    "eigen3-devel",
    "gcc",
    "gcc-c++",
    "git",
    "libassimp-devel",
    "libbenchmark-devel",
    "libbullet3-devel",
    "libccd-devel",
    "libfcl-devel",
    "libflann-devel",
    "libfmt-devel",
    "libgtest-devel",
    "libnlopt-devel",
    "libOpenSceneGraph-devel",
    "libode-devel",
    "libtinyxml2-devel",
    "libfreeglut-devel",
    "libXi-devel",
    "tinyxml-devel",
    "liburdfdom-devel",
    "make",
    "ninja-build",
    "pkg-config",
    "rsync",
    "urdfdom-headers",
]
EXCLUDES = [".git", "build", ".pixi", ".deps", ".build"]


def run(cmd, check=True, capture=False):
    if capture:
        return subprocess.run(cmd, check=check, text=True, capture_output=True)
    return subprocess.run(cmd, check=check)


def repo_root():
    return Path(__file__).resolve().parents[1]


def container_exists(container):
    result = run(
        [
            "docker",
            "ps",
            "-a",
            "--filter",
            f"name={container}",
            "--format",
            "{{.Names}}",
        ],
        check=False,
        capture=True,
    )
    return container in (result.stdout or "").splitlines()


def container_running(container):
    result = run(
        ["docker", "ps", "--filter", f"name={container}", "--format", "{{.Names}}"],
        check=False,
        capture=True,
    )
    return container in (result.stdout or "").splitlines()


def ensure_started(args):
    if not container_running(args.container):
        start_container(args)


def start_container(args):
    if container_running(args.container):
        print(f"Container '{args.container}' already running.")
        return

    if container_exists(args.container):
        run(["docker", "rm", "-f", args.container])

    run(["docker", "volume", "create", args.volume], check=False)

    cmd = [
        "docker",
        "run",
        "-d",
        "--name",
        args.container,
        "-v",
        f"{args.volume}:/work",
        "-v",
        f"{repo_root()}:{args.source_dir}:ro",
        args.image,
        "/bin/sh",
        "-lc",
        "sleep infinity",
    ]
    run(cmd)


def stop_container(args):
    if container_exists(args.container):
        run(["docker", "rm", "-f", args.container])


def exec_in_container(args, command, tty=False):
    cmd = ["docker", "exec", "-i"]
    if tty:
        cmd.append("-t")
    cmd.append(args.container)
    if tty:
        cmd.append("/bin/sh")
        run(cmd)
        return
    cmd.extend(["/bin/sh", "-lc", command])
    run(cmd)


def shell_container(args):
    ensure_started(args)
    exec_in_container(args, command=None, tty=True)


def should_skip_bootstrap():
    value = os.getenv("ALT_LINUX_SKIP_BOOTSTRAP", "")
    return value.lower() in {"1", "true", "yes"}


def bootstrap_container(args):
    ensure_started(args)
    if should_skip_bootstrap():
        return
    packages_env = os.getenv("ALT_LINUX_PACKAGES")
    packages = shlex.split(packages_env) if packages_env else DEFAULT_PACKAGES
    package_list = " ".join(shlex.quote(pkg) for pkg in packages)
    command = f"apt-get update && apt-get install -y {package_list}"
    exec_in_container(args, command)


def sync_repo(args):
    ensure_started(args)
    excludes = " ".join(f"--exclude {shlex.quote(item)}" for item in EXCLUDES)
    work_dir = shlex.quote(args.work_dir)
    src_dir = shlex.quote(args.source_dir)
    command = (
        f"mkdir -p {work_dir} && rsync -az --delete {excludes} {src_dir}/ {work_dir}/"
    )
    exec_in_container(args, command)


def test_container(args):
    ensure_started(args)
    bootstrap_container(args)
    sync_repo(args)

    build_dir = os.getenv("ALT_LINUX_BUILD_DIR", DEFAULT_BUILD_DIR)
    build_type = os.getenv("ALT_LINUX_BUILD_TYPE", "Release")
    cmake_args = [
        f"-DCMAKE_BUILD_TYPE={build_type}",
        "-DDART_BUILD_DARTPY=OFF",
        "-DDART_BUILD_GUI_OSG=ON",
        "-DDART_ENABLE_SIMD=OFF",
        "-DDART_TREAT_WARNINGS_AS_ERRORS=OFF",
        "-DDART_USE_SYSTEM_GOOGLEBENCHMARK=ON",
        "-DDART_USE_SYSTEM_GOOGLETEST=ON",
        "-DDART_VERBOSE=ON",
    ]
    cmake_args.extend(shlex.split(os.getenv("ALT_LINUX_CMAKE_ARGS", "")))
    cmake_arg_str = " ".join(shlex.quote(arg) for arg in cmake_args)

    build_targets_env = os.getenv("ALT_LINUX_BUILD_TARGETS")
    if build_targets_env:
        build_targets = shlex.split(build_targets_env)
        if not build_targets:
            build_targets = DEFAULT_BUILD_TARGETS
    else:
        build_targets = DEFAULT_BUILD_TARGETS
    build_targets_str = " ".join(build_targets)

    test_regex = os.getenv("ALT_LINUX_TEST_REGEX", DEFAULT_TEST_REGEX).strip()
    if test_regex:
        test_command = (
            f"ctest --test-dir {shlex.quote(build_dir)} "
            f"-R {shlex.quote(test_regex)} --output-on-failure"
        )
    else:
        test_command = f"ctest --test-dir {shlex.quote(build_dir)} --output-on-failure"

    command = (
        f"cd {shlex.quote(args.work_dir)} && "
        f"cmake -G Ninja -S . -B {shlex.quote(build_dir)} {cmake_arg_str} && "
        f"cmake --build {shlex.quote(build_dir)} --target {build_targets_str} && "
        f"{test_command}"
    )
    exec_in_container(args, command)


def parse_args():
    parser = argparse.ArgumentParser(description="Manage Alt Linux repro via Docker.")
    parser.add_argument(
        "--image",
        default=os.getenv("ALT_LINUX_IMAGE", DEFAULT_IMAGE),
        help="Docker image tag.",
    )
    parser.add_argument(
        "--container",
        default=os.getenv("ALT_LINUX_CONTAINER", DEFAULT_CONTAINER),
        help="Container name.",
    )
    parser.add_argument(
        "--volume",
        default=os.getenv("ALT_LINUX_VOLUME", DEFAULT_VOLUME),
        help="Docker volume for working files.",
    )
    parser.add_argument(
        "--source-dir",
        default=DEFAULT_SOURCE_DIR,
        help="Source mount point inside the container.",
    )
    parser.add_argument(
        "--work-dir",
        default=os.getenv("ALT_LINUX_WORK_DIR", DEFAULT_WORK_DIR),
        help="Working directory inside the container.",
    )

    subparsers = parser.add_subparsers(dest="command", required=True)
    subparsers.add_parser("start")
    subparsers.add_parser("stop")
    subparsers.add_parser("shell")
    subparsers.add_parser("sync")
    subparsers.add_parser("bootstrap")
    subparsers.add_parser("test")

    return parser.parse_args()


def main():
    args = parse_args()
    if args.command == "start":
        start_container(args)
    elif args.command == "stop":
        stop_container(args)
    elif args.command == "shell":
        shell_container(args)
    elif args.command == "sync":
        sync_repo(args)
    elif args.command == "bootstrap":
        bootstrap_container(args)
    elif args.command == "test":
        test_container(args)


if __name__ == "__main__":
    main()
