#!/usr/bin/env python3
import argparse
import os
import shlex
import subprocess
import sys
import time
from pathlib import Path

DEFAULT_IMAGE = "dartsim/freebsd-vm:15"
DEFAULT_CONTAINER = "dart-freebsd-vm"
DEFAULT_SSH_PORT = 10022
DEFAULT_CPUS = 4
DEFAULT_MEM = 4096
DEFAULT_USER = "freebsd"
DEFAULT_REMOTE_DIR = None
DEFAULT_BUILD_DIR = "build/freebsd/cpp/Release"
DEFAULT_BUILD_TARGETS = ["tests"]
DEFAULT_TEST_REGEX = ""
DEFAULT_CTEST_TIMEOUT = 1200
DEFAULT_CTEST_STOP_ON_FAILURE = True
DEFAULT_PORTS_PATCH_DIR = "docker/freebsd/ports-patches"
DEFAULT_PACKAGES = [
    "assimp",
    "boost-libs",
    "cmake",
    "eigen",
    "fcl",
    "libfmt",
    "git",
    "gmake",
    "googletest",
    "libccd",
    "ninja",
    "octomap",
    "ode-double",
    "pkgconf",
    "rsync",
    "spdlog",
    "tinyxml2",
    "ros-urdfdom",
    "ros-urdfdom_headers",
]
DEFAULT_IMAGE_URL = (
    "https://download.freebsd.org/ftp/snapshots/VM-IMAGES/15.0-STABLE/"
    "amd64/Latest/FreeBSD-15.0-STABLE-amd64-BASIC-CLOUDINIT-ufs.qcow2.xz"
)
SSH_OPTIONS = [
    "-o",
    "StrictHostKeyChecking=no",
    "-o",
    "UserKnownHostsFile=/dev/null",
    "-o",
    "BatchMode=yes",
    "-o",
    "IdentitiesOnly=yes",
    "-o",
    "ServerAliveInterval=30",
    "-o",
    "ServerAliveCountMax=6",
]


def env_default_int(name, fallback):
    value = os.getenv(name)
    if not value:
        return fallback
    try:
        return int(value)
    except ValueError:
        return fallback


def run(cmd, check=True, capture=False):
    if capture:
        return subprocess.run(cmd, check=check, text=True, capture_output=True)
    return subprocess.run(cmd, check=check)


def repo_root():
    return Path(__file__).resolve().parents[1]


def docker_path():
    return str(repo_root() / "docker" / "freebsd")


def dockerfile_path():
    return str(repo_root() / "docker" / "freebsd" / "Dockerfile")


def ports_patch_dir():
    return repo_root() / DEFAULT_PORTS_PATCH_DIR


def vm_dir_path(default_dir):
    return Path(os.getenv("FREEBSD_VM_DIR", default_dir)).resolve()


def image_exists(image):
    result = run(["docker", "image", "inspect", image], check=False)
    return result.returncode == 0


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
    key_path = ssh_key_path(vm_dir_path(args.vm_dir))
    if container_running(args.container):
        if not key_path.exists():
            print(
                "SSH key missing for the running FreeBSD VM; restarting to regenerate it.",
                file=sys.stderr,
            )
            stop_container(args)
            start_container(args)
    else:
        start_container(args)
    wait_for_ssh_key(args)
    wait_for_ssh(args, user=args.user)


def build_image(args):
    run(
        [
            "docker",
            "build",
            "-t",
            args.image,
            "-f",
            dockerfile_path(),
            docker_path(),
        ]
    )


def start_container(args):
    if not image_exists(args.image):
        build_image(args)

    vm_dir = vm_dir_path(args.vm_dir)
    vm_dir.mkdir(parents=True, exist_ok=True)
    ensure_ssh_key(args)

    if container_running(args.container):
        print(f"Container '{args.container}' already running.")
        return

    if container_exists(args.container):
        run(["docker", "rm", "-f", args.container])

    cmd = [
        "docker",
        "run",
        "-d",
        "--name",
        args.container,
        "-p",
        f"{args.ssh_port}:{args.ssh_port}",
        "-v",
        f"{vm_dir}:/vm",
        "-e",
        f"FREEBSD_VM_IMAGE_URL={args.image_url}",
        "-e",
        f"FREEBSD_VM_SSH_PORT={args.ssh_port}",
        "-e",
        f"FREEBSD_VM_CPUS={args.cpus}",
        "-e",
        f"FREEBSD_VM_MEM={args.mem}",
        "-e",
        f"FREEBSD_VM_USER={args.user}",
    ]
    disk_size = os.getenv("FREEBSD_VM_DISK_SIZE")
    if disk_size:
        cmd.extend(["-e", f"FREEBSD_VM_DISK_SIZE={disk_size}"])

    if Path("/dev/kvm").exists():
        cmd.extend(["--device", "/dev/kvm"])

    cmd.append(args.image)
    run(cmd)


def stop_container(args):
    if container_exists(args.container):
        run(["docker", "rm", "-f", args.container])


def ssh_key_path(vm_dir):
    return Path(vm_dir) / "id_ed25519"


def ensure_ssh_key(args):
    vm_dir = vm_dir_path(args.vm_dir)
    key_path = ssh_key_path(vm_dir)
    if key_path.exists():
        return
    vm_dir.mkdir(parents=True, exist_ok=True)
    run(["ssh-keygen", "-t", "ed25519", "-f", str(key_path), "-N", ""])


def wait_for_ssh_key(args, timeout=120):
    vm_dir = vm_dir_path(args.vm_dir)
    key_path = ssh_key_path(vm_dir)
    deadline = time.time() + timeout
    while time.time() < deadline:
        if key_path.exists():
            return
        time.sleep(2)
    print(
        f"Timed out waiting for SSH key at {key_path}.",
        file=sys.stderr,
    )
    sys.exit(1)


def wait_for_ssh(args, user, timeout=300):
    vm_dir = vm_dir_path(args.vm_dir)
    key_path = ssh_key_path(vm_dir)
    deadline = time.time() + timeout
    while time.time() < deadline:
        result = subprocess.run(
            [
                "ssh",
                *SSH_OPTIONS,
                "-o",
                "ConnectTimeout=5",
                "-i",
                str(key_path),
                "-p",
                str(args.ssh_port),
                f"{user}@127.0.0.1",
                "true",
            ],
            check=False,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        if result.returncode == 0:
            return
        time.sleep(5)
    print(
        (
            "Timed out waiting for FreeBSD SSH to become ready. "
            "If the VM was started from another checkout or the SSH key changed, "
            "run 'pixi run freebsd-stop' to reset it."
        ),
        file=sys.stderr,
    )
    sys.exit(1)


def ssh_command(args, command, user=None, tty=False):
    vm_dir = vm_dir_path(args.vm_dir)
    key_path = ssh_key_path(vm_dir)
    if not key_path.exists():
        wait_for_ssh_key(args)
    ssh_user = user or args.user
    cmd = [
        "ssh",
        *SSH_OPTIONS,
        "-i",
        str(key_path),
        "-p",
        str(args.ssh_port),
    ]
    if tty:
        cmd.append("-tt")
    cmd.extend([f"{ssh_user}@127.0.0.1", command])
    run(cmd)


def shell_vm(args):
    ensure_started(args)
    vm_dir = vm_dir_path(args.vm_dir)
    key_path = ssh_key_path(vm_dir)
    if not key_path.exists():
        wait_for_ssh_key(args)
    run(
        [
            "ssh",
            *SSH_OPTIONS,
            "-i",
            str(key_path),
            "-p",
            str(args.ssh_port),
            f"{args.user}@127.0.0.1",
        ]
    )


def sync_repo(args):
    ensure_started(args)
    vm_dir = vm_dir_path(args.vm_dir)
    key_path = ssh_key_path(vm_dir)
    if not key_path.exists():
        wait_for_ssh_key(args)

    repo = repo_root()
    remote_dir = args.remote_dir or f"/home/{args.user}/dart"

    excludes = [
        ".git",
        "build",
        ".pixi",
        ".deps",
        ".build",
    ]
    rsync_cmd = [
        "rsync",
        "-az",
        "--delete",
    ]
    for item in excludes:
        rsync_cmd.extend(["--exclude", item])
    rsync_cmd.extend(
        [
            "-e",
            f"ssh {' '.join(SSH_OPTIONS)} -i {key_path} -p {args.ssh_port}",
            f"{repo}/",
            f"{args.user}@127.0.0.1:{remote_dir}/",
        ]
    )
    run(rsync_cmd)


def should_apply_ports_patches():
    value = os.getenv("FREEBSD_VM_APPLY_PORTS_PATCHES", "1")
    return value.lower() not in {"0", "false", "no"}


def apply_ports_patches(args):
    if not should_apply_ports_patches():
        return
    patch_dir = ports_patch_dir()
    patch_files = sorted(patch_dir.glob("patch-*"))
    if not patch_files:
        print(
            f"No FreeBSD ports patches found under {patch_dir}.",
            file=sys.stderr,
        )
        return
    remote_dir = args.remote_dir or f"/home/{args.user}/dart"
    for patch_file in patch_files:
        rel_path = os.path.relpath(patch_file, repo_root())
        command = f"cd {remote_dir} && patch -p0 -N -i {shlex.quote(rel_path)}"
        ssh_command(args, command, user=args.user)


def should_skip_bootstrap():
    value = os.getenv("FREEBSD_VM_SKIP_BOOTSTRAP", "")
    return value.lower() in {"1", "true", "yes"}


def bootstrap_vm(args):
    ensure_started(args)
    if should_skip_bootstrap():
        return
    packages_env = os.getenv("FREEBSD_VM_PACKAGES")
    packages = shlex.split(packages_env) if packages_env else DEFAULT_PACKAGES
    package_list = " ".join(packages)
    root_password = os.getenv("FREEBSD_VM_ROOT_PASSWORD", "freebsd")
    command = (
        "ASSUME_ALWAYS_YES=yes pkg update -f && "
        f"ASSUME_ALWAYS_YES=yes pkg install -y {package_list}"
    )
    su_command = (
        f"printf '%s\\n' {shlex.quote(root_password)} | "
        f"su -m root -c {shlex.quote(command)}"
    )
    ssh_command(args, su_command, user=args.user, tty=True)


def test_vm(args):
    ensure_started(args)
    bootstrap_vm(args)
    sync_repo(args)
    apply_ports_patches(args)

    remote_dir = args.remote_dir or f"/home/{args.user}/dart"
    build_dir = os.getenv("FREEBSD_VM_BUILD_DIR", DEFAULT_BUILD_DIR)
    build_type = os.getenv("FREEBSD_VM_BUILD_TYPE", "Release")
    cmake_args = [
        f"-DCMAKE_BUILD_TYPE={build_type}",
        "-DDART_BUILD_DARTPY=OFF",
        "-DDART_BUILD_GUI_OSG=OFF",
        "-DDART_USE_SYSTEM_GOOGLETEST=ON",
        "-DDART_VERBOSE=ON",
    ]
    cmake_args.extend(shlex.split(os.getenv("FREEBSD_VM_CMAKE_ARGS", "")))
    cmake_arg_str = " ".join(cmake_args)
    build_targets_env = os.getenv("FREEBSD_VM_BUILD_TARGETS")
    if build_targets_env:
        build_targets = shlex.split(build_targets_env)
        if not build_targets:
            build_targets = DEFAULT_BUILD_TARGETS
    else:
        build_targets = DEFAULT_BUILD_TARGETS
    build_targets_str = " ".join(build_targets)
    build_parallel = env_default_int("FREEBSD_VM_BUILD_PARALLEL_LEVEL", 0)
    test_regex = os.getenv("FREEBSD_VM_TEST_REGEX", DEFAULT_TEST_REGEX).strip()
    exclude_regex = os.getenv("FREEBSD_VM_TEST_EXCLUDE_REGEX", "").strip()
    ctest_timeout = env_default_int(
        "FREEBSD_VM_CTEST_TIMEOUT",
        DEFAULT_CTEST_TIMEOUT,
    )
    ctest_parallel = env_default_int("FREEBSD_VM_CTEST_PARALLEL_LEVEL", 0)
    stop_on_failure = os.getenv("FREEBSD_VM_CTEST_STOP_ON_FAILURE", "1").lower()
    stop_on_failure = stop_on_failure not in {"0", "false", "no"}
    extra_ctest_args = shlex.split(os.getenv("FREEBSD_VM_CTEST_ARGS", ""))
    ctest_args = ["--output-on-failure"]
    if ctest_parallel > 0:
        ctest_args.extend(["-j", str(ctest_parallel)])
    if exclude_regex:
        ctest_args.extend(["-E", exclude_regex])
    if ctest_timeout > 0:
        ctest_args.extend(["--timeout", str(ctest_timeout)])
    if stop_on_failure:
        ctest_args.append("--stop-on-failure")
    ctest_args.extend(extra_ctest_args)
    ctest_arg_str = " ".join(shlex.quote(arg) for arg in ctest_args)
    if test_regex:
        test_command = (
            f"ctest --test-dir {build_dir} -R {shlex.quote(test_regex)} {ctest_arg_str}"
        )
    else:
        test_command = f"ctest --test-dir {build_dir} {ctest_arg_str}"

    build_command = f"cmake --build {build_dir} --target {build_targets_str}"
    if build_parallel > 0:
        build_command += f" --parallel {build_parallel}"

    command = (
        f"cd {remote_dir} && "
        f"cmake -G Ninja -S . -B {build_dir} {cmake_arg_str} && "
        f"{build_command} && "
        f"{test_command}"
    )
    try:
        ssh_command(args, command, user=args.user)
    except subprocess.CalledProcessError as exc:
        print(
            "FreeBSD VM build/test failed; ctest stops on the first failing test "
            "and returns a non-zero exit status. See the output above for details.",
            file=sys.stderr,
        )
        raise exc


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Manage FreeBSD VM via Docker. Syncs the local repo into the VM and "
            "builds that copy (pkg installs deps only)."
        )
    )
    parser.add_argument("--image", default=DEFAULT_IMAGE, help="Docker image tag.")
    parser.add_argument(
        "--container", default=DEFAULT_CONTAINER, help="Container name."
    )
    parser.add_argument("--vm-dir", default=str(repo_root() / "build" / "freebsd-vm"))
    parser.add_argument("--ssh-port", type=int, default=DEFAULT_SSH_PORT)
    parser.add_argument(
        "--cpus",
        type=int,
        default=env_default_int("FREEBSD_VM_CPUS", DEFAULT_CPUS),
    )
    parser.add_argument(
        "--mem",
        type=int,
        default=env_default_int("FREEBSD_VM_MEM", DEFAULT_MEM),
    )
    parser.add_argument("--user", default=DEFAULT_USER)
    parser.add_argument("--image-url", default=DEFAULT_IMAGE_URL)
    parser.add_argument("--remote-dir", default=DEFAULT_REMOTE_DIR)

    subparsers = parser.add_subparsers(dest="command", required=True)
    subparsers.add_parser("build-image")
    subparsers.add_parser("start")
    subparsers.add_parser("stop")
    subparsers.add_parser("shell")
    subparsers.add_parser("sync")
    subparsers.add_parser("bootstrap")
    subparsers.add_parser("test")

    return parser.parse_args()


def main():
    args = parse_args()
    if args.command == "build-image":
        build_image(args)
    elif args.command == "start":
        start_container(args)
    elif args.command == "stop":
        stop_container(args)
    elif args.command == "shell":
        shell_vm(args)
    elif args.command == "sync":
        sync_repo(args)
    elif args.command == "bootstrap":
        bootstrap_vm(args)
    elif args.command == "test":
        test_vm(args)


if __name__ == "__main__":
    main()
