#!/usr/bin/env python3
import argparse
import os
import subprocess
import sys
from pathlib import Path


DEFAULT_IMAGE = "dartsim/freebsd-vm:15"
DEFAULT_CONTAINER = "dart-freebsd-vm"
DEFAULT_SSH_PORT = 10022
DEFAULT_CPUS = 4
DEFAULT_MEM = 4096
DEFAULT_USER = "freebsd"
DEFAULT_REMOTE_DIR = None
DEFAULT_IMAGE_URL = (
    "https://download.freebsd.org/ftp/snapshots/VM-IMAGES/15.0-STABLE/"
    "amd64/Latest/FreeBSD-15.0-STABLE-amd64.qcow2.xz"
)


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


def vm_dir_path(default_dir):
    return Path(os.getenv("FREEBSD_VM_DIR", default_dir)).resolve()


def image_exists(image):
    result = run(["docker", "image", "inspect", image], check=False)
    return result.returncode == 0


def container_exists(container):
    result = run(
        ["docker", "ps", "-a", "--filter", f"name={container}", "--format", "{{.Names}}"],
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

    if container_running(args.container):
        print(f"Container '{args.container}' already running.")
        return

    if container_exists(args.container):
        run(["docker", "start", args.container])
        return

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

    if Path("/dev/kvm").exists():
        cmd.extend(["--device", "/dev/kvm"])

    cmd.append(args.image)
    run(cmd)


def stop_container(args):
    if container_exists(args.container):
        run(["docker", "rm", "-f", args.container])


def ssh_key_path(vm_dir):
    return Path(vm_dir) / "id_ed25519"


def shell_vm(args):
    vm_dir = vm_dir_path(args.vm_dir)
    key_path = ssh_key_path(vm_dir)
    if not key_path.exists():
        print(f"Missing SSH key at {key_path}. Start the VM first.", file=sys.stderr)
        sys.exit(1)
    run(
        [
            "ssh",
            "-i",
            str(key_path),
            "-p",
            str(args.ssh_port),
            f"{args.user}@127.0.0.1",
        ]
    )


def sync_repo(args):
    vm_dir = vm_dir_path(args.vm_dir)
    key_path = ssh_key_path(vm_dir)
    if not key_path.exists():
        print(f"Missing SSH key at {key_path}. Start the VM first.", file=sys.stderr)
        sys.exit(1)

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
    ]
    for item in excludes:
        rsync_cmd.extend(["--exclude", item])
    rsync_cmd.extend(
        [
            "-e",
            f"ssh -i {key_path} -p {args.ssh_port}",
            f"{repo}/",
            f"{args.user}@127.0.0.1:{remote_dir}/",
        ]
    )
    run(rsync_cmd)


def parse_args():
    parser = argparse.ArgumentParser(description="Manage FreeBSD VM via Docker.")
    parser.add_argument("--image", default=DEFAULT_IMAGE, help="Docker image tag.")
    parser.add_argument("--container", default=DEFAULT_CONTAINER, help="Container name.")
    parser.add_argument("--vm-dir", default=str(repo_root() / "build" / "freebsd-vm"))
    parser.add_argument("--ssh-port", type=int, default=DEFAULT_SSH_PORT)
    parser.add_argument("--cpus", type=int, default=DEFAULT_CPUS)
    parser.add_argument("--mem", type=int, default=DEFAULT_MEM)
    parser.add_argument("--user", default=DEFAULT_USER)
    parser.add_argument("--image-url", default=DEFAULT_IMAGE_URL)
    parser.add_argument("--remote-dir", default=DEFAULT_REMOTE_DIR)

    subparsers = parser.add_subparsers(dest="command", required=True)
    subparsers.add_parser("build-image")
    subparsers.add_parser("start")
    subparsers.add_parser("stop")
    subparsers.add_parser("shell")
    subparsers.add_parser("sync")

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


if __name__ == "__main__":
    main()
