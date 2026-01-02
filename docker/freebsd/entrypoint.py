#!/usr/bin/env python3
import os
import subprocess
import sys
from pathlib import Path


DEFAULT_IMAGE_URL = (
    "https://download.freebsd.org/ftp/snapshots/VM-IMAGES/15.0-STABLE/"
    "amd64/Latest/FreeBSD-15.0-STABLE-amd64.qcow2.xz"
)


def getenv_int(name, default):
    value = os.getenv(name)
    if value is None or value == "":
        return default
    try:
        return int(value)
    except ValueError:
        print(f"Invalid {name}={value}, expected integer.", file=sys.stderr)
        sys.exit(1)


def run(cmd, stdout_path=None):
    if stdout_path is None:
        subprocess.run(cmd, check=True)
        return
    with open(stdout_path, "wb") as handle:
        subprocess.run(cmd, check=True, stdout=handle)


def ensure_image(vm_dir, image_url, image_xz, image_qcow2):
    if not image_xz.exists():
        print(f"Downloading FreeBSD image from {image_url}...")
        run(["curl", "-L", image_url, "-o", str(image_xz)])
    if not image_qcow2.exists():
        print("Extracting FreeBSD image...")
        run(["xz", "-d", "-c", str(image_xz)], stdout_path=image_qcow2)


def ensure_ssh_key(vm_dir, ssh_key):
    if ssh_key.exists():
        return
    print("Generating SSH key...")
    run(["ssh-keygen", "-t", "ed25519", "-f", str(ssh_key), "-N", ""])


def write_seed(vm_dir, ssh_key, user_data, meta_data, seed_img, user):
    pub_key = ssh_key.with_suffix(".pub").read_text().strip()
    user_data.write_text(
        "\n".join(
            [
                "#cloud-config",
                "users:",
                f"  - name: {user}",
                "    shell: /bin/sh",
                "    ssh-authorized-keys:",
                f"      - {pub_key}",
                "",
            ]
        )
    )
    meta_data.write_text(
        "\n".join(
            [
                "instance-id: dart-freebsd",
                "local-hostname: dart-freebsd",
                "",
            ]
        )
    )
    run(["cloud-localds", str(seed_img), str(user_data), str(meta_data)])


def main():
    vm_dir = Path(os.getenv("FREEBSD_VM_DIR", "/vm"))
    image_url = os.getenv("FREEBSD_VM_IMAGE_URL", DEFAULT_IMAGE_URL)
    ssh_port = getenv_int("FREEBSD_VM_SSH_PORT", 10022)
    cpus = getenv_int("FREEBSD_VM_CPUS", 4)
    mem = getenv_int("FREEBSD_VM_MEM", 4096)
    user = os.getenv("FREEBSD_VM_USER", "freebsd")

    vm_dir.mkdir(parents=True, exist_ok=True)

    image_xz = vm_dir / "FreeBSD.qcow2.xz"
    image_qcow2 = vm_dir / "FreeBSD.qcow2"
    seed_img = vm_dir / "seed.img"
    user_data = vm_dir / "user-data"
    meta_data = vm_dir / "meta-data"
    ssh_key = vm_dir / "id_ed25519"

    ensure_image(vm_dir, image_url, image_xz, image_qcow2)
    ensure_ssh_key(vm_dir, ssh_key)
    write_seed(vm_dir, ssh_key, user_data, meta_data, seed_img, user)

    cmd = ["qemu-system-x86_64"]
    if Path("/dev/kvm").exists():
        cmd.append("-enable-kvm")
    cmd += [
        "-m",
        str(mem),
        "-smp",
        str(cpus),
        "-drive",
        f"file={image_qcow2},if=virtio",
        "-drive",
        f"file={seed_img},if=virtio,format=raw",
        "-netdev",
        f"user,id=net0,hostfwd=tcp::{ssh_port}-:22",
        "-device",
        "virtio-net-pci,netdev=net0",
        "-display",
        "none",
    ]
    os.execvp(cmd[0], cmd)


if __name__ == "__main__":
    main()
