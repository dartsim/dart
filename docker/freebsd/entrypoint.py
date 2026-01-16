#!/usr/bin/env python3
import os
import subprocess
import sys
from pathlib import Path

DEFAULT_IMAGE_URL = (
    "https://download.freebsd.org/ftp/snapshots/VM-IMAGES/15.0-STABLE/"
    "amd64/Latest/FreeBSD-15.0-STABLE-amd64-BASIC-CLOUDINIT-ufs.qcow2.xz"
)
DEFAULT_DISK_SIZE = "12G"


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


def is_xz_file(path):
    with open(path, "rb") as handle:
        return handle.read(6) == b"\xfd7zXZ\x00"


def ensure_image(vm_dir, image_url, image_xz, image_qcow2):
    if image_xz.exists() and not is_xz_file(image_xz):
        print(f"Removing invalid FreeBSD archive at {image_xz}...")
        image_xz.unlink()
    if image_qcow2.exists() and image_qcow2.stat().st_size == 0:
        print(f"Removing empty FreeBSD image at {image_qcow2}...")
        image_qcow2.unlink()
    if not image_xz.exists():
        print(f"Downloading FreeBSD image from {image_url}...")
        run(["curl", "-fL", image_url, "-o", str(image_xz)])
    if not image_qcow2.exists():
        print("Extracting FreeBSD image...")
        run(["xz", "-d", "-c", str(image_xz)], stdout_path=image_qcow2)


def ensure_overlay(image_qcow2, overlay_qcow2):
    if overlay_qcow2.exists():
        overlay_qcow2.unlink()
    run(
        [
            "qemu-img",
            "create",
            "-f",
            "qcow2",
            "-b",
            str(image_qcow2),
            "-F",
            "qcow2",
            str(overlay_qcow2),
        ]
    )


def resize_overlay(overlay_qcow2, disk_size):
    if not disk_size:
        return
    run(["qemu-img", "resize", str(overlay_qcow2), disk_size])


def ensure_ssh_key(vm_dir, ssh_key):
    if ssh_key.exists():
        return
    print("Generating SSH key...")
    run(["ssh-keygen", "-t", "ed25519", "-f", str(ssh_key), "-N", ""])


def write_seed(vm_dir, ssh_key, user_data, meta_data, seed_img, user):
    pub_key = ssh_key.with_suffix(".pub").read_text().strip()
    lines = [
        "#cloud-config",
        "ssh_authorized_keys:",
        f"  - {pub_key}",
        "users:",
        "  - default",
    ]
    if user not in {"default", "freebsd", "root"}:
        lines.extend(
            [
                f"  - name: {user}",
                "    groups: wheel",
                "    shell: /bin/sh",
                "    sudo: ALL=(ALL) NOPASSWD:ALL",
                "    ssh_authorized_keys:",
                f"      - {pub_key}",
            ]
        )
    elif user == "root":
        lines.extend(
            [
                "write_files:",
                "  - path: /root/.ssh/authorized_keys",
                f"    content: {pub_key}",
                "    permissions: '0600'",
            ]
        )

    lines.append("")
    user_data.write_text("\n".join(lines))
    meta_data.write_text(
        "\n".join(
            [
                "instance-id: dart-freebsd",
                "local-hostname: dart-freebsd",
                "dsmode: local",
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
    disk_size = os.getenv("FREEBSD_VM_DISK_SIZE", DEFAULT_DISK_SIZE)
    serial_log = os.getenv("FREEBSD_VM_SERIAL_LOG", str(vm_dir / "serial.log"))

    vm_dir.mkdir(parents=True, exist_ok=True)

    image_xz = vm_dir / "FreeBSD.qcow2.xz"
    image_qcow2 = vm_dir / "FreeBSD.qcow2"
    overlay_qcow2 = vm_dir / "FreeBSD-overlay.qcow2"
    seed_img = vm_dir / "seed.img"
    user_data = vm_dir / "user-data"
    meta_data = vm_dir / "meta-data"
    ssh_key = vm_dir / "id_ed25519"

    ensure_image(vm_dir, image_url, image_xz, image_qcow2)
    ensure_overlay(image_qcow2, overlay_qcow2)
    resize_overlay(overlay_qcow2, disk_size)
    ensure_ssh_key(vm_dir, ssh_key)
    user = os.getenv("FREEBSD_VM_USER", "freebsd")
    write_seed(vm_dir, ssh_key, user_data, meta_data, seed_img, user)

    cmd = ["qemu-system-x86_64"]
    kvm_path = Path("/dev/kvm")
    print(
        f"Checking KVM: /dev/kvm exists={kvm_path.exists()}",
        file=sys.stderr,
        flush=True,
    )
    kvm_usable = False
    if kvm_path.exists():
        # Check if we can actually access KVM
        try:
            kvm_readable = os.access("/dev/kvm", os.R_OK)
            kvm_writable = os.access("/dev/kvm", os.W_OK)
            print(
                f"KVM permissions: readable={kvm_readable}, writable={kvm_writable}",
                file=sys.stderr,
                flush=True,
            )
            kvm_usable = kvm_readable and kvm_writable
        except Exception as e:
            print(f"KVM access check failed: {e}", file=sys.stderr, flush=True)

    # Use Penryn CPU model (SSE4.1, no SSE4.2/AVX) for maximum compatibility.
    # GitHub Actions runs in nested virtualization which can cause issues
    # with certain SIMD instructions even when KVM is enabled.
    # Westmere (SSE4.2) still caused SIGILL in some tests, so we use Penryn.
    # See: https://gitlab.com/qemu-project/qemu/-/issues/361
    if kvm_usable:
        print(
            "KVM available - using Penryn CPU for nested virt compatibility",
            file=sys.stderr,
            flush=True,
        )
        cmd.extend(["-enable-kvm", "-cpu", "Penryn"])
    else:
        print(
            "KVM not available - using Penryn CPU with TCG emulation",
            file=sys.stderr,
            flush=True,
        )
        cmd.extend(["-cpu", "Penryn"])
    print(f"QEMU command: {' '.join(cmd[:10])}...", file=sys.stderr, flush=True)
    cmd += [
        "-m",
        str(mem),
        "-smp",
        str(cpus),
        "-drive",
        f"file={overlay_qcow2},if=virtio,format=qcow2",
        "-drive",
        f"file={seed_img},media=cdrom,format=raw",
        "-netdev",
        f"user,id=net0,hostfwd=tcp:0.0.0.0:{ssh_port}-:22",
        "-device",
        "virtio-net-pci,netdev=net0",
        "-display",
        "none",
    ]
    if serial_log:
        cmd.extend(["-serial", f"file:{serial_log}"])
    os.execvp(cmd[0], cmd)


if __name__ == "__main__":
    main()
