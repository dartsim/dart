#!/usr/bin/env python3
"""
Build DART Docker images locally for testing.

Usage:
    python scripts/build_docker.py all              # Build all Ubuntu versions
    python scripts/build_docker.py ubuntu jammy     # Build specific Ubuntu distro
    python scripts/build_docker.py ubuntu noble     # Build specific Ubuntu distro
    python scripts/build_docker.py ubuntu questing  # Build specific Ubuntu distro
    python scripts/build_docker.py manylinux        # Build manylinux image
"""

import argparse
import shutil
import subprocess
import sys
from pathlib import Path

# Ubuntu version mapping: version number -> codename
UBUNTU_VERSION_MAP = {
    "22.04": "jammy",
    "24.04": "noble",
    "25.04": "plucky",
    "25.10": "questing",
}

# Reverse mapping: codename -> version number (for display)
UBUNTU_CODENAME_MAP = {v: k for k, v in UBUNTU_VERSION_MAP.items()}


def resolve_ubuntu_distro(distro: str) -> str:
    """
    Resolve Ubuntu distro to codename.
    Accepts either version number (e.g., '22.04') or codename (e.g., 'jammy').
    Returns the codename.
    """
    # If it's a version number, convert to codename
    if distro in UBUNTU_VERSION_MAP:
        return UBUNTU_VERSION_MAP[distro]
    # If it's already a codename, return as-is
    elif distro in UBUNTU_CODENAME_MAP:
        return distro
    else:
        # Try to provide helpful error message
        valid_options = []
        for version, codename in UBUNTU_VERSION_MAP.items():
            valid_options.append(f"{version} ({codename})")
        raise ValueError(
            f"Unknown Ubuntu distro: {distro}\n"
            f"Valid options: {', '.join(valid_options)}"
        )


def get_container_tool():
    """Check for available container tool (docker or podman) and return the command."""
    if shutil.which("docker"):
        return "docker"
    elif shutil.which("podman"):
        return "podman"
    else:
        print("❌ Error: Neither Docker nor Podman is installed or in PATH")
        print()
        print("A container tool is required to build images locally.")
        print()
        print("Option 1 - Install Podman (recommended for Fedora/RHEL):")
        print("  Fedora/RHEL:")
        print("    sudo dnf install podman")
        print()
        print("  Or install podman-docker for 'docker' command compatibility:")
        print("    sudo dnf install podman-docker")
        print()
        print("Option 2 - Install Docker:")
        print("  Fedora/RHEL:")
        print("    sudo dnf install docker")
        print("    sudo systemctl start docker")
        print("    sudo systemctl enable docker")
        print()
        print("  Ubuntu/Debian:")
        print("    sudo apt install docker.io")
        print("    sudo systemctl start docker")
        print("    sudo systemctl enable docker")
        print()
        print("  macOS:")
        print(
            "    Install Docker Desktop from https://www.docker.com/products/docker-desktop"
        )
        print()
        print(
            "After Docker installation, you may need to add your user to the docker group:"
        )
        print("    sudo usermod -aG docker $USER")
        print("    (then log out and back in)")
        return None


def build_ubuntu(distro: str, container_tool: str, dart_version: str = "v6.15"):
    """Build Ubuntu Docker image for a specific distro."""
    # Resolve distro to codename (supports both version numbers and codenames)
    try:
        codename = resolve_ubuntu_distro(distro)
        version = UBUNTU_CODENAME_MAP.get(codename, distro)
        print(f"ℹ️  Building Ubuntu {version} ({codename})")
    except ValueError as e:
        print(f"❌ {e}")
        return False

    dockerfile = Path(f"docker/dev/{dart_version}/Dockerfile.ubuntu.{codename}")
    if not dockerfile.exists():
        print(f"❌ Error: Dockerfile not found: {dockerfile}")
        return False

    tag = f"dart-dev:ubuntu-{codename}-{dart_version}"
    print(f"🔨 Building {tag}...")

    cmd = [
        container_tool,
        "build",  # Use 'build' instead of 'buildx build' for podman compatibility
        "--file",
        str(dockerfile),
        "--tag",
        tag,
        ".",
    ]

    try:
        result = subprocess.run(cmd, check=True)
        print(f"✅ Successfully built {tag}")
        return result.returncode == 0
    except subprocess.CalledProcessError:
        print(f"❌ Failed to build {tag}")
        return False


def build_all_ubuntu(container_tool: str, dart_version: str = "v6.15"):
    """Build all Ubuntu Docker images."""
    distros = ["jammy", "noble", "questing"]
    success = True

    for distro in distros:
        if not build_ubuntu(distro, container_tool, dart_version):
            success = False

    return success


def build_manylinux(container_tool: str, dart_version: str = "v6.15"):
    """Build manylinux Docker image."""
    dockerfile = Path(f"docker/dev/{dart_version}/Dockerfile.manylinux_2_28_x86_64")
    if not dockerfile.exists():
        print(f"❌ Error: Dockerfile not found: {dockerfile}")
        return False

    tag = f"dart-dev:manylinux_2_28_x86_64-{dart_version}"
    print(f"🔨 Building {tag}...")

    cmd = [
        container_tool,
        "build",  # Use 'build' instead of 'buildx build' for podman compatibility
        "--file",
        str(dockerfile),
        "--build-arg",
        "BASE_IMAGE=quay.io/pypa/manylinux_2_28_x86_64",
        "--tag",
        tag,
        ".",
    ]

    try:
        result = subprocess.run(cmd, check=True)
        print(f"✅ Successfully built {tag}")
        return result.returncode == 0
    except subprocess.CalledProcessError:
        print(f"❌ Failed to build {tag}")
        return False


def main():
    parser = argparse.ArgumentParser(
        description="Build DART Docker images locally for testing",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s all                  # Build all Ubuntu versions
  %(prog)s ubuntu 22.04         # Build Ubuntu 22.04 (Jammy)
  %(prog)s ubuntu jammy         # Build Ubuntu 22.04 (Jammy) - same as above
  %(prog)s ubuntu 24.04         # Build Ubuntu 24.04 (Noble)
  %(prog)s ubuntu 25.10         # Build Ubuntu 25.10 (Questing)
  %(prog)s manylinux            # Build manylinux image
        """,
    )

    parser.add_argument(
        "target",
        choices=["all", "ubuntu", "manylinux"],
        help="What to build: all (all Ubuntu versions), ubuntu (specific distro), or manylinux",
    )

    parser.add_argument(
        "distro",
        nargs="?",
        help="Ubuntu distro to build (required when target is 'ubuntu'). "
        "Can be either codename (jammy, noble, questing) or version (22.04, 24.04, 25.10)",
    )

    parser.add_argument(
        "--dart-version",
        default="v6.15",
        help="DART version to build (default: v6.15)",
    )

    args = parser.parse_args()

    # Check for available container tool (docker or podman)
    container_tool = get_container_tool()
    if not container_tool:
        return 1

    print(f"ℹ️  Using container tool: {container_tool}")
    print()

    # Validate arguments
    if args.target == "ubuntu" and not args.distro:
        parser.error("distro is required when target is 'ubuntu'")

    # Build based on target
    success = False
    if args.target == "all":
        print("🔨 Building all Ubuntu Docker images...")
        success = build_all_ubuntu(container_tool, args.dart_version)
    elif args.target == "ubuntu":
        success = build_ubuntu(args.distro, container_tool, args.dart_version)
    elif args.target == "manylinux":
        success = build_manylinux(container_tool, args.dart_version)

    return 0 if success else 1


if __name__ == "__main__":
    sys.exit(main())
