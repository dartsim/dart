#!/usr/bin/env python3
"""Test a repaired Linux wheel's TLS trust on Ubuntu 24.04 and Debian 12."""

from __future__ import annotations

import argparse
import subprocess
from pathlib import Path


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("wheel", type=Path)
    args = parser.parse_args()
    wheel = args.wheel.resolve(strict=True)
    root = Path(__file__).resolve().parents[1]
    for distro in ("ubuntu:24.04", "debian:12"):
        image = "dartpy-tls-" + distro.replace(":", "-")
        subprocess.run(
            [
                "docker",
                "build",
                "--build-arg",
                f"BASE_IMAGE={distro}",
                "-f",
                str(root / "docker/wheels/Dockerfile.tls-smoke"),
                "-t",
                image,
                str(root / "docker/wheels"),
            ],
            check=True,
        )
        print(f"TLS smoke: {distro}", flush=True)
        subprocess.run(
            [
                "docker",
                "run",
                "--rm",
                "-v",
                f"{root}:/io:ro",
                "-v",
                f"{wheel.parent}:/w:ro",
                image,
                "sh",
                "-ec",
                'python -m pip install --disable-pip-version-check "$1" && '
                "python -I -u /io/scripts/test_wheel_tls.py --test-ca-fallback",
                "tls-smoke",
                f"/w/{wheel.name}",
            ],
            check=True,
        )


if __name__ == "__main__":
    main()
