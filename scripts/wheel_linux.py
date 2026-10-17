#!/usr/bin/env python3
"""Build and test Linux wheels with the manylinux 2.28 native toolchain."""

from __future__ import annotations

import argparse
import platform
import subprocess
from pathlib import Path


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("python_tag", choices=["cp310", "cp311", "cp312", "cp313"])
    args = parser.parse_args()
    arch = platform.machine()
    if arch not in {"x86_64", "aarch64"}:
        parser.error(f"Unsupported wheel architecture: {arch}")

    root = Path(__file__).resolve().parents[1]
    dist = root / "dist"
    dist.mkdir(exist_ok=True)
    base = f"quay.io/pypa/manylinux_2_28_{arch}"
    image = f"dartpy-manylinux-2-28-{arch}"
    subprocess.run(
        [
            "docker",
            "build",
            "--build-arg",
            f"BASE_IMAGE={base}",
            "-f",
            str(root / "docker/wheels/Dockerfile.manylinux_2_28"),
            "-t",
            image,
            str(root / "docker/wheels"),
        ],
        check=True,
    )
    python = f"/opt/python/{args.python_tag}-{args.python_tag}/bin/python"
    # wheel_build.py builds the CPython 3.12 wheel for the stable ABI (abi3).
    abi3 = args.python_tag == "cp312"
    # Keep builds out of the checkout, including stale host CMake caches.
    command = f"""
set -euo pipefail
mkdir /tmp/dart
tar -C /io --exclude=.git --exclude=.pixi --exclude=build --exclude=dist \\
    --exclude=__pycache__ --exclude='*.egg-info' -cf - . | tar -C /tmp/dart -xf -
cd /tmp/dart
export PATH=/opt/python/{args.python_tag}-{args.python_tag}/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/lib:/usr/local/lib64:${{LD_LIBRARY_PATH:-}}
{python} -m pip install 'setuptools>=80.1.0,<81' 'wheel>=0.48.0,<0.49' ninja
CONDA_PREFIX=/usr/local {python} scripts/wheel_build.py OFF
auditwheel show dist/dartpy-*-linux_{arch}.whl
auditwheel repair --plat manylinux_2_28_{arch} --only-plat \\
    dist/dartpy-*-linux_{arch}.whl -w /output
{python} scripts/verify_wheel.py /output/dartpy-*-{args.python_tag}-*.whl
"""
    if abi3:
        command += f"""{python} -m pip install abi3audit==0.0.26
abi3audit --strict --report /output/dartpy-*-cp312-abi3-*.whl
"""
    subprocess.run(
        [
            "docker",
            "run",
            "--rm",
            "-v",
            f"{root}:/io:ro",
            "-v",
            f"{dist}:/output",
            image,
            "bash",
            "-c",
            command,
        ],
        check=True,
    )
    # A fresh base image has none of our source-built dependencies installed.
    # The abi3 wheel also serves newer interpreters.
    for tag in (args.python_tag, "cp313") if abi3 else (args.python_tag,):
        subprocess.run(
            [
                "docker",
                "run",
                "--rm",
                "-v",
                f"{root}:/io:ro",
                base,
                "env",
                "-u",
                "LD_LIBRARY_PATH",
                "-u",
                "PYTHONPATH",
                f"/opt/python/{tag}-{tag}/bin/python",
                "/io/scripts/test_wheel.py",
                f"/io/dist/dartpy-*-{args.python_tag}-*-manylinux_2_28_{arch}.whl",
            ],
            check=True,
        )


if __name__ == "__main__":
    main()
