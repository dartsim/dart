#!/usr/bin/env python3
# Copyright (c) 2011, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/main/LICENSE
#
# This file is provided under the following "BSD-style" License:
#   Redistribution and use in source and binary forms, with or
#   without modification, are permitted provided that the following
#   conditions are met:
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
#   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
#   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
#   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
#   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
#   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
#   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
#   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
#   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
#   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#   POSSIBILITY OF SUCH DAMAGE.


"""External-baseline availability probe for the Kamino solver.

Kamino is the ADMM-based rigid-multibody solver shipped inside NVIDIA's
open-source Newton physics engine (``newton.solvers.SolverKamino``,
``pip install newton``, Apache-2.0; companion paper arXiv:2603.16536). It is
one of the baselines named by the SCA 2026 exact-Coulomb FBF paper
("NVIDIA Warp/Newton, Kamino").

This script is a benchmark/example-only AVAILABILITY PROBE, not a wired
paper-parity comparison:

- It lazily imports the optional ``warp`` and ``newton`` pypi packages and
  exits 0 with a one-line skip note when they are missing, so no default
  build or CTest gate depends on them (see pixi.toml's optional
  ``fbf-baselines`` feature/environment).
- Newton/Kamino's contact kernels target GPU-scale parallelism, so a
  hardware-comparable timing row needs a CUDA device; the probe records the
  GPU name/driver/toolkit when one is present and skips cleanly when not
  (CI cannot assume a CUDA device).
- Building FBF paper-parity scenes (incline, backspin, card house, arch)
  inside Newton with matched materials is an OPEN INTEGRATION TASK; this
  probe intentionally does not attempt it. Its CSV row only documents
  availability plus hardware metadata so the paper-parity reports can cite
  the current state honestly.
"""

import sys


def emit(scene, source, notes):
    print("scene,source,step,time,notes")
    print(f"{scene},{source},0,0,{notes}")


def main():
    try:
        import warp as wp
    except Exception as error:  # noqa: BLE001 - availability probe
        emit("kamino_probe", "kamino", f"skipped: warp-lang not importable ({error})")
        return 0

    try:
        import newton
        from newton.solvers import SolverKamino  # noqa: F401 - probe import
    except Exception as error:  # noqa: BLE001 - availability probe
        emit("kamino_probe", "kamino", f"skipped: newton/SolverKamino not importable ({error})")
        return 0

    try:
        wp.init()
        devices = [str(device) for device in wp.get_devices()]
    except Exception as error:  # noqa: BLE001 - availability probe
        emit(
            "kamino_probe",
            "kamino",
            f"skipped: warp runtime initialization failed ({error})",
        )
        return 0

    cuda_devices = [d for d in devices if d.startswith("cuda")]
    newton_version = getattr(newton, "__version__", "unknown")
    warp_version = getattr(wp.config, "version", "unknown")

    if not cuda_devices:
        emit(
            "kamino_probe",
            "kamino",
            "skipped: SolverKamino importable "
            f"(newton {newton_version}; warp {warp_version}) but no CUDA "
            "device is present; a hardware-comparable Kamino row needs one",
        )
        return 0

    device = wp.get_device(cuda_devices[0])
    name = getattr(device, "name", cuda_devices[0])
    emit(
        "kamino_probe",
        "kamino",
        f"available: SolverKamino importable (newton {newton_version}; warp "
        f"{warp_version}) on {name}; paper-parity Newton scenes are an open "
        "integration task",
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
