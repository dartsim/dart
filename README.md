# DART

<p align="center">
  <img src="https://raw.githubusercontent.com/dartsim/dart/main/docs/dart_logo_377x107.jpg" alt="DART: Dynamic Animation and Robotics Toolkit">
</p>

<p align="center">
  <a href="https://github.com/dartsim/dart/actions/workflows/ci_ubuntu.yml"><img src="https://github.com/dartsim/dart/actions/workflows/ci_ubuntu.yml/badge.svg" alt="CI Ubuntu"></a>
  <a href="https://github.com/dartsim/dart/actions/workflows/ci_macos.yml"><img src="https://github.com/dartsim/dart/actions/workflows/ci_macos.yml/badge.svg" alt="CI macOS"></a>
  <a href="https://github.com/dartsim/dart/actions/workflows/ci_windows.yml"><img src="https://github.com/dartsim/dart/actions/workflows/ci_windows.yml/badge.svg" alt="CI Windows"></a>
  <br>
  <a href="https://dart.readthedocs.io/en/latest/?badge=latest"><img src="https://readthedocs.org/projects/dart/badge/?version=latest" alt="Documentation Status"></a>
  <a href="https://deepwiki.com/dartsim/dart"><img src="https://deepwiki.com/badge.svg" alt="Ask DeepWiki"></a>
  <a href="https://codecov.io/gh/dartsim/dart"><img src="https://codecov.io/gh/dartsim/dart/branch/main/graph/badge.svg" alt="codecov"></a>
  <a href="https://app.codacy.com/gh/dartsim/dart/dashboard?utm_source=github.com&utm_medium=referral&utm_content=dartsim/dart&utm_campaign=Badge_Grade"><img src="https://app.codacy.com/project/badge/Grade/2d95a9b951be4b73a71097670ec351e8" alt="Codacy Badge"></a>
  <br>
  <a href="https://anaconda.org/conda-forge/dartsim"><img src="https://anaconda.org/conda-forge/dartsim/badges/version.svg" alt="Anaconda-Server Badge"></a>
  <a href="https://pypi.org/project/dartpy/"><img src="https://img.shields.io/pypi/v/dartpy" alt="PyPI Version"></a>
  <a href="https://github.com/dartsim/dart/blob/main/LICENSE"><img src="https://img.shields.io/badge/License-BSD_2--Clause-blue.svg" alt="License"></a>
</p>

DART (Dynamic Animation and Robotics Toolkit) is an open-source,
research-focused physics engine for robotics, animation, and machine learning.
It provides transparent kinematics, dynamics, collision, and constraint-solving
foundations for users who need more than a black-box simulator. DART uses
generalized coordinates for articulated rigid body systems and Featherstone's
Articulated Body Algorithm for accurate, stable motion dynamics.

> [!IMPORTANT]
> **Which DART is this?** This `main` branch tracks **DART 7**, an in-progress
> redesign that is **not yet recommended for production use**. Its documentation
> is the [`latest`](https://dart.readthedocs.io/en/latest/) site.
>
> For a stable release, use **DART 6 LTS**:
> [documentation](https://dart.readthedocs.io/en/stable/) ·
> [`release-6.20` branch](https://github.com/dartsim/dart/tree/release-6.20).

<p align="center">
  <img src="docs/assets/unitree_g1_demo.gif" width="720" alt="Unitree G1 humanoid demo" />
</p>

## Why DART?

- **Research-grade dynamics** — Featherstone algorithms, generalized coordinates, and direct access to dynamics quantities
- **Easy to start** — Python and C++ packages through common package managers, plus reproducible source builds with pixi
- **Extensible foundations** — Math, native collision, constraints, model loading, benchmarks, and tests that support new algorithms and baseline comparisons
- **Unified model loading** — Load URDF, SDF, and MJCF models through a single API
- **Scalable compute roadmap** — Cross-platform CPU support today, with roadmap work for multi-core, SIMD, and accelerator backends
- **Battle-tested ecosystem** — Powers [Gazebo](https://gazebosim.org), research labs, and production systems worldwide, with best-effort support for production use

## Quick Start

**Python**

```python
import dartpy as dart
import numpy as np

world = dart.World(time_step=1.0 / 1000.0)

# Add a 1 kg rigid body one metre above the origin.
options = dart.RigidBodyOptions()
options.mass = 1.0
options.position = np.array([0.0, 0.0, 1.0])
box = world.add_rigid_body("box", options)

# Lock topology, then simulate 100 steps.
world.enter_simulation_mode()
for _ in range(100):
    world.step()

print(f"t = {world.time:.3f} s, box z = {float(box.translation[2]):.3f} m")
```

**C++**

The DART 7 C++ simulation facade is available in source builds while the final
public header transaction is still in progress. Use the C++ API shape recorded
in [`docs/design/simulation_cpp_api.md`](docs/design/simulation_cpp_api.md)
for current source-checkout examples; DART 6 C++ snippets remain on the
`release-6.*` branches for compatibility-line users.

## Installation

The quick-start snippets above target the current `main` branch and DART 7 API.
The tracked DART 7 wheel lane builds CPython 3.14 wheels on Linux, macOS, and
Windows, but PyPI currently serves the stable DART 6 line as the latest
non-yanked `dartpy` package. Use the source checkout path for the DART 7 Python
facade until a non-yanked DART 7 wheel is published.

### Python (Recommended)

| Method                                  | Command                                                         |
| --------------------------------------- | --------------------------------------------------------------- |
| **source checkout** (DART 7 today)      | `pixi run build` from this repository                           |
| **uv/pip** (DART 7 after wheel publish) | `uv add dartpy --prerelease allow` / `pip install --pre dartpy` |
| **pixi** (DART 6 stable today)          | `pixi add dartpy`                                               |
| **conda** (DART 6 stable today)         | `conda install -c conda-forge dartpy`                           |

### C++

| Platform                         | Command                                                              |
| -------------------------------- | -------------------------------------------------------------------- |
| **Cross-platform** (recommended) | `pixi add dartsim-cpp` or `conda install -c conda-forge dartsim-cpp` |
| Ubuntu                           | `sudo apt install libdart-all-dev`                                   |
| Arch Linux                       | `yay -S libdart`                                                     |
| FreeBSD                          | `pkg install dartsim`                                                |
| macOS                            | `brew install dartsim`                                               |
| Windows                          | `vcpkg install dartsim:x64-windows`                                  |

[All distributions →](https://repology.org/project/dart-sim/versions)

### Current DART 7 Smoke Checks

These snippets create a tiny model in code, so they do not depend on sample data
files being present. The Python snippet targets a DART 7 source build; if
`dart.World` or `add_rigid_body` is missing, the environment resolved a stable
DART 6 package and should use the stable documentation.

**Python source build**

Run this after `pixi run build` from a source checkout:

```bash
PYTHONPATH=build/default/cpp/Release/python python - <<'PY'
import dartpy as dart

world = dart.World()
body = world.add_rigid_body("box", dart.RigidBodyOptions())
world.enter_simulation_mode()
world.step()
print(f"t = {world.time:.4f} s, box position = {body.translation}")
PY
```

**C++ package**

The published C++ package smoke for the compatibility line is maintained on
`release-6.*`. DART 7 C++ package smoke checks move through
`check-dart7-artifacts` and the local source-checkout promotion gates until
DART 7 C++ artifacts are published.

### Source checkout

```bash
pixi install
# Smoke the Python-first DART 7 rigid-body demo surface.
pixi run py-demos -- --scene rigid_body --headless --frames 1

# Open the curated rigid-body GUI verifier.
pixi run py-demos
pixi run py-demos -- --scene rigid_solver_compare

# Capture docked visual evidence for the solver comparison.
pixi run py-demo-capture -- --scene rigid_solver_compare --frames 24 --width 960 --height 540 --show-ui

# Optional C++ World-only companion smoke.
pixi run demos -- --scene rigid_body --headless --frames 1
```

The full 36-row rigid-body visual verification workflow, `Rigid Workflow`
search terms, and capture commands are in the
[Python demos README](python/examples/demos/README.md#rigid-body-visual-verification-workflow).

## Documentation

- **Static Docs**: [English](https://dart.readthedocs.io/) | [한국어](https://dart-ko.readthedocs.io/)
- **AI Docs (experimental)**: [DeepWiki](https://deepwiki.com/dartsim/dart) for quick codebase Q&A | [NotebookLM](https://notebooklm.google.com/notebook/c0cfc8ce-17ae-415a-a615-44c4342f0da6) (Google account required)
- **Questions**: Ask in [GitHub Discussions](https://github.com/dartsim/dart/discussions) when you want maintainer or community input.

### Developer Resources

- **[Developer Onboarding Guide](docs/onboarding/README.md)** — Architecture, components, and workflows
- **[Contributing Guide](CONTRIBUTING.md)** — Style guide and contribution process
- **[Project Direction](docs/ai/north-star.md)** — Research-focused north star and priorities
- **[Living Plans](docs/plans/README.md)** — Active and proposed roadmap items
- **[Background Theory](docs/background/README.md)** — Dynamics, contact solving, and mathematical foundations
- **[Gazebo Integration](docs/onboarding/build-system.md#gazebo-integration-feature)** — gz-physics integration workflow
- **[Release Roadmap](docs/onboarding/release-roadmap.md)** — Compatibility, deprecations, and future plans

#### Branches

- `main` — Active development targeting DART 7 (not yet production-ready)
- `release-6.20` — Maintenance branch for DART 6 LTS (critical fixes only); see
  the [DART 6 documentation](https://dart.readthedocs.io/en/stable/)

## Citation

If you use DART in an academic publication, please consider citing this [JOSS Paper](https://doi.org/10.21105/joss.00500):

```bibtex
@article{Lee2018,
  doi = {10.21105/joss.00500},
  url = {https://doi.org/10.21105/joss.00500},
  year = {2018},
  publisher = {The Open Journal},
  volume = {3},
  number = {22},
  pages = {500},
  author = {Jeongseok Lee and Michael X. Grey and Sehoon Ha and Tobias Kunz and Sumit Jain and Yuting Ye and Siddhartha S. Srinivasa and Mike Stilman and C. Karen Liu},
  title = {DART: Dynamic Animation and Robotics Toolkit},
  journal = {Journal of Open Source Software}
}
```

## License

DART is licensed under the [BSD 2-Clause License](LICENSE).

## Star History

<a href="https://www.star-history.com/?repos=dartsim%2Fdart&type=date&legend=top-left">
 <picture>
   <source media="(prefers-color-scheme: dark)" srcset="https://api.star-history.com/chart?repos=dartsim/dart&type=date&theme=dark&legend=top-left" />
   <source media="(prefers-color-scheme: light)" srcset="https://api.star-history.com/chart?repos=dartsim/dart&type=date&legend=top-left" />
   <img alt="Star History Chart" src="https://api.star-history.com/chart?repos=dartsim/dart&type=date&legend=top-left" />
 </picture>
</a>
