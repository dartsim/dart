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

<p align="center">
  <img src="docs/assets/unitree_g1_demo.gif" width="720" alt="Unitree G1 humanoid demo" />
</p>

## Why DART?

- **Research-grade dynamics** — Featherstone algorithms, generalized coordinates, and direct access to dynamics quantities
- **Easy to start** — Python and C++ packages through common package managers, plus reproducible source builds with pixi
- **Extensible foundations** — Math, native collision, constraints, model loading, benchmarks, and tests that support new algorithms and baseline comparisons
- **Unified model loading** — Load URDF, SDF, MJCF, and SKEL models through a single API
- **Scalable compute roadmap** — Cross-platform CPU support today, with roadmap work for multi-core, SIMD, and accelerator backends
- **Battle-tested ecosystem** — Powers [Gazebo](https://gazebosim.org), research labs, and production systems worldwide, with best-effort support for production use

## Quick Start

**Python**

```python
import dartpy as dart

world = dart.World()

# Load a robot from URDF
urdf = dart.io.UrdfParser()
robot = urdf.parse_skeleton("dart://sample/urdf/KR5/KR5 sixx R650.urdf")
world.add_skeleton(robot)

# Simulate 100 steps
for _ in range(100):
    world.step()
    print(f"Positions: {robot.get_positions()}")
```

**C++**

```cpp
#include <dart/all.hpp>
#include <dart/io/read.hpp>

#include <iostream>

int main() {
  auto world = dart::simulation::World::create();

  // Load a robot from URDF
  auto robot = dart::io::readSkeleton(
      "dart://sample/urdf/KR5/KR5 sixx R650.urdf");
  world->addSkeleton(robot);

  // Simulate 100 steps
  for (int i = 0; i < 100; ++i) {
    world->step();
    std::cout << "Positions: " << robot->getPositions().transpose() << "\n";
  }
  return 0;
}
```

## Installation

The quick-start snippets above target the current `main` branch and DART 7 API.
Until DART 7 package artifacts are published, package managers may resolve the
latest DART 6.16 packages instead. Use the file-free package smoke checks below
for currently published packages, or use the source checkout path for the DART 7
quick starts.

### Python (Recommended)

| Method             | Command                               |
| ------------------ | ------------------------------------- |
| **uv** (preferred) | `uv add dartpy`                       |
| **pip**            | `pip install dartpy`                  |
| **pixi**           | `pixi add dartpy`                     |
| **conda**          | `conda install -c conda-forge dartpy` |

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

### Current Package Smoke Checks

These snippets create a tiny model in code, so they do not depend on sample data
files being present in the installed package.

**Python package**

```python
import dartpy as dart

world = dart.simulation.World()
skeleton = dart.dynamics.Skeleton("box")
skeleton.createFreeJointAndBodyNodePair()
world.addSkeleton(skeleton)
world.step()
print(f"Positions: {skeleton.getPositions()}")
```

**C++ package**

```cpp
#include <dart/dart.hpp>

#include <iostream>

int main() {
  auto world = dart::simulation::World::create();
  auto skeleton = dart::dynamics::Skeleton::create("box");
  skeleton->createJointAndBodyNodePair<dart::dynamics::FreeJoint>();
  world->addSkeleton(skeleton);
  world->step();
  std::cout << "Positions: " << skeleton->getPositions().transpose() << "\n";
  return 0;
}
```

### Source checkout

```bash
pixi install
pixi run ex headless_simulation --steps 1
```

## Documentation

- **Static Docs**: [English](https://dart.readthedocs.io/) | [한국어](https://dart-ko.readthedocs.io/)
- **AI Docs (interactive Q&A; experimental)**: [DeepWiki](https://deepwiki.com/dartsim/dart) | [NotebookLM](https://notebooklm.google.com/notebook/c0cfc8ce-17ae-415a-a615-44c4342f0da6) (Google account required)

### Developer Resources

- **[Developer Onboarding Guide](docs/onboarding/README.md)** — Architecture, components, and workflows
- **[Contributing Guide](CONTRIBUTING.md)** — Style guide and contribution process
- **[Project Direction](docs/ai/north-star.md)** — Research-focused north star and priorities
- **[Living Plans](docs/plans/README.md)** — Active and proposed roadmap items
- **[Background Theory](docs/background/README.md)** — Dynamics, contact solving, and mathematical foundations
- **[Gazebo Integration](docs/onboarding/build-system.md#gazebo-integration-feature)** — gz-physics integration workflow
- **[Release Roadmap](docs/onboarding/release-roadmap.md)** — Compatibility, deprecations, and future plans

#### Branches

- `main` — Active development targeting DART 7
- `release-6.16` — Maintenance branch for DART 6 (critical fixes only)

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
