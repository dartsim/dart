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

DART (Dynamic Animation and Robotics Toolkit) is an open-source library that
provides data structures and algorithms for kinematic and dynamic applications
in robotics and computer animation. Renowned for its accuracy and stability,
DART utilizes generalized coordinates to represent articulated rigid body
systems and employs Featherstone's Articulated Body Algorithm to compute motion
dynamics.

<p align="center">
  <img src="docs/assets/unitree_g1_demo.gif" width="720" alt="Unitree G1 humanoid demo" />
</p>

## Why DART?

- **Accuracy & Stability** — Featherstone's Articulated Body Algorithm with proven numerical stability
- **Unified Format Support** — Load URDF, SDF, MJCF, and SKEL models through a single API
- **Full-featured Collision** — Multiple collision detection backends (FCL, Bullet, ODE)
- **Constraint Dynamics** — Joint limits, contacts, and closed-loop constraints solved together
- **Cross-platform** — Linux, macOS, Windows with Python bindings included
- **Battle-tested** — Powers [Gazebo](https://gazebosim.org), research labs, and production systems worldwide

## Quick Start

**Python**

```python
import dartpy as dart

world = dart.World()

# Load a robot from URDF
urdf = dart.io.UrdfParser()
robot = urdf.parseSkeleton("dart://sample/urdf/KR5/KR5 sixx R650.urdf")
world.addSkeleton(robot)

# Simulate 100 steps
for _ in range(100):
    world.step()
    print(f"Positions: {robot.getPositions()}")
```

**C++**

```cpp
#include <dart/dart.hpp>

int main() {
  auto world = dart::simulation::World::create();

  // Load a robot from URDF
  auto robot = dart::io::urdf::readSkeleton("path/to/robot.urdf");
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

## Documentation

- **Static Docs**: [English](https://dart.readthedocs.io/) | [한국어](https://dart-ko.readthedocs.io/)
- **AI Docs (interactive Q&A; experimental)**: [DeepWiki](https://deepwiki.com/dartsim/dart) | [NotebookLM](https://notebooklm.google.com/notebook/c0cfc8ce-17ae-415a-a615-44c4342f0da6) (Google account required)

### Developer Resources

- **[Developer Onboarding Guide](docs/onboarding/README.md)** — Architecture, components, and workflows
- **[Contributing Guide](CONTRIBUTING.md)** — Style guide and contribution process
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
