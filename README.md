# DART

<p align="center">
  <img src="https://raw.githubusercontent.com/dartsim/dart/main/docs/dart_logo_377x107.jpg" alt="DART: Dynamic Animation and Robotics Toolkit">
</p>

<p align="center">
  <a href="https://github.com/dartsim/dart/actions/workflows/ci_ubuntu.yml"><img src="https://github.com/dartsim/dart/actions/workflows/ci_ubuntu.yml/badge.svg" alt="CI Linux"></a>
  <a href="https://github.com/dartsim/dart/actions/workflows/ci_macos.yml"><img src="https://github.com/dartsim/dart/actions/workflows/ci_macos.yml/badge.svg" alt="CI macOS"></a>
  <a href="https://github.com/dartsim/dart/actions/workflows/ci_windows.yml"><img src="https://github.com/dartsim/dart/actions/workflows/ci_windows.yml/badge.svg" alt="CI Windows"></a>
  <br>
  <a href="https://dart.readthedocs.io/en/latest/"><img src="https://readthedocs.org/projects/dart/badge/?version=latest" alt="Documentation Status"></a>
  <a href="https://codecov.io/gh/dartsim/dart"><img src="https://codecov.io/gh/dartsim/dart/branch/main/graph/badge.svg" alt="codecov"></a>
  <a href="https://app.codacy.com/gh/dartsim/dart/dashboard"><img src="https://app.codacy.com/project/badge/Grade/2d95a9b951be4b73a71097670ec351e8" alt="Codacy Badge"></a>
  <br>
  <a href="https://pypi.org/project/dartpy/"><img src="https://img.shields.io/pypi/v/dartpy" alt="PyPI Version"></a>
  <a href="https://anaconda.org/conda-forge/dartsim"><img src="https://img.shields.io/conda/vn/conda-forge/dartsim?label=conda-forge" alt="conda-forge Version"></a>
  <a href="https://github.com/dartsim/dart/blob/main/LICENSE"><img src="https://img.shields.io/badge/License-BSD_2--Clause-blue.svg" alt="License"></a>
</p>

DART (Dynamic Animation and Robotics Toolkit) is an open-source C++17 library,
with Python bindings (dartpy), for the kinematics, dynamics, and contact
simulation of articulated rigid and soft bodies in robotics and computer
animation. It represents articulated systems in generalized coordinates and
computes their motion with Featherstone's Articulated Body Algorithm, which
makes it accurate and stable.

> [!NOTE]
> `main` is the development branch. For the latest stable release, install a
> package below.

<p align="center">
  <img src="https://raw.githubusercontent.com/dartsim/dart/main/docs/assets/dart_demos.gif" alt="Four dart-demos scenes: an Atlas humanoid walking under a SIMBICON controller, 125 boxes dropping onto the ground, a soft-body worm crawling, and a soft ball landing with adaptive soft contact">
  <br>
  <sub>Scenes from the <code>dart-demos</code> app (<code>pixi run demos</code>)</sub>
</p>

## Why DART?

- **Research-grade dynamics** — Featherstone's algorithms in generalized
  coordinates, with direct access to the mass matrix, Coriolis and gravity
  forces, Jacobians, and their derivatives
- **Contacts and constraints** — LCP-based contact with Dantzig and PGS solvers,
  joint limits, closed kinematic loops, and soft bodies; collision detection
  through FCL (default), Bullet, ODE, or DART's native detector
- **Model loading** — URDF, SDF, SKEL, and MJCF (experimental) parsers
- **Kinematics and control** — hierarchical whole-body inverse kinematics with
  analytical solver support (e.g., IkFast), plus operational-space and
  stable-PD control examples
- **C++ and Python** — the same engine through dartpy, packages on PyPI,
  conda-forge, and major package managers, and reproducible source builds with
  pixi
- **Battle-tested** — powers [Gazebo](https://gazebosim.org) through
  gz-physics, and is used by
  [research labs worldwide](https://dart.readthedocs.io/en/latest/community/who_uses_dart.html)

## Quick Start

**Python**

```python
import dartpy as dart

world = dart.simulation.World()  # 1 ms time step, gravity along -z

# A free-floating 1 kg body (the default inertia), 1 m above the ground.
box = dart.dynamics.Skeleton("box")
box.createFreeJointAndBodyNodePair()
box.setPosition(5, 1.0)  # FreeJoint DOFs: rotation (0-2), translation (3-5)
world.addSkeleton(box)

for _ in range(100):
    world.step()

print(f"t = {world.getTime():.3f} s, z = {box.getPosition(5):.3f} m")
# t = 0.100 s, z = 0.950 m
```

**C++**

```cpp
#include <dart/dart.hpp>

#include <iostream>

int main()
{
  auto world = dart::simulation::World::create();

  // A free-floating 1 kg body (the default inertia), 1 m above the ground.
  auto box = dart::dynamics::Skeleton::create("box");
  box->createJointAndBodyNodePair<dart::dynamics::FreeJoint>();
  box->setPosition(5, 1.0); // FreeJoint DOFs: rotation (0-2), translation (3-5)
  world->addSkeleton(box);

  for (int i = 0; i < 100; ++i)
    world->step();

  std::cout << "t = " << world->getTime() << " s, z = " << box->getPosition(5)
            << " m\n";
}
```

```cmake
find_package(DART REQUIRED CONFIG)
target_link_libraries(my_app PUBLIC dart)
```

## Installation

### Python

| Method    | Command                               |
| --------- | ------------------------------------- |
| **pip**   | `pip install dartpy`                  |
| **uv**    | `uv add dartpy`                       |
| **pixi**  | `pixi add dartpy`                     |
| **conda** | `conda install -c conda-forge dartpy` |

PyPI wheels cover the most common platforms and Python versions; see the
[Python installation guide](https://dart.readthedocs.io/en/latest/dartpy/user_guide/installation.html)
for the exact list.

### C++

| Platform                         | Command                                                              |
| -------------------------------- | -------------------------------------------------------------------- |
| **Cross-platform** (recommended) | `pixi add dartsim-cpp` or `conda install -c conda-forge dartsim-cpp` |
| Ubuntu / Debian                  | `sudo apt install libdart-all-dev`                                   |
| Arch Linux (AUR)                 | `yay -S libdart`                                                     |
| FreeBSD                          | `pkg install dartsim`                                                |
| macOS (Homebrew)                 | `brew install dartsim`                                               |
| Windows (vcpkg)                  | `vcpkg install dartsim:x64-windows`                                  |

Distribution packages can lag behind the latest release:
[all distributions →](https://repology.org/project/dart-sim/versions). See the
[C++ installation guide](https://dart.readthedocs.io/en/latest/dart/user_guide/installation.html)
for details.

### Build from source

```bash
git clone https://github.com/dartsim/dart.git
cd dart
pixi run test   # configure, build, and run the C++ tests
pixi run demos  # build and launch the interactive demo app
```

See the [build guide](https://dart.readthedocs.io/en/latest/dart/developer_guide/build.html)
for options and other platforms.

## Documentation

- **User guide**: [English](https://dart.readthedocs.io/) |
  [한국어](https://dart.readthedocs.io/ko/latest/)
- **Gallery**: [videos and projects](https://dart.readthedocs.io/en/latest/gallery.html)
  · **Performance**: [benchmark dashboard](https://dartsim.github.io/dart/performance/dart6/)
- **Questions**: [GitHub Discussions](https://github.com/dartsim/dart/discussions)
  · **AI Q&A (experimental)**: [DeepWiki](https://deepwiki.com/dartsim/dart)

## Contributing

Contributions are welcome; start with the
[contributing guide](https://github.com/dartsim/dart/blob/main/CONTRIBUTING.md).

- [Architecture](https://github.com/dartsim/dart/blob/main/docs/onboarding/architecture.md)
  — components and compatibility boundaries
- [Building](https://github.com/dartsim/dart/blob/main/docs/onboarding/building.md)
  and [testing](https://github.com/dartsim/dart/blob/main/docs/onboarding/testing.md)
- [Background theory](https://github.com/dartsim/dart/blob/main/docs/background/README.md)
  — dynamics, contact solving, and mathematical foundations
- [Changelog](https://github.com/dartsim/dart/blob/main/CHANGELOG.md)
- [AGENTS.md](https://github.com/dartsim/dart/blob/main/AGENTS.md) — guidelines
  for AI coding agents

### Branches

`main` is the development branch for the next release: pull requests target it,
all new patches land on it, and releases are tagged from it. Stable versions are
available as [tagged releases](https://github.com/dartsim/dart/releases) and as
the packages above.

## Citation

If you use DART in an academic publication, please consider citing the
[JOSS paper](https://doi.org/10.21105/joss.00500):

```bibtex
@article{Lee2018,
  doi       = {10.21105/joss.00500},
  url       = {https://doi.org/10.21105/joss.00500},
  year      = {2018},
  publisher = {The Open Journal},
  volume    = {3},
  number    = {22},
  pages     = {500},
  author    = {Jeongseok Lee and Michael X. Grey and Sehoon Ha and Tobias Kunz and Sumit Jain and Yuting Ye and Siddhartha S. Srinivasa and Mike Stilman and C. Karen Liu},
  title     = {DART: Dynamic Animation and Robotics Toolkit},
  journal   = {Journal of Open Source Software}
}
```

## License

DART is licensed under the
[BSD 2-Clause License](https://github.com/dartsim/dart/blob/main/LICENSE).

## Star History

<a href="https://star-history.com/#dartsim/dart&Date">
  <picture>
    <source media="(prefers-color-scheme: dark)" srcset="https://api.star-history.com/svg?repos=dartsim/dart&type=Date&theme=dark">
    <img alt="Star History Chart" src="https://api.star-history.com/svg?repos=dartsim/dart&type=Date">
  </picture>
</a>
