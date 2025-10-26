# DART

<br>
<p align="center">
  <img src="https://raw.githubusercontent.com/dartsim/dart/master/docs/dart_logo_377x107.jpg" alt="DART: Dynamic Animation and Robotics Toolkit">
</p>

DART (Dynamic Animation and Robotics Toolkit) is an open-source library that
provides data structures and algorithms for kinematic and dynamic applications
in robotics and computer animation. Renowned for its accuracy and stability,
DART utilizes generalized coordinates to represent articulated rigid body
systems and employs Featherstone's Articulated Body Algorithm to compute motion
dynamics.

## Getting Started

Install DART using your preferred package manager:

**Python**
```shell
uv add dartpy                             # uv (preferred)
pip install dartpy                        # PyPI
pixi add dartpy                           # or Pixi (preferred)
conda install -c conda-forge dartpy       # or Conda
```

**C++**
```shell
# Cross-platform (recommended)
pixi add dartsim-cpp  # or: conda install -c conda-forge dartsim-cpp

# Platform-specific
sudo apt install libdart-all-dev          # Ubuntu
yay -S libdart                            # Arch Linux
pkg install dartsim                       # FreeBSD
brew install dartsim                      # macOS
vcpkg install dartsim:x64-windows         # Windows
```

## Documentation

### Developer Resources

- **[Developer Onboarding Guide](docs/onboarding/README.md)** - Comprehensive guide for new contributors covering architecture, components, and workflows
- **[Contributing Guide](CONTRIBUTING.md)** - Style guide and contribution process

### User Documentation

For more information on DART, please visit the DART documentation: [English](https://dart.readthedocs.io/) | [한국어](https://dart-ko.readthedocs.io/) (WIP)

An overview of DART is also available on [DeepWiki](https://deepwiki.com/dartsim/dart).

## Project Status

| Item                  | Status |
| --------------------- | ------ |
| Build                 | [![CI Ubuntu](https://github.com/dartsim/dart/actions/workflows/ci_ubuntu.yml/badge.svg)](https://github.com/dartsim/dart/actions/workflows/ci_ubuntu.yml) [![CI macOS](https://github.com/dartsim/dart/actions/workflows/ci_macos.yml/badge.svg)](https://github.com/dartsim/dart/actions/workflows/ci_macos.yml) [![CI Windows](https://github.com/dartsim/dart/actions/workflows/ci_windows.yml/badge.svg)](https://github.com/dartsim/dart/actions/workflows/ci_windows.yml) |
| Doc, Coverage, Linter | [![API Documentation](https://github.com/dartsim/dart/actions/workflows/api_doc.yml/badge.svg)](https://github.com/dartsim/dart/actions/workflows/api_doc.yml)  [![Documentation Status](https://readthedocs.org/projects/dart/badge/?version=latest)](https://dart.readthedocs.io/en/latest/?badge=latest) [![codecov](https://codecov.io/gh/dartsim/dart/branch/main/graph/badge.svg)](https://codecov.io/gh/dartsim/dart)   [![Codacy Badge](https://app.codacy.com/project/badge/Grade/2d95a9b951be4b73a71097670ec351e8)](https://www.codacy.com/gh/dartsim/dart/dashboard?utm_source=github.com&amp;utm_medium=referral&amp;utm_content=dartsim/dart&amp;utm_campaign=Badge_Grade) |
| Packages              | [![Packaging status](https://repology.org/badge/vertical-allrepos/dart-sim.svg)](https://repology.org/project/dart-sim/versions) [![Anaconda-Server Badge](https://anaconda.org/conda-forge/dartsim/badges/version.svg)](https://anaconda.org/conda-forge/dartsim) [![PyPI Version](https://img.shields.io/pypi/v/dartpy)](https://pypi.org/project/dartpy/) |
| Maintenance           | [![Average time to resolve an issue](http://isitmaintained.com/badge/resolution/dartsim/dart.svg)](http://isitmaintained.com/project/dartsim/dart "Average time to resolve an issue") [![Percentage of issues still open](http://isitmaintained.com/badge/open/dartsim/dart.svg)](http://isitmaintained.com/project/dartsim/dart "Percentage of issues still open") |

## Citation

If you use DART in an academic publication, please consider citing this [JOSS Paper](https://doi.org/10.21105/joss.00500) [[BibTeX](https://gist.github.com/jslee02/998b8809e3ae1b7aef6ef04dd2ad5e27)]
