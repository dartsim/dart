Installation
============

Quick install commands
----------------------

For the DART 6 Python bindings, use one of the package channels below:

.. code-block:: bash

   pixi add dartpy
   # or
   conda install -c conda-forge dartpy
   # or
   pip install --upgrade dartpy

Supported PyPI wheel lanes
--------------------------

The release branch's ``publish_dartpy.yml`` workflow builds these PyPI wheel
lanes. Wheel versions are sourced from ``package.xml`` and are published from
matching version tags.

.. list-table::
   :header-rows: 1
   :widths: 35 65

   * - Platform
     - CPython wheels built by this branch
   * - Linux x86_64 and aarch64
     - 3.13 on regular wheel runs; 3.12, 3.13, 3.14, and 3.15 on nightly,
       release branch, and tag builds
   * - macOS arm64
     - 3.13 on regular wheel runs; 3.12, 3.13, and 3.14 on nightly, release
       branch, and tag builds
   * - Windows x86_64
     - 3.13 on regular wheel runs; 3.12, 3.13, and 3.14 on nightly, release
       branch, and tag builds
   * - Other Python versions or platforms
     - Build from source with Python 3.10 or newer, or use conda-forge/Pixi when
       packages are available there

cp315 wheels cover Linux x86_64 and aarch64. macOS arm64 and Windows x64 cp315
wheels follow once conda-forge's bullet-cpp has Python 3.15 builds
(`conda-forge/bullet-feedstock#70 <https://github.com/conda-forge/bullet-feedstock/pull/70>`_).

Starting with DART 6.20.1, Python 3.10 and 3.11 no longer receive new wheels;
``pip`` on those versions continues to install dartpy 6.20.0.

The source-build platform requirements in the
:doc:`DART build guide </dart/developer_guide/build>` differ from the wheel
runtime requirements. Current macOS arm64 wheels target macOS 15.0. Linux
wheel builds use a glibc 2.28 sysroot; the repaired wheel's manylinux tag
records its final runtime requirement, including bundled dependencies.

Building from source
--------------------

The DART 6.20 ``setup.py`` package requires Python 3.10 or newer and NumPy
1.21.5 or newer. Building the bindings locally requires a matching C++
toolchain, CMake, Ninja, and the DART dependencies described in the
:doc:`DART build guide </dart/developer_guide/build>`. Isolated ``pip`` builds
use the newer tool requirements from ``pyproject.toml``.
