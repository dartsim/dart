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
   * - Linux x86_64
     - One ``cp312-abi3`` wheel for 3.12 and newer on every wheel run; 3.10
       and 3.11 on release branch and tag builds
   * - macOS arm64
     - One ``cp312-abi3`` wheel for 3.12 and newer
   * - Windows x86_64
     - One ``cp312-abi3`` wheel for 3.12 and newer
   * - Other Python versions or platforms
     - Build from source with Python 3.10 or newer, or use conda-forge/Pixi when
       packages are available there

The source-build platform requirements in the
:doc:`DART build guide </dart/developer_guide/build>` differ from the wheel
runtime requirements. Current macOS arm64 wheels target macOS 15.0. Linux
wheel builds use a glibc 2.28 sysroot; the repaired wheel's manylinux tag
records its final runtime requirement, including bundled dependencies. The
``abi3`` wheels use CPython's stable ABI, so pip installs the same wheel on
Python 3.12, 3.13, and later releases.

Building from source
--------------------

The DART 6.20 ``setup.py`` package requires Python 3.10 or newer and NumPy
1.21.5 or newer. Building the bindings locally requires a matching C++
toolchain, CMake, Ninja, and the DART dependencies described in the
:doc:`DART build guide </dart/developer_guide/build>`. Isolated ``pip`` builds
use the newer tool requirements from ``pyproject.toml``.
