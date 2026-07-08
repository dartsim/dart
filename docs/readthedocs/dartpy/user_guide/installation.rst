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
     - 3.13 on regular wheel runs; 3.10, 3.11, 3.12, and 3.13 on release branch
       and tag builds
   * - macOS arm64
     - 3.13 on regular wheel runs; 3.12 and 3.13 on release branch and tag
       builds
   * - Windows x86_64
     - 3.13 on regular wheel runs; 3.12 and 3.13 on release branch and tag
       builds
   * - Other Python versions or platforms
     - Build from source with Python 3.7 or newer, or use conda-forge/Pixi when
       packages are available there

Building from source
--------------------

The DART 6 ``setup.py`` package supports Python 3.7 or newer. Building the
bindings locally requires a matching C++ toolchain, CMake, Ninja, and the DART
dependencies used by the source checkout.
