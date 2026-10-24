Installation
============

Quick install commands
----------------------

For packaged dartpy bindings, use one of the package channels below:

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
     - One ``cp312-abi3`` wheel for 3.12 and newer
   * - macOS arm64
     - One ``cp312-abi3`` wheel for 3.12 and newer
   * - Windows x86_64
     - One ``cp312-abi3`` wheel for 3.12 and newer
   * - Other Python versions or platforms
     - Build from source with Python 3.12 or newer, or use conda-forge/Pixi when
       packages are available there

The source-build platform requirements in the
:doc:`DART build guide </dart/developer_guide/build>` differ from the wheel
runtime requirements. Current macOS arm64 wheels target macOS 15.0. Linux
wheels, including their bundled dependencies, are built with the manylinux
2.28 toolchain and tagged ``manylinux_2_28``. The
``abi3`` wheels use CPython's stable ABI, so pip installs the same wheel on
Python 3.12, 3.13, and later releases.

Building from source
--------------------

The package requires Python 3.12 or newer and NumPy 1.26.4 or newer.
``pip install .`` builds it with scikit-build-core (``pyproject.toml``), which
requires a matching C++ toolchain, CMake 3.28 or newer, Ninja, and the DART
dependencies described in the
:doc:`DART build guide </dart/developer_guide/build>`. Isolated ``pip`` builds
install the tool requirements from ``pyproject.toml``. Linux source installs also
enable ``utils-assets`` and require the libcurl >= 8.5 and OpenSSL Crypto
>= 3.0.13 development packages (for example, ``libcurl4-openssl-dev`` and
``libssl-dev`` on Debian/Ubuntu). macOS and Windows source installs need those
libraries when explicitly enabling ``DART_BUILD_UTILS_ASSETS``. Stock packages
meet these floors on Ubuntu 24.04 or newer; Debian 12 (libcurl 7.88.1) and
Ubuntu 22.04 (libcurl 7.81, OpenSSL 3.0.2) are too old. To omit the retriever and these development
packages, use ``CMAKE_ARGS=-DDART_BUILD_UTILS_ASSETS=OFF pip install .``.

dartpy always includes ``dartpy.gui``. The wheel build uses DART's
pinned private Filament SDK on Linux, macOS arm64, and 64-bit Windows.
Configuring downloads that SDK; on Linux it is compiled from source, which also
needs Clang 20 or newer. Other platforms need an installed Filament 1.77.3 SDK,
found through ``Filament_ROOT`` or the CMake prefix path.
