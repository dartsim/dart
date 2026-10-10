.. _building_dart:

Build
=====

This page describes the DART 6 LTS source-build paths for the ``main`` development
branch. The current source version is read from ``package.xml``; until release
packaging bumps it, this branch can still report a ``6.19.x`` package version
while collecting changes for DART 6.21.0. DART 6.20.0 is being stabilized on
``release-6.20``.

Recommended Pixi build
----------------------

The reproducible developer path is the Pixi environment tracked in
``pixi.toml``. It supplies the compiler tools, CMake, Ninja, Python, Doxygen,
Sphinx, test tools, and the C++ dependencies used by the branch.

.. code-block:: bash

   pixi run config
   pixi run build
   pixi run build-py-dev

Run the relevant verification gates from the same environment:

.. code-block:: bash

   pixi run lint
   pixi run test
   pixi run test-py
   pixi run docs-build

``pixi run test-all`` builds the default aggregate CMake target. The default
configuration enables testing, so that aggregate also runs CTest and the
Python tests through the CMake graph. Run lint separately, and use the focused
test tasks when you need clearer failure output.

Platform and tool requirements
------------------------------

DART 6.20 requires C++17 and the following source-build baselines:

.. list-table::
   :header-rows: 1
   :widths: 20 30 50

   * - Platform
     - OS baseline
     - Compiler minimum
   * - Ubuntu
     - 22.04 LTS; also tested on 24.04 LTS
     - GCC 11.2 or upstream LLVM Clang 13
   * - macOS
     - 13 Ventura
     - Apple Clang 14 from Xcode 14.1
   * - Windows
     - Windows Server 2022 CI baseline
     - Visual Studio 2022, v143 toolset (``MSVC_VERSION >= 1930``)

Apple Clang and upstream LLVM Clang use different version numbers. The
upstream Clang minimum also applies to clang-cl. The Windows CI runner
baseline does not establish a minimum supported Windows desktop version.
CMake checks the Xcode version when the Xcode generator reports it; other
generators check the Apple Clang version. Use Xcode 14.1 or newer for the
macOS source-build baseline.
Hosted CI uses newer macOS runners because macOS 13 runners have been retired;
the macOS 13/Xcode 14.1 source baseline has no current native CI lane.

Manual builds require CMake 3.22.1 or newer and pkg-config 0.29.2 or newer.
Ninja is the Pixi default; other CMake generators can work. Pixi supplies
newer tool and dependency versions than these minimum source requirements.
Isolated Python package builds use ``setuptools >= 84.0.0``,
``wheel >= 0.45.1``, ``ninja >= 1.12.1``, and the existing CMake
``>= 4.4.4, < 4.4.5`` constraint from ``pyproject.toml``.

Library requirements
--------------------

The minimum versions below apply when the corresponding component is enabled.
Optional components remain optional.

.. list-table::
   :header-rows: 1
   :widths: 35 20 45

   * - Dependency
     - Minimum version
     - Used by
   * - Assimp
     - 5.2.2
     - Core mesh loading
   * - Eigen
     - 3.4.0
     - Core mathematics
   * - FCL
     - 0.7.0
     - Core collision detection
   * - fmt
     - 8.1.1
     - Core formatting
   * - Bullet
     - 3.06
     - Bullet collision backend
   * - OctoMap
     - 1.9.7
     - Occupancy maps
   * - ODE
     - 0.16.2
     - ODE collision backend
   * - tinyxml2
     - 9.0.0
     - XML parsers
   * - urdfdom
     - 3.0.1
     - URDF parser
   * - spdlog
     - 1.9.2
     - Logging support
   * - OpenSceneGraph
     - 3.6.5
     - OSG GUI
   * - ImGui
     - 1.91.9
     - OSG GUI controls
   * - Python
     - 3.10
     - dartpy
   * - NumPy
     - 1.21.5
     - dartpy
   * - pybind11
     - 3.0.3
     - dartpy (bundled by default)
   * - Tracy
     - 0.11.1
     - Optional profiling backend

FCL's libccd dependency must be built in double precision
(``-DENABLE_DOUBLE_PRECISION=ON``), with FCL built against it; otherwise the
build stops. ``-DDART_ALLOW_SINGLE_PRECISION_LIBCCD=ON`` builds anyway.

These source-build requirements differ from the runtime requirements of
published :doc:`dartpy wheels </dartpy/user_guide/installation>`.

Manual CMake build
------------------

Install the dependencies with your platform package manager or use the Pixi
environment as the dependency prefix. Then configure and build:

.. code-block:: bash

   cmake -G Ninja -S . -B build/default/cpp/Release \
       -DCMAKE_BUILD_TYPE=Release \
       -DDART_BUILD_DARTPY=ON \
       -DDART_BUILD_PROFILE=ON \
       -DDART_USE_SYSTEM_GOOGLEBENCHMARK=ON \
       -DDART_USE_SYSTEM_GOOGLETEST=ON \
       -DDART_USE_SYSTEM_IMGUI=ON \
       -DDART_USE_SYSTEM_PYBIND11=ON \
       -DDART_USE_SYSTEM_TRACY=ON
   cmake --build build/default/cpp/Release -j

Use ``-DCMAKE_PREFIX_PATH=<prefix>`` when dependencies are installed outside the
compiler's default search paths.

Important CMake options
-----------------------

The source-of-truth option list is in ``CMakeLists.txt``. Common branch options
include:

.. list-table::
   :header-rows: 1
   :widths: 35 20 45

   * - Option
     - Default
     - Purpose
   * - ``DART_BUILD_DARTPY``
     - ``OFF``
     - Build the Python bindings.
   * - ``DART_BUILD_GUI_OSG``
     - ``ON``
     - Build the OpenSceneGraph GUI component.
   * - ``DART_ENABLE_GUI_OSG_SMOKE_TESTS``
     - ``OFF``
     - Build off-screen GUI capture smoke tests when a display or Xvfb is
       available.
   * - ``DART_ENABLE_SIMD``
     - ``OFF``
     - Add local-machine SIMD compiler flags such as ``-march=native``.
   * - ``DART_SIMD_FORCE_SCALAR``
     - ``OFF``
     - Force the header-only ``dart/simd`` module to use its scalar fallback
       backend in tests.
   * - ``DART_BUILD_PROFILE``
     - ``OFF``
     - Build profiling support.
   * - ``DART_PROFILE_BUILTIN``
     - ``ON``
     - Enable DART's built-in text profiling backend.
   * - ``DART_PROFILE_TRACY``
     - ``OFF``
     - Enable the Tracy profiling backend for local developer profiling.
   * - ``DART_USE_SYSTEM_IMGUI``
     - ``OFF``
     - Use a system ImGui package instead of the bundled compatibility target.
   * - ``DART_USE_SYSTEM_PYBIND11``
     - ``OFF``
     - Use a system pybind11 package.

Build targets
-------------

Useful CMake targets include:

* ``all``: build the default libraries and tools.
* ``tests``: build the C++ tests.
* ``test``: run CTest after tests are built.
* ``examples``: build examples.
* ``tutorials``: build tutorials.
* ``dartpy``: build the Python bindings.
* ``pytest``: run Python tests through the CMake target.
* ``install``: install the configured components.
* ``view_docs``: build and open local documentation.

For most development work, prefer the Pixi task names above because they encode
the branch's expected build directory, dependency prefix, and platform-specific
settings.
