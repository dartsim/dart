.. _building_dart:

Build
=====

This page describes the DART 6 LTS source-build paths for the ``release-6.20``
branch. The current source version is read from ``package.xml``; until release
packaging bumps it, this branch can still report a ``6.19.x`` package version
while collecting changes for DART 6.20.0.

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

``pixi run test-all`` builds the default aggregate CMake target. Run the lint
and test tasks separately when you need explicit verification output.

Core requirements
-----------------

For manual builds, use the versions and dependency set in the source tree as
the authority:

* CMake minimum: ``3.22.1`` from ``CMakeLists.txt``.
* Language level: C++17 from the DART CMake targets.
* Build system: Ninja is the Pixi default; other CMake generators can work.
* Core package dependencies: Assimp, Eigen, FCL, fmt, Bullet, ODE, OctoMap,
  spdlog, tinyxml2, urdfdom, and OpenSceneGraph.
* dartpy dependencies: Python, NumPy, and pybind11.

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
