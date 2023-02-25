.. _building_dart:

Building DART
=============

This guide describes how to build DART, a C++ library for robotics and motion
planning, using CMake. DART also has Python bindings, called dartpy, which will
be covered in a separate section.

Supported Environments
----------------------

DART is supported on the following operating systems and compilers:

+-----------------------+-----------------------+
| Operating System      | Compiler              |
+=======================+=======================+
| Ubuntu 22.04 or later | GCC 11.2 or later     |
+-----------------------+-----------------------+
| Windows 2022 or later | Visual Studio 2022    |
+-----------------------+-----------------------+
| macOS 13 or later     | Clang 13 or later     |
+-----------------------+-----------------------+

Prerequisites
-------------

Before you can build DART, you'll need to install the required and optional
dependencies. The required dependencies are the minimum set of dependencies
needed to build DART, while the optional dependencies enable additional
features in DART.

The steps for installing dependencies may vary depending on your operating
system and package manager. Below, we provide instructions for installing the
required and optional dependencies on Ubuntu, macOS, and Windows, as well as
some experimental guidance for other platforms.

.. note::

   Please note that the dependencies and installation steps are subject to
   change, so we encourage you to report any issues you encounter and
   contribute to keeping the instructions up-to-date for the community. By
   working together, we can help ensure that the DART documentation is accurate
   and helpful for everyone who uses it.

Ubuntu
~~~~~~

The dependencies for Ubuntu can be installed using the ``apt`` package
manager. The following command will install the required dependencies:

.. code-block:: bash

   $ sudo apt install \
      build-essential cmake pkg-config git libassimp-dev libccd-dev \
      libeigen3-dev libfcl-dev libfmt-dev

The following command will install the optional dependencies:

.. code-block:: bash

   $ sudo apt install \
      coinor-libipopt-dev freeglut3-dev libxi-dev libxmu-dev libbullet-dev \
      libtinyxml2-dev liburdfdom-dev liburdfdom-headers-dev \
      libopenscenegraph-dev libnlopt-cxx-dev liboctomap-dev libode-dev \
      libspdlog-dev libyaml-cpp-dev ocl-icd-opencl-dev opencl-headers \
      opencl-clhpp-headers

macOS
~~~~~

The dependencies for macOS can be installed using the ``brew`` package
manager. The following command will install the required dependencies:

.. code-block:: bash

   $ brew install assimp cmake eigen fmt fcl libccd

The following command will install the optional dependencies:

.. code-block:: bash

   $ brew install bullet freeglut ipopt nlopt octomap ode \
      open-scene-graph --HEAD \
      spdlog tinyxml2 urdfdom yaml-cpp

Windows
~~~~~~~

The dependencies for Windows can be installed using the ``vcpkg`` package
manager. The following command will install the required dependencies:

.. code-block:: bash

   $ vcpkg install --triplet x64-windows assimp ccd eigen3 entt fcl fmt spdlog

The following command will install the optional dependencies:

.. code-block:: bash

   $ vcpkg install --triplet x64-windows \
      assimp ccd eigen3 entt fcl fmt spdlog bullet3 freeglut glfw3 nlopt ode \
      opencl opengl osg pagmo2 pybind11 tinyxml2 urdfdom yaml-cpp

Arch Linux (experimental)
~~~~~~~~~~~~~~~~~~~~~~~~~

The dependencies for Arch Linux can be installed using the ``yay`` package
manager. The following command will install the required dependencies:

.. code-block:: bash

   $ yay -S assimp cmake eigen fcl fmt libccd

The following command will install the optional dependencies:

.. code-block:: bash

   $ yay -S \
      bullet coin-or-ipopt freeglut nlopt octomap ode opencl-clhpp \
      opencl-headers opencl-icd-loader openscenegraph pagmo spdlog tinyxml2 \
      urdfdom pybind11

FreeBSD (experimental)
~~~~~~~~~~~~~~~~~~~~~~

TODO

Clone the DART Repository
-------------------------

To get started with building DART, you'll need to clone the DART repository.
Here's how to do it:

1. Clone the DART repository by running the following command in your terminal:

   .. code-block:: bash

      $ git clone https://github.com/dartsim/dart.git

2. (Optional) If you want to build a specific version of DART, you can checkout
   a specific branch, tag, or commit.

   .. code-block:: bash

      $ git checkout -b <branch_or_tag_or_commit>

.. note::

   Please note that the DART repository is actively maintained, so there may be
   changes and updates to the repository over time. To get the latest
   information, we recommend referring to the DART GitHub repository.

Build Configuration
-------------------

DART uses CMake as its build system. CMake is a powerful tool that generates
build files for a variety of build systems, including Makefiles, Visual Studio
projects, and Xcode projects. For more information about available generators,
we recommend referring to the
`CMake documentation <https://cmake.org/cmake/help/latest/manual/cmake-generators.7.html>`_.

To configure the build, you'll need to create a build directory and run CMake
from that directory. Here's how to do it:

1. Create a build directory by running the following command in your terminal:

   .. code-block:: bash

      $ mkdir build

2. Change into the build directory by running the following command:

   .. code-block:: bash

      $ cd build

3. Run CMake from the build directory by running the following command:

   .. code-block:: bash

      $ cmake ..

If you want to configure the build, you can pass additional options to CMake.
For example, you can specify the build type by passing the
``-DCMAKE_BUILD_TYPE`` option. DART provides a number of CMake options that
allow you to customize the build process. Here are some of the most important
options:

+---------------------------+----------------------+------------------------------------------+
| Option                    | Default Value        | Description                              |
+===========================+======================+==========================================+
| CMAKE_BUILD_TYPE          | Release              | Specifies the build type.                |
+---------------------------+----------------------+------------------------------------------+
| DART_ENABLE_SIMD          | ON                   | Enables use of SIMD instructions.        |
+---------------------------+----------------------+------------------------------------------+
| TODO                      |                      |                                          |
+---------------------------+----------------------+------------------------------------------+

.. note::

   This list of options may not be exhaustive or up-to-date. Please refer to
   the main CMakeLists.txt file in the DART repository to confirm the list of
   available options. If you find any discrepancies or errors, please consider
   submitting a pull request to update this document.

Here are some example commands that you can use to configure the build on
different platforms with different generators:

.. code-block:: bash

   $ cmake .. -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=Release
   $ cmake .. -G "Visual Studio 15 2017" -A x64 -DCMAKE_BUILD_TYPE=Release
   $ cmake .. -G "Xcode" -DCMAKE_BUILD_TYPE=Release

Building DART from Command Line
-------------------------------

Whether or not you configured the build for IDEs, you can still build DART from
the command line using CMake's unified build commands.

To build DART from the command line, you'll need to run the build command from
the build directory. Here's how to do it:

1. Change into the build directory by running the following command:

   .. code-block:: bash

      $ cd build

2. Run the build command by running the following command:

   .. code-block:: bash

      $ cmake --build . [--target <target> [, <target2>, ...]] [-j<num_core>]

DART provides a number of CMake targets that you can use to build different
parts of the project. Here are some of the most important targets:

* ``ALL``: Builds all the targets in the project, including building tests,
  examples, tutorials, and running tests.
* ``all``: Builds core targets without tests, examples, and tutorials.
* ``tests``: Builds all the tests.
* ``test``: Runs tests (need to build tests first).
* ``tests_and_run``: Builds and runs tests.
* ``examples``: Builds all the examples.
* ``tutorials``: Builds all the tutorials.
* ``view_docs``: Builds the documentation and opens it in a web browser.
* ``install``: Installs the project.
* ``dartpy``: Builds the Python bindings (it's encouraged to build using pip
  instead).
* ``pytest``: Runs Python tests (building tests if necessary).
* ``coverage``: Runs tests and generates a coverage report.
* ``coverage_html``: Runs tests and generates an HTML coverage report.
* ``coverage_view``: Runs tests, generates an HTML coverage report, and opens
  it in a web browser.

.. note::

   Please note that this list of targets may not be exhaustive or up-to-date.
   To confirm the full list of available targets, we recommend referring to the
   main CMakeLists.txt file in the DART repository. If you find any
   discrepancies or errors, we encourage you to submit a pull request to
   update this document and help keep the documentation up-to-date for the
   community.

Building DART from IDEs
-----------------------

If you configured the build for IDEs, you can build DART from the IDEs. This
section doesn't cover how to build DART from IDEs. Please refer to the IDEs
documentation for more information. However, it's always to welcome to submit a
pull request to update this document with instructions for your favorite IDE!

Building dartpy
===============

In general, building dartpy from source is not necessary. The easiest way to
install dartpy is to use pip:

.. code-block:: bash

   $ pip install dartpy -U

TODO
