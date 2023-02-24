.. _building_dart:

Building DART
=============

This guide describes how to build DART, a C++ library for robotics and motion planning, using CMake. DART also has Python bindings, called dartpy, which will be covered in a separate section.

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

Build System
------------

DART uses CMake as its build system. CMake is a powerful tool that generates build files for a variety of build systems, including Makefiles, Visual Studio projects, and Xcode projects.

CMake Options
-------------

DART provides the following CMake options:

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

Building on Ubuntu
-------------------

To build DART on Ubuntu, follow the steps below.

Prerequisites
~~~~~~~~~~~~~

1. Consider removing DART installed at the system directory to avoid any potential conflicts:

   .. code-block:: bash

      $ sudo apt-get remove libdart7-*

2. To build DART from source, you need to install the required dependencies and then build and install DART. Here's how to do it:

   .. code-block:: bash

      $ sudo apt install build-essential cmake pkg-config git
      $ sudo apt install \
         libassimp-dev \
         libccd-dev \
         libeigen3-dev \
         libfcl-dev \
         libfmt-dev

3. To build the full features, install optional dependencies:

   .. code-block:: bash

      $ sudo apt install \
         coinor-libipopt-dev \
         freeglut3-dev \
         libxi-dev \
         libxmu-dev \
         libbullet-dev \
         libtinyxml2-dev \
         liburdfdom-dev \
         liburdfdom-headers-dev \
         libopenscenegraph-dev \
         libnlopt-cxx-dev \
         liboctomap-dev \
         libode-dev \
         libspdlog-dev \
         libyaml-cpp-dev \
         ocl-icd-opencl-dev \
         opencl-headers \
         opencl-clhpp-headers

4. Clone the DART repository:

   .. code-block:: bash

      $ git clone https://github.com/dartsim/dart.git

5. (Optional) Change directory to the DART repository if you want to build a specific branch, tag, or commit:

   .. code-block:: bash

      $ cd dart
      $ git checkout -b <branch_or_tag_or_commit>

Build Configuration
~~~~~~~~~~~~~~~~~~~

To build DART, you first need to configure the build system by creating a build directory and running CMake from that directory. Here's how to configure the build system:

1. Create a build directory:

   .. code-block:: bash

      $ cd DART
      $ mkdir build
      $ cd build

2. Run CMake to generate build files:

   .. code-block:: bash

      $ cmake ..

   This command generates build files in the build directory using the CMakeLists.txt file in the DART directory.

Build Instructions
~~~~~~~~~~~~~~~~~~

Once you have configured the build system, you can build DART using the following commands:

1. Build the project:

   .. code-block:: bash

      $ cmake --build . [--target <target> [, <target2>, ...]] [-j<num_core>]

2. Run the tests (optional):

   .. code-block:: bash

      $ ctest

Building on macOS
-----------------

TODO

Building on Windows
-------------------

TODO

Building dartpy
===============

TODO
