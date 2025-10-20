Building dartpy from Source
===========================

This guide covers building dartpy (the Python bindings) from source for development purposes.
For end users, we recommend installing from PyPI (``pip install dartpy``) or conda-forge.

Prerequisites
-------------

Before building dartpy from source, ensure you have:

* Python 3.7 or higher
* pip
* pybind11 (>= 2.13)
* The dependencies required to build DART (see the C++ build guide)

.. note::

   For the most up-to-date build requirements, refer to the
   :doc:`C++ build guide </dart/developer_guide/build>`.

Installation Methods
--------------------

There are several ways to build and install dartpy from source, depending on your use case:

Method 1: Traditional CMake Install (System-wide or Virtual Environment)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This method builds DART with Python bindings and installs them to your system Python
or active virtual environment.

.. code-block:: bash

   # Create build directory
   mkdir build && cd build

   # Configure with Python bindings enabled
   cmake -DDART_BUILD_DARTPY=ON ..

   # Build the dartpy module
   cmake --build . --target dartpy

   # Install (may require sudo for system-wide installation)
   sudo cmake --install .

   # Or install to a virtual environment (no sudo needed)
   cmake --install . --prefix $VIRTUAL_ENV

.. warning::

   System-wide installation (with sudo) will install dartpy to your system Python.
   Consider using a virtual environment to avoid conflicts with system packages.

Method 2: Editable Install with pip (Recommended for Development)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This method creates an editable install, meaning changes to the Python code will be
reflected immediately without reinstalling.

.. code-block:: bash

   # From the DART source root directory
   pip install -e . -v --no-build-isolation

This uses scikit-build-core to:

1. Run CMake with ``-DDART_BUILD_DARTPY=ON -DDART_BUILD_WHEELS=ON``
2. Build dartpy in ``build/``
3. Create an editable install that links to the build directory

.. note::

   The ``--no-build-isolation`` flag uses your environment's build dependencies
   instead of creating an isolated build environment. This is faster for development.

Method 3: Using pixi (Reproducible Development Environment)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

pixi provides a reproducible development environment with all dependencies managed automatically:

.. code-block:: bash

   # Install dependencies and set up environment (first time only)
   pixi install

   # Build dartpy for development (creates local build)
   pixi run build-py-dev

   # Run examples with the built dartpy
   pixi run py-ex-hello-world

   # Run tests
   pixi run test-py

This method:

* Manages all C++ and Python dependencies automatically
* Provides tasks for common development workflows
* Isolates the build from your system environment

For more pixi tasks, see:

.. code-block:: bash

   pixi task list

Method 4: Building Wheels Locally
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

To build a wheel package for testing or distribution:

.. code-block:: bash

   # Build wheel without installing
   pip wheel . --no-deps -w dist/

   # Or using pixi
   pixi run build-wheel

   # Verify wheel contents
   python scripts/verify_wheel.py dist/dartpy-*.whl

   # Install the wheel
   pip install dist/dartpy-*.whl

Building for Different Configurations
--------------------------------------

Minimal Build (Core Only)
~~~~~~~~~~~~~~~~~~~~~~~~~~

For a faster build with only core functionality:

.. code-block:: bash

   cmake -DDART_BUILD_DARTPY=ON \
         -DDART_BUILD_GUI_OSG=OFF \
         -DDART_BUILD_WHEELS=ON \
         ..

This skips optional components like OpenSceneGraph GUI.

Debug Build
~~~~~~~~~~~

For development and debugging:

.. code-block:: bash

   cmake -DDART_BUILD_DARTPY=ON \
         -DCMAKE_BUILD_TYPE=Debug \
         ..

Release Build
~~~~~~~~~~~~~

For performance testing:

.. code-block:: bash

   cmake -DDART_BUILD_DARTPY=ON \
         -DCMAKE_BUILD_TYPE=Release \
         ..

Verifying the Installation
---------------------------

After installation, verify that dartpy can be imported:

.. code-block:: bash

   python -c "import dartpy; print(dartpy.__file__)"

This should print the path to the dartpy module. If you see an ImportError, check:

1. The module was built successfully (check build output for errors)
2. The module was installed to a location in your PYTHONPATH
3. All required shared libraries are available (check with ``ldd`` on Linux, ``otool`` on macOS)

Troubleshooting
---------------

"Cannot find dartpy module"
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

* **Virtual environment**: Ensure your virtual environment is activated
* **System install**: The module may require ``sudo`` for installation or may be in a non-standard location
* **Check install location**: Run ``python -m site`` to see your site-packages directories

"Shared library not found" errors
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

* Install missing dependencies (fcl, assimp, urdfdom, etc.)
* On Linux, update ``LD_LIBRARY_PATH``: ``export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH``
* On macOS, update ``DYLD_LIBRARY_PATH``

Build fails with CMake errors
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

* Ensure you have all required dependencies installed
* Try a clean build: ``rm -rf build && mkdir build && cd build``
* Check the :doc:`C++ build guide </dart/developer_guide/build>` for platform-specific instructions

For more information on the packaging strategy and build system architecture,
see ``PACKAGING_STRATEGY.md`` in the DART source repository.
