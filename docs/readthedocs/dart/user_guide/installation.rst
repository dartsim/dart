Installation
============

Cross-platform (recommended)
----------------------------

The conda-forge ecosystem provides pre-built binaries for Linux, macOS, and Windows. These packages are also consumable via Pixi.

.. code-block:: bash

   pixi add dartsim-cpp
   # or
   conda install -c conda-forge dartsim-cpp

Platform packages
-----------------

If you prefer native package managers, use the commands below.

**Ubuntu / Debian**

.. code-block:: bash

   sudo apt update
   sudo apt install libdart-all-dev

**macOS (Homebrew)**

.. code-block:: bash

   brew install dartsim

**Windows (vcpkg)**

.. code-block:: bash

   git clone https://github.com/microsoft/vcpkg.git
   cd vcpkg
   ./bootstrap-vcpkg.sh         # or .\bootstrap-vcpkg.bat on Windows
   ./vcpkg install dartsim:x64-windows

**Arch Linux (AUR)**

.. code-block:: bash

   yay -S libdart

**FreeBSD**

.. code-block:: bash

   pkg install dartsim

Package availability
--------------------

For an up-to-date view of every distribution that packages DART, refer to Repology:

.. image:: https://repology.org/badge/vertical-allrepos/dart-sim.svg
   :target: https://repology.org/project/dart-sim/versions
   :alt: Packaging status
