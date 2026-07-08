Installation
============

Cross-platform
--------------

For a reproducible DART 6 environment across Linux, macOS, and Windows, use
Pixi or conda-forge:

.. code-block:: bash

   pixi add dartsim-cpp
   # or
   conda install -c conda-forge dartsim-cpp

Ubuntu / Debian
---------------

To install the distro-packaged DART C++ libraries on Ubuntu or Debian:

.. code-block:: bash

   sudo apt update
   sudo apt install libdart-all-dev

If you need a newer DART 6 LTS build than your distro provides, prefer the
cross-platform package channels above.

macOS
-----

To install DART on macOS, use Homebrew:

.. code-block:: bash

   brew install dartsim

Windows
-------

To install DART on Windows, use vcpkg:

.. code-block:: bash

   git clone https://github.com/microsoft/vcpkg.git
   cd vcpkg
   .\bootstrap-vcpkg.bat
   .\vcpkg install dartsim:x64-windows

Arch Linux
----------

To install DART on Arch Linux using the ``yay`` package manager:

.. code-block:: bash

   yay -S libdart

FreeBSD
-------

To install DART on FreeBSD:

.. code-block:: bash

   pkg install dartsim

Package availability
--------------------

For an up-to-date view of every distribution that packages DART, refer to
Repology:

.. image:: https://repology.org/badge/vertical-allrepos/dart-sim.svg
   :target: https://repology.org/project/dart-sim/versions
   :alt: Packaging status
