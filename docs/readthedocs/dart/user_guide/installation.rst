Installation
============

Ubuntu
------

To install DART on Ubuntu, you can use the following commands:

1. Add the DART PPA to your system:

.. code-block:: bash

   sudo apt-add-repository ppa:dartsim/ppa

2. Update your package list:

.. code-block:: bash

   sudo apt-get update

3. Install the `libdart7-all-dev-nightly` package:

.. code-block:: bash

   sudo apt-get install libdart7-all-dev-nightly

macOS
-----

To install DART on macOS, you can use Homebrew:

1. Install Homebrew if you haven't already:

.. code-block:: bash

   /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"

2. Install the `dartsim` formula:

.. code-block:: bash

   brew install dartsim

Windows
-------

To install DART on Windows, you can use vcpkg:

1. Install vcpkg if you haven't already:

.. code-block:: bash

   git clone https://github.com/microsoft/vcpkg.git
   cd vcpkg
   bootstrap-vcpkg.bat

2. Install the `dartsim` package:

.. code-block:: bash

   vcpkg install dartsim:x64-windows

Arch Linux (experimental)
--------------------------

To install DART on Arch Linux using the `yay` package manager, you can use the
following commands:

1. Update your package list:

.. code-block:: bash

   yay -Syu

2. Install the `libdart` package:

.. code-block:: bash

   yay -S libdart

FreeBSD (experimental)
----------------------

To install DART on FreeBSD, you can use the following commands:

1. Update your package list:

.. code-block:: bash

   pkg update

2. Install the `dartsim` package:

.. code-block:: bash

   pkg install dartsim
