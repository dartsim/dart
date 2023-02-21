Python
======

The Python bindings for DART are available on PyPI as the `dartpy` package. The following operating systems are supported:

+--------------+--------+--------+--------+--------+--------+
| Operating    | Python | Python | Python | Python | Python |
| System       | 3.7    | 3.8    | 3.9    | 3.10   | 3.11   |
+==============+========+========+========+========+========+
| manylinux    |   O    |   O    |   O    |   O    |   O    |
+--------------+--------+--------+--------+--------+--------+
| macOS        |   X    |   O    |   O    |   O    |   O    |
+--------------+--------+--------+--------+--------+--------+
| macOS arm64  |   X    |   O    |   X    |   O    |   O    |
+--------------+--------+--------+--------+--------+--------+
| Windows      |   X    |   O    |   O    |   O    |   O    |
+--------------+--------+--------+--------+--------+--------+

.. note::

   The information in this table may be outdated. For the most up-to-date information on the availability of the Python bindings for DART, please refer to the `dartpy` package on PyPI: https://pypi.org/project/dartpy/

To install the Python bindings for DART using the `dartpy` package from PyPI, you can use the following command:

.. code-block:: bash

   pip install dartpy -U

C++
===

Ubuntu
------

To install DART on Ubuntu, you can use the following commands:

1. Add the DART PPA to your system:

.. code-block:: bash

   sudo apt-add-repository ppa:dartsim/ppa

2. Update your package list:

.. code-block:: bash

   sudo apt-get update

3. Install the `libdart6-all-dev` package:

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

Arch Linux (Experimental)
-------------------------

.. note::

   The installation instructions for Arch Linux are currently experimental and may not work on all systems. Use at your own risk.

To install DART on Arch Linux using the `yay` package manager, you can use the following commands:

1. Update your package list:

.. code-block:: bash

   yay -Syu

2. Install the `libdart` package:

.. code-block:: bash

   yay -S libdart
