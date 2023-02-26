Installation
============

Python
------

To install the Python bindings for DART using the `dartpy` package from PyPI,
you can use the following command:

.. code-block:: bash

   pip install dartpy -U

The following operating systems are currently supported:

+----------------+--------+--------+--------+--------+--------+
| Operating      | Python | Python | Python | Python | Python |
| System         | 3.7    | 3.8    | 3.9    | 3.10   | 3.11   |
+================+========+========+========+========+========+
| Linux x86_64   |   O    |   O    |   O    |   O    |   O    |
+----------------+--------+--------+--------+--------+--------+
| Linux arm64    |   X    |   X    |   X    |   X    |   O    |
+----------------+--------+--------+--------+--------+--------+
| macOS x86_64   |   X    |   O    |   O    |   O    |   O    |
+----------------+--------+--------+--------+--------+--------+
| macOS arm64    |   X    |   O    |   X    |   O    |   O    |
+----------------+--------+--------+--------+--------+--------+
| Windows x86_64 |   X    |   O    |   O    |   O    |   O    |
+----------------+--------+--------+--------+--------+--------+
| Windows arm64  |   X    |   X    |   X    |   X    |   O    |
+----------------+--------+--------+--------+--------+--------+

.. note::

   This table may not be up-to-date. For the latest information on the
   availability of the Python bindings for DART, please refer to the dartpy
   package on PyPI: https://pypi.org/project/dartpy/. If you would like to use
   dartpy on an unsupported OS or Python version, please let us know so we can
   consider adding support.

C++
---

Ubuntu
~~~~~~

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
~~~~~

To install DART on macOS, you can use Homebrew:

1. Install Homebrew if you haven't already:

.. code-block:: bash

   /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"

2. Install the `dartsim` formula:

.. code-block:: bash

   brew install dartsim

Windows
~~~~~~~

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
~~~~~~~~~~~~~~~~~~~~~~~~~

To install DART on Arch Linux using the `yay` package manager, you can use the
following commands:

1. Update your package list:

.. code-block:: bash

   yay -Syu

2. Install the `libdart` package:

.. code-block:: bash

   yay -S libdart

FreeBSD (experimental)
~~~~~~~~~~~~~~~~~~~~~~

To install DART on FreeBSD, you can use the following commands:

1. Update your package list:

.. code-block:: bash

   pkg update

2. Install the `dartsim` package:

.. code-block:: bash

   pkg install dartsim
