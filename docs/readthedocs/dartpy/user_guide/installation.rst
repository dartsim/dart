Installation
============

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
