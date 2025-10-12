Building dartpy from Source
===========================

For information on building dartpy (the Python bindings) from source, please refer to the
:doc:`main build guide </dart/developer_guide/build>` which includes instructions for
building DART with Python bindings enabled.

Prerequisites
-------------

Before building dartpy from source, ensure you have:

* Python 3.7 or higher
* pip
* The dependencies required to build DART (see the C++ build guide)

Building with Python Bindings
------------------------------

When configuring DART with CMake, enable the Python bindings:

.. code-block:: bash

   cmake -DDART_BUILD_DARTPY=ON ...

For detailed instructions, see the :doc:`C++ build guide </dart/developer_guide/build>`.
