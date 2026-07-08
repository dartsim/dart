Installation
============

DART 7 source build
-------------------

DART 7's Python-first API is available from a source checkout today. Public
package channels currently resolve stable DART 6 artifacts, or no usable
non-yanked DART 7 wheel, so do not use package-manager commands for the DART 7
examples until a non-yanked DART 7 ``dartpy`` wheel is published.

.. code-block:: bash

   git clone https://github.com/dartsim/dart.git
   cd dart
   pixi install
   pixi run build

After a non-yanked DART 7 wheel is published on PyPI, use pre-release
resolution to select it instead of the stable DART 6 package line:

.. code-block:: bash

   uv add dartpy --prerelease allow
   pip install dartpy --pre

The default ``pixi add dartpy`` and ``conda install -c conda-forge dartpy``
commands currently install the stable DART 6 package line.

Supported platforms
-------------------

The tracked DART 7 wheel workflow builds these configurations:

.. list-table::
   :header-rows: 1
   :widths: 50 50

   * - Platform / Python
     - Status
   * - Linux / CPython 3.14
     - Built by ``publish_dartpy.yml`` and installable from PyPI when the
       matching tag is published
   * - macOS / CPython 3.14
     - Built by ``publish_dartpy.yml`` and installable from PyPI when the
       matching tag is published
   * - Windows / CPython 3.14
     - Built by ``publish_dartpy.yml`` and installable from PyPI when the
       matching tag is published
   * - Other CPython versions
     - Build from source with a matching Python 3.14 toolchain

.. note::

   Wheel versions are sourced from ``package.xml``. DART 7 wheels are usable
   from PyPI only after a non-yanked DART 7 release is published. Until then,
   build from source for this guide's DART 7 examples. For the most up-to-date
   published availability, check the `dartpy project page on PyPI
   <https://pypi.org/project/dartpy/>`_.

Building from source
--------------------

If you need a configuration that is not covered by the pre-built packages, build
the Python bindings locally by following the instructions in
:doc:`../python_api_reference` or the developer knowledge base. Building
requires a full DART build with the nanobind-based `dartpy` target enabled and a
matching Python toolchain.
