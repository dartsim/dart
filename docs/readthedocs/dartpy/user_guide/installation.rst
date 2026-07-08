Installation
============

Quick install commands
----------------------

Use your preferred package manager to add ``dartpy`` to an existing environment.
For DART 7, opt into PyPI pre-releases so the Python-first API in this guide is
selected instead of the stable DART 6 package line:

.. code-block:: bash

   uv add dartpy --prerelease allow  # uv (recommended for Python-first projects)
   pip install dartpy --pre          # PyPI wheels, CPython 3.14
   pixi add dartpy                   # stable DART 6 package line today
   conda install -c conda-forge dartpy

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

   Wheel versions are sourced from ``package.xml``. Use the ``--pre`` flag with
   ``pip`` for DART 7 pre-release tags; otherwise package managers can fall
   back to the stable DART 6 package line. For the most up-to-date published
   availability, check the `dartpy project page on PyPI
   <https://pypi.org/project/dartpy/>`_.

Building from source
--------------------

If you need a configuration that is not covered by the pre-built packages, build
the Python bindings locally by following the instructions in
:doc:`../python_api_reference` or the developer knowledge base. Building
requires a full DART build with the nanobind-based `dartpy` target enabled and a
matching Python toolchain.
