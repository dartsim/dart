Installation
============

Quick install commands
----------------------

Use your preferred package manager to add ``dartpy`` to an existing environment:

.. code-block:: bash

   uv add dartpy                 # uv (recommended for Python-first projects)
   pip install dartpy --pre      # PyPI wheels (Linux x86_64, CPython 3.12–3.14)
   pixi add dartpy               # Pixi environment
   conda install -c conda-forge dartpy

Supported platforms
-------------------

Pre-built wheels on PyPI currently cover the following configurations:

.. list-table::
   :header-rows: 1
   :widths: 50 50

   * - Platform / Python
     - Status
   * - Linux x86_64 / CPython 3.12–3.14
     - ✅ Published as ``dartpy`` wheels (7.0.0.dev0, ``pip install --pre``)
   * - Other CPython versions and platforms
     - ⚠️ Use conda-forge, pixi, or build from source (no recent wheels yet)

.. note::

   The latest PyPI upload is a pre-release (``7.0.0.dev0``). Use the ``--pre`` flag with ``pip`` if you want that build; otherwise pip falls back to the last stable 0.2.x wheel. For the most up-to-date availability, check the `dartpy project page on PyPI <https://pypi.org/project/dartpy/>`_.

Building from source
--------------------

If you need a configuration that is not covered by the pre-built packages, build the Python bindings locally by following the instructions in :doc:`../python_api_reference` or the developer onboarding guide. Building requires a full DART build with pybind11 enabled and a matching Python toolchain.
