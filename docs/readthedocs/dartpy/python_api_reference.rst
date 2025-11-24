Python API Reference
====================

The full ``dartpy`` API reference is built directly on Read the Docs using the
wheel pinned in ``docs/readthedocs/requirements.txt``. Sphinx imports the
modules under ``docs/python_api/`` and auto-documents them using the installed
package. Local builds use the compiled extension in ``build/.../python`` so the
API pages always reflect the current nanobind bindings.

.. note::
   If a compatible wheel is unavailable, ``conf.py`` falls back to the stub
   package in ``python/stubs/dartpy`` so RTD still renders the module layout.
   Local builds can always use ``pixi run docs-build`` or ``pixi run api-docs-py``.

Getting Started
---------------

To explore the bindings locally:

.. code-block:: bash

   pip install dartpy
   python - <<'PY'
   import dartpy as dart
   world = dart.World()
   print(world.get_gravity())
   PY

Module Reference
----------------

.. note::
   The dartpy API is now flattened: core classes/functions live on the
   top-level ``dartpy`` package and file parsers under ``dartpy.io``. The
   legacy submodules below remain importable in DART 7.x for compatibility but
   are deprecated and scheduled for removal in DART 8.0. Prefer importing from
   ``dartpy`` / ``dartpy.io`` in new code.

The sections below mirror the legacy module layout from ``docs/python_api/`` (kept for compatibility):

.. toctree::
   :maxdepth: 1
   :titlesonly:

   api/modules/common
   api/modules/math
   api/modules/dynamics
   api/modules/simulation
   api/modules/collision
   api/modules/constraint
   api/modules/optimizer
   api/modules/utils
   api/modules/gui
