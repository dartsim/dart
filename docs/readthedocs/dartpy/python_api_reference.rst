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

.. toctree::
   :maxdepth: 1
   :titlesonly:

   api/modules/dartpy
   api/modules/utils
   api/modules/gui
