Python API Reference
====================

The full ``dartpy`` API reference is authored under ``docs/python_api/modules``
and included into this site through the shims under
``docs/readthedocs/dartpy/api/modules``. Sphinx imports a live ``dartpy`` module
when one is available; otherwise ``docs/readthedocs/conf.py`` falls back to the
committed stubs under ``python/stubs/dartpy`` so Read the Docs renders the
current repository API shape without pinning an older release wheel.

.. note::
   Local ``pixi run docs-build`` uses the same fallback path unless the current
   shell already exposes a built ``dartpy`` module. Use ``pixi run api-docs-py``
   when you need a local API build backed by the freshly compiled extension.

Getting Started
---------------

To explore the bindings from a source checkout:

.. code-block:: bash

   pixi run build
   PYTHONPATH=build/default/cpp/Release/python pixi run python - <<'PY'
   import dartpy as dart
   world = dart.World()
   print(world.gravity)
   PY

Module Reference
----------------

.. toctree::
   :maxdepth: 1
   :titlesonly:

   api/modules/dartpy
   api/modules/simulation
   api/modules/io
   api/modules/gui
