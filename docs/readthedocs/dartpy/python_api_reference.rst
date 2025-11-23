Python API Reference
====================

The complete ``dartpy`` API reference now lives on GitHub Pages so it can be
rendered directly from the compiled wheels that ship with each release. Read
the Docs focuses on the narrative guides only and therefore no longer attempts
to import ``dartpy`` during its builds—this avoids the ``stbrp_pack_rects``
linker errors that cropped up when the binary extension was unavailable.

.. note:: Why the API docs moved

   * CI publishes the API reference right after each build, so the rendered
     members always match the packaged wheel.
   * Read the Docs does not compile the bindings, and mocking the modules would
     result in empty pages.
   * The nanobind port (``dartpy``) shares the same public surface and ships
     its API docs with the built wheels.

Where to read the API docs
--------------------------

* `Current release (|python_api_url|) <|python_api_url|>`_
* `Latest main branch build <https://dartsim.github.io/dart/main-py/>`_

Local exploration
-----------------

You can continue browsing the API locally via the published wheels:

.. code-block:: bash

   pip install --pre dartpy
   python - <<'PY'
   import dartpy as dart
   world = dart.simulation.World()
   print(world.getGravity())
   PY

``pixi run api-docs-py`` produces the exact same API pages locally, which is
especially useful when iterating on new nanobind bindings.
