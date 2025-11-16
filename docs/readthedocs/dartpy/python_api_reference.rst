Python API Reference
=====================

The full dartpy API reference now builds directly on Read the Docs, so you no
longer need to leave the site or rely on a separate GitHub Pages deployment.
This page renders the documentation straight from the published ``dartpy``
wheel, which means the members, signatures, and type hints always match the
latest PyPI release.

.. admonition:: How these docs are generated

   * RTD installs the ``dartpy`` wheel listed in :file:`docs/readthedocs/requirements.txt`.
   * Sphinx loads the modules under :file:`docs/python_api/` and auto-documents
     them using the installed package.
   * Local builds can use ``pixi run docs-build`` or ``pixi run api-docs-py``
     for the same result.

Getting Started
---------------

To explore the bindings locally:

.. code-block:: bash

   pip install dartpy
   python - <<'PY'
   import dartpy as dart
   world = dart.simulation.World()
   print(world.getGravity())
   PY

The sections below mirror the module layout from :file:`docs/python_api/`.

Module Reference
----------------

.. toctree::
   :maxdepth: 2
   :titlesonly:

   /dartpy/api/modules/common
   /dartpy/api/modules/math
   /dartpy/api/modules/dynamics
   /dartpy/api/modules/simulation
   /dartpy/api/modules/collision
   /dartpy/api/modules/constraint
   /dartpy/api/modules/optimizer
   /dartpy/api/modules/utils
   /dartpy/api/modules/gui

Indices and tables
------------------

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`
