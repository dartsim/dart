C++ API Reference
==================

The full Doxygen site for the C++ API now builds directly inside Read the Docs,
so it inherits the same branch and tag versioning controls as the rest of this
documentation set. Each RTD build runs Doxygen against the checked-out source
tree and publishes the generated HTML bundle alongside the rest of the pages.

.. admonition:: How this page works

   * ``conf.py`` renders a dedicated Doxyfile and executes Doxygen during the
     ``builder-inited`` step.
   * The HTML bundle is staged under ``_generated/cpp-api/`` (ignored by git)
     and then copied to ``cpp-api/`` so it is served by RTD under the current
     version.
   * This page immediately redirects to ``../cpp-api/index.html`` so the
     Doxygen-rendered reference opens as a standalone documentation section.

Direct View
-----------

.. ifconfig:: cpp_api_available

   .. raw:: html

      <p>Loading the generated C++ API reference…</p>
      <script>
        window.location.replace("../cpp-api/index.html");
      </script>
      <noscript>
        <p>JavaScript is disabled. <a href="../cpp-api/index.html">Open the C++ API reference manually.</a></p>
      </noscript>

.. ifconfig:: not cpp_api_available

   .. warning::

      This build skipped the embedded API viewer because Doxygen was not found
      in the environment. Install Doxygen (``pixi run config`` pulls it in) to
      regenerate the bundle locally.

Local Builds
------------

To preview these docs locally, make sure ``pixi`` has installed the toolchain
(``pixi run config``) and then run:

.. code-block:: bash

   pixi run docs-build

The command above mirrors Read the Docs by executing ``sphinx-build`` from
``docs/readthedocs/`` and drops the Doxygen site into
``docs/readthedocs/_generated/cpp-api``.
