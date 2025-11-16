C++ API Reference
==================

The full Doxygen site for the C++ API now builds directly inside Read the Docs,
so it inherits the same branch and tag versioning controls as the rest of this
documentation set. Each RTD build runs Doxygen against the checked-out source
tree and publishes the generated HTML bundle alongside the rest of the pages.

.. admonition:: How this page works

   * ``conf.py`` renders a dedicated Doxyfile and executes Doxygen during the
     ``builder-inited`` step.
   * The HTML bundle is copied to ``cpp-api/`` so it is served by RTD under the
     current version.
   * The iframe below loads ``../cpp-api/index.html`` to keep the docs inside
     the main navigation shell while also exposing a direct link for the
     full-width experience.

Embedded Viewer
---------------

.. ifconfig:: cpp_api_available

   .. raw:: html

      <div class="dart-api-frame" style="width: 100%; height: 1200px; border: 1px solid #dfe2e5; border-radius: 4px; overflow: hidden;">
        <iframe
          src="../cpp-api/index.html"
          title="DART C++ API Reference"
          loading="lazy"
          style="width: 100%; height: 100%; border: 0;"
        ></iframe>
      </div>

   Prefer a dedicated tab? `Open the full Doxygen site <../cpp-api/index.html>`_.

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
``docs/readthedocs/cpp-api``.
