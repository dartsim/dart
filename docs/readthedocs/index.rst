.. DART documentation master file, created by
   sphinx-quickstart on Sun Feb 19 22:01:28 2023.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Welcome to DART documentation!
==============================

Introduction
------------

DART (Dynamic Animation and Robotics Toolkit) is a collaborative,
cross-platform, open-source library developed by the
`Graphics Lab <http://www.cc.gatech.edu/~karenliu/Home.html>`_ and
`Humanoid Robotics Lab <http://www.golems.org/>`_ at the
`Georgia Institute of Technology <http://www.gatech.edu/>`_, with ongoing
contributions from the
`Personal Robotics Lab <http://personalrobotics.cs.washington.edu/>`_ at the
`University of Washington <http://www.washington.edu/>`_ and the
`Open Source Robotics Foundation <https://www.osrfoundation.org/>`_. It provides
data structures and algorithms for kinematic and dynamic applications in
robotics and computer animation. DART stands out due to its accuracy and
stability, which are achieved through the use of generalized coordinates to
represent articulated rigid body systems and the application of Featherstone's
Articulated Body Algorithm to compute motion dynamics.

AI Docs (Interactive Q&A; Experimental)
---------------------------------------

The official documentation is the source of truth. If you want AI-assisted
Q&A and summaries, try:

* `DeepWiki <https://deepwiki.com/dartsim/dart>`_
* `NotebookLM <https://notebooklm.google.com/notebook/c0cfc8ce-17ae-415a-a615-44c4342f0da6>`_ (Google account required)

These tools are experimental and may be incomplete or occasionally outdated.

Documentation can lag behind the code as the project evolves; please report
outdated pages or errors in `GitHub Issues <https://github.com/dartsim/dart/issues>`_.

News and Announcements
----------------------

For updates and announcements, see the
`GitHub Discussions Announcements category <https://github.com/dartsim/dart/discussions/categories/announcements>`_.
Release notes are available in
`GitHub Releases <https://github.com/dartsim/dart/releases>`_ and the
`CHANGELOG <https://github.com/dartsim/dart/blob/main/CHANGELOG.md>`_.

Project Stats
-------------

Track GitHub interest over time with
`Star History <https://star-history.com/#dartsim/dart&type=date&legend=top-left>`_.

Social Media
------------

Stay updated with the latest news and developments about DART by following us
on `Twitter <https://twitter.com/dartsim_org>`_ and subscribing to our
`YouTube channel <https://www.youtube.com/@dartyoutube3531>`_.

Vision for the next version of DART
-----------------------------------

* Elevate the Python binding to a first-class component, ensuring full support
  and equivalent functionality to the C++ APIs, rather than remaining in an
  experimental stage.
* Modularize the library so that users can select specific components to use
  with minimal required dependencies, rather than having to use the entire
  library, including unnecessary parts.
* Utilize hardware accelerations, such as SIMD, multi-core CPUs, and GPUs,
  whenever available and enabled by the user, to maximize overall performance.
* Support both single and double precision, with options to compile the library
  for required scalar types or leave the template code uncompiled.
* Minimize dependencies to make the library usable without bringing in all
  transitive dependencies.
* Modernize implementation and public APIs to enable users to work with more
  intuitive and user-friendly APIs.
* Provide various resources, such as a quick start guide, examples, and
  tutorials, to lower the initial learning curve for users.

Citation
--------

If you use DART in an academic publication, please consider citing this
`JOSS Paper <https://doi.org/10.21105/joss.00500>`_
[`BibTeX <https://gist.github.com/jslee02/998b8809e3ae1b7aef6ef04dd2ad5e27>`_]

.. code-block:: bib

   @article{Lee2018,
     doi = {10.21105/joss.00500},
     url = {https://doi.org/10.21105/joss.00500},
     year  = {2018},
     month = {Feb},
     publisher = {The Open Journal},
     volume = {3},
     number = {22},
     pages = {500},
     author = {Jeongseok Lee and Michael X. Grey and Sehoon Ha and Tobias Kunz and Sumit Jain and Yuting Ye and Siddhartha S. Srinivasa and Mike Stilman and C. Karen Liu},
     title = {{DART}: Dynamic Animation and Robotics Toolkit},
     journal = {The Journal of Open Source Software}
   }


.. toctree::
   :maxdepth: 1
   :hidden:
   :caption: Home

   overview
   gallery

.. toctree::
   :maxdepth: 2
   :hidden:
   :caption: Key Topics

   Topics <topics/index>

.. toctree::
   :maxdepth: 1
   :hidden:
   :caption: dartpy (Python)

   dartpy/user_guide/installation
   dartpy/user_guide/examples
   API Reference <dartpy/python_api_reference>

.. toctree::
   :maxdepth: 1
   :hidden:
   :caption: DART (C++)

   dart/user_guide/installation
   dart/user_guide/migration_guide
   API Reference <dart/cpp_api_reference>

.. toctree::
   :maxdepth: 1
   :hidden:
   :caption: Community

   community/who_uses_dart
   license
