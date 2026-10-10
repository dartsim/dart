.. DART documentation master file, created by
   sphinx-quickstart on Sun Feb 19 22:01:28 2023.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Welcome to DART documentation!
==============================

.. admonition:: You are reading the DART 6 LTS documentation
   :class: important

   This site documents the **DART 6** compatibility line. The current stable
   release is **DART 6.19.5**; ``main`` is the development branch for the next
   release, currently DART 6.20, and receives all new patches.
   The source version is read from ``package.xml``, and release notes are tracked
   in ``CHANGELOG.md``.

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

Getting started
---------------

* **C++:** Follow the :doc:`installation guide <dart/user_guide/installation>`
  and work through the :doc:`tutorials <dart/user_guide/tutorials>`.
* **Python:** Install dartpy using the
  :doc:`installation guide <dartpy/user_guide/installation>` and run the
  :doc:`examples <dartpy/user_guide/examples>`.

Updates
-------

* 2026-10-04: DART version 6.19.5 released. See the
  `CHANGELOG <https://github.com/dartsim/dart/blob/main/CHANGELOG.md>`_.
* DART 6.20.0 is in progress on the
  `main branch <https://github.com/dartsim/dart/tree/main>`_.
* 2022-12-31: DART version 6.13.0 released.

Project Stats
-------------

Track GitHub interest over time with
`Star History <https://star-history.com/#dartsim/dart&type=date&legend=top-left>`_.
DART 6 benchmark history over commits is covered by the
:doc:`performance dashboard <community/performance_dashboard>`. After the first
dashboard publication, the hosted benchmark dashboard is available at
`dartsim.github.io/dart/performance/dart6/
<https://dartsim.github.io/dart/performance/dart6/>`_.

Social Media
------------

Stay updated with the latest news and developments about DART by following us
on `Twitter <https://twitter.com/dartsim_org>`_ and subscribing to our
`YouTube channel <https://www.youtube.com/@dartyoutube3531>`_.

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
   :maxdepth: 1
   :hidden:
   :caption: dart (C++)

   dart/user_guide/installation
   dart/user_guide/tutorials
   dart/developer_guide/build
   dart/developer_guide/contribution
   dart/developer_guide/code_style_guide

.. toctree::
   :maxdepth: 1
   :hidden:
   :caption: dartpy (Python)

   dartpy/user_guide/installation
   dartpy/user_guide/examples
   dartpy/user_guide/tutorials
   dartpy/developer_guide/build
   dartpy/developer_guide/contribution

.. toctree::
   :maxdepth: 1
   :hidden:
   :caption: Community

   community/who_uses_dart
   community/performance_dashboard
   license
