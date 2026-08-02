Research Papers And References
==============================

Use this page to cite DART and to find research papers that are directly
referenced by DART 6 documentation, examples, or experimental implementation
work.

DART
----

If you use DART in an academic publication, please cite:

.. code-block:: bib

   @article{Lee2018,
     doi = {10.21105/joss.00500},
     url = {https://doi.org/10.21105/joss.00500},
     year = {2018},
     month = {Feb},
     publisher = {The Open Journal},
     volume = {3},
     number = {22},
     pages = {500},
     author = {Jeongseok Lee and Michael X. Grey and Sehoon Ha and Tobias Kunz and Sumit Jain and Yuting Ye and Siddhartha S. Srinivasa and Mike Stilman and C. Karen Liu},
     title = {{DART}: Dynamic Animation and Robotics Toolkit},
     journal = {The Journal of Open Source Software}
   }

Exact Reduced Coulomb Friction
------------------------------

DART 6 includes experimental development work toward exact reduced Coulomb
friction using the forward-backward-forward splitting architecture described by
Hongcheng Song, Ye Fan, Uri M. Ascher, and Dinesh K. Pai:

.. code-block:: bib

   @article{Song2026ExactReducedCoulombFriction,
     title = {A Splitting Architecture for Exact Reduced Coulomb Friction},
     author = {Hongcheng Song and Ye Fan and Uri M. Ascher and Dinesh K. Pai},
     journal = {Computer Graphics Forum},
     volume = {45},
     number = {8},
     year = {2026},
     note = {ACM SIGGRAPH / Eurographics Symposium on Computer Animation 2026}
   }

Project resources:

* `Project page <https://www.cs.ubc.ca/research/fbf-friction/>`_
* `Paper PDF <https://www.cs.ubc.ca/research/fbf-friction/paper.pdf>`_
* `Video <https://www.youtube.com/watch?v=5THad4PAGmI>`_

The DART 6 implementation is opt-in experimental work on the release branch.
The default DART 6 contact solver remains the boxed-LCP path until parity and
compatibility evidence justify broader exposure.

The paper's masonry-arch scenes credit the Rigid-IPC dataset. DART's
paper-parity arch fixtures use the arch geometry published by the Rigid-IPC
reference implementation (MIT license), which accompanies:

.. code-block:: bib

   @article{Ferguson2021RigidIPC,
     title = {Intersection-free Rigid Body Dynamics},
     author = {Zachary Ferguson and Minchen Li and Teseo Schneider and
               Francisca Gil-Ureta and Timothy Langlois and Chenfanfu Jiang and
               Denis Zorin and Danny M. Kaufman and Daniele Panozzo},
     journal = {ACM Transactions on Graphics (SIGGRAPH)},
     volume = {40},
     number = {4},
     year = {2021}
   }

* `Rigid-IPC repository <https://github.com/ipc-sim/rigid-ipc>`_

Optional external comparison baselines for the exact-Coulomb benchmarks (for
example MuJoCo) are scoped to tests, examples, and benchmarks only and are
never core DART library dependencies.
