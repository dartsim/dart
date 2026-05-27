Research Papers
===============

DART is a research-focused simulator. This page collects papers that users
should cite when using DART itself or when relying on research methods that
DART implements, extends, or reproduces.

This page is intentionally more visible than internal planning documents. The
lower-level implementation plans remain useful for developers, but paper credit
and implementation status belong on the published documentation site.

DART Core Paper
---------------

If you use DART in an academic publication, please cite:

Jeongseok Lee, Michael X. Grey, Sehoon Ha, Tobias Kunz, Sumit Jain, Yuting Ye,
Siddhartha S. Srinivasa, Mike Stilman, and C. Karen Liu. "DART: Dynamic
Animation and Robotics Toolkit." *Journal of Open Source Software*, 3(22),
500, 2018. DOI: `10.21105/joss.00500 <https://doi.org/10.21105/joss.00500>`__.

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

Research Implementations And Credits
------------------------------------

The entries below identify research work that informs active or implemented
DART features. Each entry separates credit, current DART status, and the
remaining validation needed before stronger parity claims are appropriate.

Incremental Potential Contact
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The experimental deformable-body work in DART is inspired by and intended to
reproduce the method family introduced by:

Minchen Li, Zachary Ferguson, Teseo Schneider, Timothy R. Langlois, Denis
Zorin, Daniele Panozzo, Chenfanfu Jiang, and Danny M. Kaufman. "Incremental
Potential Contact: Intersection- and Inversion-free, Large-Deformation
Dynamics." *ACM Transactions on Graphics*, 39(4), Article 49, 2020. DOI:
`10.1145/3386569.3392425 <https://doi.org/10.1145/3386569.3392425>`__.

Related public resources:

* Paper/project page: `ipc-sim.github.io <https://ipc-sim.github.io/>`__
* Reference implementation: `ipc-sim/IPC <https://github.com/ipc-sim/IPC>`__
* Modern toolkit: `ipc-sim/ipc-toolkit <https://github.com/ipc-sim/ipc-toolkit>`__

.. code-block:: bib

   @article{Li2020IPC,
     author = {Minchen Li and Zachary Ferguson and Teseo Schneider and Timothy R. Langlois and Denis Zorin and Daniele Panozzo and Chenfanfu Jiang and Danny M. Kaufman},
     title = {Incremental Potential Contact: Intersection- and Inversion-free, Large-Deformation Dynamics},
     journal = {ACM Transactions on Graphics},
     volume = {39},
     number = {4},
     articleno = {49},
     year = {2020},
     doi = {10.1145/3386569.3392425}
   }

Current DART status:

* The implementation is experimental and DART-owned; DART does not vendor or
  link the upstream IPC repository as a runtime dependency.
* The first DART slice provides a public deformable-body facade, point-mass
  nodes, distance-spring edges, fixed nodes, an implicit step stage, an
  analytic static-ground barrier, focused tests, a benchmark surface, and a
  Filament GUI example.
* The current slice should be described as IPC-inspired deformable groundwork,
  not as full IPC parity, mesh IPC contact, or IPC friction.

Planned parity work:

* Mesh-backed deformables with tetrahedral and surface topology, density-based
  masses, Young's modulus, Poisson ratio, damping, neo-Hookean and
  fixed-corotational materials, backward Euler, implicit Newmark, boundary
  conditions, scripted motion, restart, and diagnostics.
* Point-triangle and edge-edge distances, derivatives, tangent bases,
  edge-edge mollifier, broad-phase candidate generation, adjacency filtering,
  conservative CCD line search, exact post-step audits, and inversion-aware
  filtering.
* Sparse projected Newton solve with local Hessian positive-semidefinite
  projection, adaptive barrier stiffness, complementarity/positivity checks,
  admissibility diagnostics, and robust fallback policies.
* Smoothed lagged friction with stiction velocity tolerance, tangent-basis
  lagging, normal-force lagging, friction iterations, and convergence tests.
* Replication of the IPC paper examples, upstream experimental scenes, tests,
  benchmarks, scaling studies, comparison scenes, and failure-mode scenes as
  DART tests, benchmarks, examples, or explicitly documented manual baselines.
* Long-horizon headless Filament evidence for promoted GUI examples, including
  nonblank checks, motion-difference checks, contact sheets or videos, and
  PR-linked visual artifacts.

The active implementation checklist lives in the repository planning document:
`PLAN-081: Deformable Implicit-Barrier Solver <https://github.com/dartsim/dart/blob/main/docs/plans/081-deformable-implicit-barrier-solver.md>`__.
