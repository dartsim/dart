IkFast Support and Integration
==============================

IkFast is the analytic inverse kinematics generator that ships with
`OpenRAVE <https://github.com/rdiankov/openrave>`_. DART exposes the generated
solutions through :class:`dart::dynamics::SharedLibraryIkFast` so that an IKFast
shared library can be plugged into :class:`dart::dynamics::InverseKinematics`
instances. This page documents the current status of IkFast and how it fits into
DART.

.. contents::
   :local:
   :depth: 2

Current upstream status
-----------------------

* The OpenRAVE repository is still maintained. For example, commit
  ``ec22ecf`` (2024‑08‑16) at
  https://github.com/rdiankov/openrave/commit/ec22ecfaf006688cbc5ee0fdd8fa05d2c5676d37
  shows that contributions continue to land and the IkFast generator files (for
  example ``python/ikfast.py`` and ``python/ikfast_generator_cpp.py``) are still
  part of the tree.
* There are **no tagged releases or official binaries**; the GitHub releases
  feed at https://github.com/rdiankov/openrave/releases is empty. Likewise,
  ``apt-cache search openrave``, ``pip index versions openrave``, and
  https://api.anaconda.org/package/conda-forge/openrave currently report that no
  packages exist. Expect to build OpenRAVE from source (see ``INSTALL`` in the
  upstream repository) or rely on a community-provided Docker image if you need
  the generator.
* There is **no standalone ``ikfast`` Python package** on PyPI or the major
  conda channels. ``pip index versions ikfast`` returns "No matching
  distribution" and searching ``https://api.anaconda.org/search?name=ikfast``
  only surfaces ROS-specific plugins that already embed pre-generated solvers.
  You still need the OpenRAVE tree if you want to generate new IKFast C++
  sources.
* Because OpenRAVE does not distribute binaries, **DART keeps IkFast support on
  a best-effort basis**. The API surface inside DART is stable, but the tooling
  required to regenerate new IkFast solvers is outside of DART's release
  process.

Generating a solver today
-------------------------

1. Clone ``https://github.com/rdiankov/openrave`` and follow the ``INSTALL`` and
   ``docs`` instructions to build OpenRAVE for your platform. The generator
   requires Python 3 and SymPy, both of which are already vendored in the
   repository.
2. Import your manipulator into OpenRAVE (URDF, Collada, or OpenRAVE XML). The
   upstream samples under ``plugins/ikfastsolvers/`` illustrate the supported
   formats.
3. Run the generator scripts from the OpenRAVE workspace; the entry points are
   ``python/ikfast.py`` and ``python/ikfast_generator_cpp.py``. The IkFast
   manual inside ``python/ikfast.py`` documents the available IK types
   (Transform6D, TranslationDirection5D, etc.) and the CLI arguments for
   selecting the manipulator and fixing redundant joints. The generator emits a
   single ``ikfastXX.<iktype>.<parameters>.cpp`` file.
4. Compile that file into a shared library. The simplest approach is to follow
   the ``examples/wam_ikfast/ikfast/CMakeLists.txt`` template in this
   repository, which builds a ``Generated<Name>IkFast`` shared object with the
   ``IKFAST_NO_MAIN`` and ``IKFAST_CLIBRARY`` flags set.

A generated solver is only tied to kinematics: every time the URDF or joint
limits change, the IkFast source should be regenerated to stay consistent with
the robot description that DART loads.

Using IkFast within DART
------------------------

* Use :func:`dart::dynamics::InverseKinematics::setGradientMethod` with
  :class:`dart::dynamics::SharedLibraryIkFast`. You must pass the shared library
  path and two index lists: ``dofMap`` enumerates the DOFs solved by IkFast and
  ``freeDofMap`` enumerates the redundant joints that IkFast expects the user to
  set before solving (matching the ``free indices`` you chose during generator
  execution).
* ``tests/integration/io/test_IkFast.cpp`` shows how DART wires a generated WAM
  arm solver into an end-effector IK node, validates version strings, and
  iterates through the returned solutions.
* The ``examples/wam_ikfast`` sample demonstrates how to load the generated
  solver at runtime, feed solutions back into a skeleton, and visualize the
  result with OSG.

Support policy
--------------

* The :class:`dart::dynamics::IkFast` / :class:`dart::dynamics::SharedLibraryIkFast`
  APIs remain part of DART's C++ surface and have automated coverage in
  ``tests/integration/io/test_IkFast.cpp``. There is **no plan to deprecate**
  these entry points for DART 6.x, because downstream applications (including
  the WAM example) still rely on them.
* The feature is intentionally isolated: it depends on a shared library supplied
  by the user, and DART does not vend generators or binaries. This isolation
  lets us keep the API available even as upstream OpenRAVE packaging fluctuates,
  and it contains the maintenance cost if we eventually have to disable it on a
  specific platform.
* We will continue to re-evaluate IkFast during each major release. If upstream
  OpenRAVE becomes unavailable or unbuildable, we will mark the feature as
  deprecated in release notes **before** removing it and will keep the tests in
  place until that happens.

If you depend on IkFast, please let the maintainers know by filing an issue or
commenting on the `IkFast documentation ticket
<https://github.com/dartsim/dart/issues/1526>`_. Community feedback is what will
ultimately determine whether the analytical interface stays enabled or gets
retired in a future major version.
