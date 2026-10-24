Section: Breaking Changes

  * Remove IKFast support: the
    `dart::dynamics::IkFast` and `SharedLibraryIkFast` classes,
    `dart::dynamics::wrapCyclicSolution`, and the installed
    `dart/dynamics/IkFast.hpp`, `dart/dynamics/SharedLibraryIkFast.hpp`,
    `dart/dynamics/ikfast.h` and `dart/external/ikfast/ikfast.h` headers.
    Implement `InverseKinematics::Analytical` for a C++ analytical solver, which
    can load a generated solver through `dart::common::SharedLibrary`, or pass a
    Python solver such as ssik to `InverseKinematics.setPythonAnalytical`.
    [#3824](https://github.com/dartsim/dart/pull/3824)
