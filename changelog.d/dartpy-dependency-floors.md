Section: Python

  * dartpy requires nanobind 3.1.x and NumPy 1.26.4 or newer; NumPy 2.x is
    supported. The bundled nanobind and robin_map use a hash-verified archive;
    incompatible nanobind versions and internals fail at configure time. CMake
    discovers Python once through FindPython; `Python_EXECUTABLE` selects the
    interpreter.
    [#3892](https://github.com/dartsim/dart/pull/3892)
