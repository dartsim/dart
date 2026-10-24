Section: Build

  * With system Bullet 3.17 or newer, DART links the double-precision
    `-float64` Bullet libraries when they are installed beside the
    single-precision ones, as on Ubuntu and conda-forge, and the installed
    CMake package and `dart.pc` require the same libraries.
    `DART_BULLET_FLOAT64=OFF` keeps single precision. A process must not load
    both Bullet precisions, so applications that load another
    single-precision Bullet must switch it to double precision or build DART
    with that option.
    [#3868](https://github.com/dartsim/dart/pull/3868)
