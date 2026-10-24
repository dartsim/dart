Section: Build

  * Enabling IO requires tinyxml2 and urdfdom. SDF support is optional:
    missing sdformat warns and builds IO without SDF, and
    `DART_SKIP_SDFORMAT=ON` disables it deliberately. Request the `io-sdf`
    CMake component to require SDF support. Shared IO consumers no longer
    require sdformat development files; static consumers still do. Python
    SDF loading raises `RuntimeError` when SDF support is disabled.
    [#3889](https://github.com/dartsim/dart/pull/3889)
