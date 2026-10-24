Section: Build

  * Fix C++20 compilation of DART's headers, including the GUI backend, which
    is always C++20, against octomap 1.10.0 or older with libstdc++ 15 and
    against ODE fetched with `DART_USE_SYSTEM_ODE=OFF`. Both failed in builds
    that treat warnings as errors, the default except in Debug builds.
    [#3925](https://github.com/dartsim/dart/pull/3925)
