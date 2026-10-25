Section: Build

  * Standalone source builds now fetch FCL 0.7.0 and a double-precision
    libccd 2.1 and install them with DART, so FCL and libccd need not be
    installed. Configure builds both with the GJK fix that the Linux wheels
    carry, which needs the `patch` program. FCL gets octree support when DART
    finds octomap. Set `DART_USE_SYSTEM_FCL=ON` or
    `DART_USE_SYSTEM_DEPENDENCIES=ON`, as the Pixi tasks do, to keep an
    installed FCL; package managers and Gazebo-facing builds should. Visual
    Studio and Xcode builds keep it by default, and builds with Eigen 5 need
    it.
    [#3966](https://github.com/dartsim/dart/pull/3966)
