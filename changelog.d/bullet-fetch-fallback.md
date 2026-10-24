Section: Build

  * Fix the Bullet 3.25 fetch fallback under CMake 4 and build and install
    only the double-precision collision and math libraries DART uses. Accept
    these libraries in installed DART consumers without requiring
    BulletDynamics or BulletSoftBody.
    [#3901](https://github.com/dartsim/dart/pull/3901)
