Section: Build

  * Shared-library core consumers no longer need Bullet or ODE development
    packages. Shared-library consumers of their collision headers must request the
    `collision-bullet` or `collision-ode` CMake components, or the corresponding
    `dart-collision-bullet` or `dart-collision-ode` pkg-config modules. Static
    builds keep their existing dependency requirements.
    [#3952](https://github.com/dartsim/dart/pull/3952)
