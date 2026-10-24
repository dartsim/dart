Section: Simulation

  * Add C++ `World::captureState()` and `World::restoreState()`
    for dynamic World snapshots: positions, velocities, accelerations, forces,
    commands, body external forces, soft point-mass state and time. A snapshot
    restores into the same World or a structurally identical one, such as a
    clone; parameters and solver or contact history are not part of it.
    Restoring clears built-in detector contact history, solver warm starts
    and sleep history left by earlier steps, and may allocate with FCL or
    Bullet.
    Clones omit manual constraints; re-add them in the same order with
    endpoints mapped to the clone's bodies before restoring.
    Commands follow the target's actuator types and limits; EndEffector support
    flags remain target-owned.
    Manual ball, revolute, cylindrical and weld joint constraints are
    supported, and restoring resets their previous solve impulses; custom
    joints, unsupported stateful aspects and other manual constraints are
    rejected before any state changes.
    [#3816](https://github.com/dartsim/dart/pull/3816),
    [#3817](https://github.com/dartsim/dart/pull/3817),
    [#3827](https://github.com/dartsim/dart/pull/3827)
