Section: API

  * Add `MetaSkeleton::getPositions(Eigen::VectorXd&)` and matching overloads
    of `getCommands()`, `getVelocities()`, `getAccelerations()` and
    `getForces()`. They write into a caller's vector and do not allocate when
    it already has `getNumDofs()` entries, so code that reads state every step,
    such as gz-physics checking each model's positions for NaN, can reuse one
    buffer instead of allocating a new vector per call.
