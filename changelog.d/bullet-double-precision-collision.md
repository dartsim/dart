Section: Collision

  * The Bullet detector no longer drops or tilts contacts of centimeter-scale
    shapes, which let small boxes sink into or fall through heightmaps, now
    that DART links double-precision Bullet (see Build). Heightmaps also work
    with double-precision Bullet, where they read garbage heights, and
    `MultiSphereConvexHullShape` spheres smaller than 4 cm get correct
    contacts (see
    [gz-physics#1117](https://github.com/gazebosim/gz-physics/issues/1117)).
    [#3868](https://github.com/dartsim/dart/pull/3868)
