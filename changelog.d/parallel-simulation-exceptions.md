Section: Simulation

  * Exceptions thrown during multithreaded collision detection,
    constraint solving or World stepping now reach the caller instead of
    terminating the process, and the World stays usable.
    [#3825](https://github.com/dartsim/dart/pull/3825)
