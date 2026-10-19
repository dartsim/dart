"""Switch the box-stacking demo's backend while keeping solver borrowers alive."""

import dartpy as dart


def set_boxed_lcp_solver(world, kind):
    if world is None:
        raise ValueError("world must not be null")
    if not isinstance(world, dart.simulation.World):
        raise TypeError("world must be a World")
    if not isinstance(kind, str):
        raise TypeError("kind must be a string")
    if kind not in ("dantzig", "pgs"):
        raise ValueError("kind must be 'dantzig' or 'pgs'")
    solver = world.getConstraintSolver()
    if not isinstance(solver, dart.constraint.BoxedLcpConstraintSolver):
        raise ValueError("world must use a BoxedLcpConstraintSolver")
    backend = (
        dart.constraint.DantzigBoxedLcpSolver()
        if kind == "dantzig"
        else dart.constraint.PgsBoxedLcpSolver()
    )
    solver.setBoxedLcpSolver(backend)
    for i in range(world.getNumSkeletons()):
        skeleton = world.getSkeleton(i)
        if not skeleton.isMobile():
            continue
        if skeleton.isResting():
            skeleton.setResting(False)
            skeleton.set_island_index(-1)
        skeleton.set_sleep_candidate(False)
