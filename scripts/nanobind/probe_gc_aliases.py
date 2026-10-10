"""Report conservative collection when two owners copy one Python pin."""

import gc
import json
import weakref

import dartpy as dart

if __name__ == "__main__":
    retained = 0
    for _ in range(20):

        class Problem(dart.optimizer.Problem):
            pass

        child = Problem(1)
        properties = dart.optimizer.GradientDescentSolverProperties()
        properties.mProblem = child
        solver = dart.optimizer.GradientDescentSolver(properties)
        child.owner = (solver, properties)
        ref = weakref.ref(child)
        del child, solver, properties
        gc.collect()
        retained += ref() is not None
        if ref() is not None:
            ref().owner = None
        gc.collect()
        assert ref() is None
    print(
        json.dumps(
            {
                "case": "one Problem pin copied into Properties and Solver",
                "trials": 20,
                "retained_after_gc": retained,
                "released_after_explicit_cleanup": True,
            },
            indent=2,
        )
    )
