"""Strict ownership-cycle oracle, including public fields and clear-order stress."""

import argparse
import gc
import json
import weakref

import dartpy as dart
from probe_cycles import make_case

KINDS = (
    "Function->Problem",
    "Problem->GradientDescentSolver",
    "CollisionFilter->CollisionOption",
    "ResourceRetriever->CompositeResourceRetriever",
    "Constraint->ConstraintSolver",
    "Solver->InverseKinematics",
    "Shape->World",
    "SimpleFrame->World",
)


def case(kind):
    if kind not in KINDS[-2:]:
        return make_case(kind)
    owner = dart.simulation.World()
    if kind == "Shape->World":

        class Shape(dart.dynamics.BoxShape):
            pass

        child = Shape([1, 1, 1])
        skeleton = dart.dynamics.Skeleton()
        skeleton.createFreeJointAndBodyNodePair()[1].createShapeNode(child)
        owner.addSkeleton(skeleton)
    else:

        class Frame(dart.dynamics.SimpleFrame):
            pass

        child = Frame()
        owner.addSimpleFrame(child)
    child.owner = owner
    return child, owner


def probe(trials=20):
    rows = []
    for kind in KINDS:
        refs = []
        for _ in range(trials):
            child, owner = case(kind)
            refs.append((weakref.ref(child), weakref.ref(owner)))
            del child, owner
        gc.collect()
        retained = sum(
            child() is not None or owner() is not None for child, owner in refs
        )
        rows.append({"kind": kind, "collected": trials - retained, "trials": trials})
        # Keep a failing probe's shutdown diagnostic useful without masking its result.
        for child, _ in refs:
            if child() is not None:
                child().owner = None
        gc.collect()
    assert all(row["collected"] == trials for row in rows), rows
    return rows


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trials", type=int, default=20)
    args = parser.parse_args()
    print(json.dumps(probe(args.trials), indent=2))
