"""Construct native ownership cycles for the strict GC probe."""

import dartpy as dart


def make_case(kind):
    if kind == "Function->Problem":

        class Function(dart.optimizer.Function):
            def eval(self, x):
                return 0.0

        child = Function()
        owner = dart.optimizer.Problem(1)
        owner.setObjective(child)
    elif kind == "Problem->GradientDescentSolver":

        class Problem(dart.optimizer.Problem):
            pass

        child = Problem(1)
        owner = dart.optimizer.GradientDescentSolver(child)
    elif kind == "CollisionFilter->CollisionOption":

        class Filter(dart.collision.BodyNodeCollisionFilter):
            pass

        child = Filter()
        owner = dart.collision.CollisionOption()
        owner.collisionFilter = child
    elif kind == "ResourceRetriever->CompositeResourceRetriever":

        class Retriever(dart.utils.DartResourceRetriever):
            pass

        child = Retriever()
        owner = dart.utils.CompositeResourceRetriever()
        owner.addDefaultRetriever(child)
    elif kind == "Constraint->ConstraintSolver":

        class Constraint(dart.constraint.BallJointConstraint):
            pass

        skeleton = dart.dynamics.Skeleton()
        _, first = skeleton.createFreeJointAndBodyNodePair()
        _, second = skeleton.createFreeJointAndBodyNodePair()
        child = Constraint(first, second, [0, 0, 0])
        child.skeleton = skeleton
        owner = dart.constraint.BoxedLcpConstraintSolver()
        owner.addConstraint(child)
    else:
        assert kind == "Solver->InverseKinematics"

        class Solver(dart.optimizer.Solver):
            def solve(self):
                return True

            def getType(self):
                return "cycle_probe"

            def clone(self):
                return self

        skeleton = dart.dynamics.Skeleton()
        _, body = skeleton.createFreeJointAndBodyNodePair()
        child = Solver()
        child.skeleton = skeleton
        owner = body.getOrCreateIK()
        owner.setSolver(child)
    child.owner = owner
    return child, owner
