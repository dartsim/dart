import dartpy as dart
import numpy as np

Fbf = dart.constraint.FbfFrictionSolver
_STATS_FIELDS = (
    "numSolves",
    "numConverged",
    "numAcceptedAtCap",
    "numFailed",
    "numContacts",
    "numBoxContacts",
    "numLocalFallbacks",
    "numIterations",
    "numInnerIterations",
    "numStepShrinks",
    "numInnerCaps",
    "maxViolation",
)


def test_options_and_backend_selection():
    options = Fbf.Options()
    assert options.boxForAnisotropic
    assert options.maxOuterIterations == 100
    assert options.tolerance == 1e-5
    assert options.stepScale == 0.5
    assert options.maxInnerSweeps == 20
    assert options.innerToleranceFactor == 0.1

    options.boxForAnisotropic = False
    options.maxOuterIterations = 37
    options.tolerance = 2e-6
    options.stepScale = 0.7
    options.maxInnerSweeps = 9
    options.innerToleranceFactor = 0.05
    primary = Fbf(options)
    primary.reserve(12)

    stored = primary.getOptions()
    assert not stored.boxForAnisotropic
    assert stored.maxOuterIterations == 37
    assert stored.tolerance == 2e-6
    assert stored.stepScale == 0.7
    assert stored.maxInnerSweeps == 9
    assert stored.innerToleranceFactor == 0.05
    stored.maxOuterIterations = 9
    assert primary.getOptions().maxOuterIterations == 37
    primary.setOptions(stored)
    assert primary.getOptions().maxOuterIterations == 9

    solver = dart.constraint.BoxedLcpConstraintSolver()
    assert solver.getBoxedLcpSolver().getType() == "DantzigBoxedLcpSolver"
    assert solver.getSecondaryBoxedLcpSolver().getType() == "PgsBoxedLcpSolver"
    solver.setBoxedLcpSolver(primary)
    assert solver.getBoxedLcpSolver() is primary
    assert primary.getType() == Fbf.getStaticType() == "FbfFrictionSolver"

    secondary = Fbf()
    solver.setSecondaryBoxedLcpSolver(secondary)
    assert solver.getSecondaryBoxedLcpSolver() is secondary
    solver.setSecondaryBoxedLcpSolver(None)
    assert solver.getSecondaryBoxedLcpSolver() is None


def test_world_clone_preserves_independent_friction_options():
    world = dart.simulation.World()
    solver = world.getConstraintSolver()
    primary_values = {
        "boxForAnisotropic": False,
        "maxOuterIterations": 71,
        "tolerance": 2e-7,
        "stepScale": 0.7,
        "maxInnerSweeps": 9,
        "innerToleranceFactor": 0.05,
    }
    secondary_values = {
        "boxForAnisotropic": True,
        "maxOuterIterations": 33,
        "tolerance": 3e-6,
        "stepScale": 0.4,
        "maxInnerSweeps": 5,
        "innerToleranceFactor": 0.2,
    }
    backends = []
    for values in (primary_values, secondary_values):
        options = Fbf.Options()
        for name, value in values.items():
            setattr(options, name, value)
        backends.append(Fbf(options))
    primary, secondary = backends
    solver.setBoxedLcpSolver(primary)
    solver.setSecondaryBoxedLcpSolver(secondary)

    clone = world.clone()
    cloned_solver = clone.getConstraintSolver()
    cloned_backends = (
        cloned_solver.getBoxedLcpSolver(),
        cloned_solver.getSecondaryBoxedLcpSolver(),
    )
    for backend, source, values in zip(
        cloned_backends, backends, (primary_values, secondary_values)
    ):
        assert isinstance(backend, Fbf)
        assert backend is not source
        for name, value in values.items():
            assert getattr(backend.getOptions(), name) == value
        assert all(getattr(backend.getStats(), field) == 0 for field in _STATS_FIELDS)

    options = primary.getOptions()
    options.maxOuterIterations = 7
    primary.setOptions(options)
    assert cloned_backends[0].getOptions().maxOuterIterations == 71


def _contact_world():
    world = dart.simulation.World()
    world.setGravity([0.0, 0.0, -9.81])
    world.getConstraintSolver().setCollisionDetector(
        dart.collision.DARTCollisionDetector()
    )
    for name, shape, height, mobile in [
        ("ground", dart.dynamics.BoxShape([2.0, 2.0, 0.2]), -0.1, False),
        ("sphere", dart.dynamics.SphereShape(0.1), 0.0999, True),
    ]:
        skeleton = dart.dynamics.Skeleton(name)
        joint, body = skeleton.createFreeJointAndBodyNodePair()
        shape_node = body.createShapeNode(shape)
        shape_node.createCollisionAspect()
        shape_node.createDynamicsAspect()
        joint.setTransform(dart.math.Isometry3(np.eye(3), [0.0, 0.0, height]))
        skeleton.setMobile(mobile)
        world.addSkeleton(skeleton)
    return world


def test_live_stats_snapshots_reset_and_clone():
    world = _contact_world()
    backend = Fbf()
    world.getConstraintSolver().setBoxedLcpSolver(backend)
    before = backend.getStats()
    assert isinstance(before, dart.constraint.FrictionSolveStats)
    assert all(getattr(before, field) == 0 for field in _STATS_FIELDS)

    world.step()
    after = backend.getStats()
    assert before.numSolves == 0
    assert after.numSolves > 0
    assert after.numContacts > 0
    assert after.numIterations > 0
    assert after.numInnerIterations > 0
    assert after.numFailed == 0
    assert after.numConverged + after.numAcceptedAtCap == after.numSolves
    assert after.numBoxContacts == 0
    assert np.isfinite(after.maxViolation)

    cloned_backend = world.clone().getConstraintSolver().getBoxedLcpSolver()
    assert isinstance(cloned_backend, Fbf)
    assert all(
        getattr(cloned_backend.getStats(), field) == 0 for field in _STATS_FIELDS
    )
    backend.resetStats()
    reset = backend.getStats()
    assert all(getattr(reset, field) == 0 for field in _STATS_FIELDS)
    assert after.numSolves > 0


def test_failed_solves_do_not_contribute_to_max_violation():
    world = _contact_world()
    options = Fbf.Options()
    options.maxOuterIterations = -1
    backend = Fbf(options)
    world.getConstraintSolver().setBoxedLcpSolver(backend)

    world.step()
    failed = backend.getStats()
    assert failed.numFailed == failed.numSolves > 0
    assert failed.numConverged == failed.numAcceptedAtCap == 0
    assert failed.maxViolation == 0

    options.maxOuterIterations = 100
    backend.setOptions(options)
    world.step()
    accepted = backend.getStats()
    assert accepted.numSolves > failed.numSolves
    assert accepted.numFailed == failed.numFailed
    assert accepted.numConverged + accepted.numAcceptedAtCap > 0
    assert np.isfinite(accepted.maxViolation)
