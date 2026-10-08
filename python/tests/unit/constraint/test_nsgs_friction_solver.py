import dartpy as dart
import numpy as np
import pytest

Nsgs = dart.constraint.NsgsFrictionSolver


@pytest.mark.parametrize("law", [Nsgs.Law.Coulomb, Nsgs.Law.Associated, Nsgs.Law.Box])
def test_options_and_backend_selection(law):
    options = Nsgs.Options()
    assert options.law == Nsgs.Law.Coulomb
    assert options.boxForAnisotropic
    assert options.maxSweeps == 100
    assert options.tolerance == 1e-5

    options.law = law
    options.boxForAnisotropic = False
    options.maxSweeps = 37
    options.tolerance = 2e-6
    primary = Nsgs(options)
    primary.reserve(12)

    stored = primary.getOptions()
    assert stored.law == law
    assert not stored.boxForAnisotropic
    assert stored.maxSweeps == 37
    assert stored.tolerance == 2e-6
    stored.maxSweeps = 9
    assert primary.getOptions().maxSweeps == 37
    primary.setOptions(stored)
    assert primary.getOptions().maxSweeps == 9

    solver = dart.constraint.BoxedLcpConstraintSolver()
    assert solver.getBoxedLcpSolver().getType() == "DantzigBoxedLcpSolver"
    assert solver.getSecondaryBoxedLcpSolver().getType() == "PgsBoxedLcpSolver"
    solver.setBoxedLcpSolver(primary)
    assert solver.getBoxedLcpSolver() is primary
    assert primary.getType() == Nsgs.getStaticType() == "NsgsFrictionSolver"

    secondary = Nsgs()
    solver.setSecondaryBoxedLcpSolver(secondary)
    assert solver.getSecondaryBoxedLcpSolver() is secondary
    solver.setSecondaryBoxedLcpSolver(None)
    assert solver.getSecondaryBoxedLcpSolver() is None


def test_world_clone_preserves_independent_friction_options():
    world = dart.simulation.World()
    solver = world.getConstraintSolver()
    options = Nsgs.Options()
    options.law = Nsgs.Law.Associated
    options.boxForAnisotropic = False
    options.maxSweeps = 23
    options.tolerance = 4e-6
    primary = Nsgs(options)
    solver.setBoxedLcpSolver(primary)

    options.law = Nsgs.Law.Box
    options.maxSweeps = 11
    secondary = Nsgs(options)
    solver.setSecondaryBoxedLcpSolver(secondary)

    clone = world.clone()
    cloned_solver = clone.getConstraintSolver()
    cloned_primary = cloned_solver.getBoxedLcpSolver()
    cloned_secondary = cloned_solver.getSecondaryBoxedLcpSolver()
    assert isinstance(cloned_primary, Nsgs)
    assert isinstance(cloned_secondary, Nsgs)
    assert cloned_primary is not primary
    assert cloned_secondary is not secondary
    cloned_options = cloned_primary.getOptions()
    assert cloned_options.law == Nsgs.Law.Associated
    assert not cloned_options.boxForAnisotropic
    assert cloned_options.maxSweeps == 23
    assert cloned_options.tolerance == 4e-6
    assert cloned_secondary.getOptions().law == Nsgs.Law.Box
    assert cloned_secondary.getOptions().maxSweeps == 11
    cloned_options.maxSweeps = 7
    cloned_primary.setOptions(cloned_options)
    assert primary.getOptions().maxSweeps == 23


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


@pytest.mark.parametrize("law", [Nsgs.Law.Coulomb, Nsgs.Law.Associated, Nsgs.Law.Box])
def test_live_stats_snapshots_reset_and_clone(law):
    world = _contact_world()
    options = Nsgs.Options()
    options.law = law
    backend = Nsgs(options)
    world.getConstraintSolver().setBoxedLcpSolver(backend)
    before = backend.getStats()
    assert isinstance(before, dart.constraint.FrictionSolveStats)
    assert before.numSolves == 0

    world.step()
    after = backend.getStats()
    assert before.numSolves == 0
    assert after.numSolves > 0
    assert after.numContacts > 0
    assert after.numFailed == 0
    assert after.numConverged + after.numAcceptedAtCap == after.numSolves
    assert after.numBoxContacts == (after.numContacts if law == Nsgs.Law.Box else 0)
    assert np.isfinite(after.maxViolation)

    cloned_backend = world.clone().getConstraintSolver().getBoxedLcpSolver()
    assert cloned_backend.getStats().numSolves == 0
    assert cloned_backend.getOptions().law == law
    backend.resetStats()
    reset = backend.getStats()
    for field in [
        "numSolves",
        "numConverged",
        "numAcceptedAtCap",
        "numFailed",
        "numContacts",
        "numBoxContacts",
        "numLocalFallbacks",
        "numIterations",
        "maxViolation",
    ]:
        assert getattr(reset, field) == 0
    assert after.numSolves > 0


def test_failed_solves_do_not_contribute_to_max_violation():
    world = _contact_world()
    options = Nsgs.Options()
    options.maxSweeps = -1
    backend = Nsgs(options)
    world.getConstraintSolver().setBoxedLcpSolver(backend)

    world.step()
    failed = backend.getStats()
    assert failed.numFailed == failed.numSolves > 0
    assert failed.numConverged == failed.numAcceptedAtCap == 0
    assert failed.maxViolation == 0

    options.maxSweeps = 100
    backend.setOptions(options)
    world.step()
    accepted = backend.getStats()
    assert accepted.numSolves > failed.numSolves
    assert accepted.numFailed == failed.numFailed
    assert accepted.numConverged + accepted.numAcceptedAtCap > 0
    assert np.isfinite(accepted.maxViolation)
