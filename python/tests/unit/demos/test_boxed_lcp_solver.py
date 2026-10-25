"""The Python chooser changes the backend without replacing its owning solver."""

import sys
from pathlib import Path

import dartpy as dart
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
from examples.demos.models.boxed_lcp_solver import set_boxed_lcp_solver


def _box(name, position, size, *, fixed=False):
    skeleton = dart.dynamics.Skeleton(name)
    if fixed:
        skeleton.setMobile(False)
    joint, body = (
        skeleton.createWeldJointAndBodyNodePair()
        if fixed
        else skeleton.createFreeJointAndBodyNodePair()
    )
    node = body.createShapeNode(dart.dynamics.BoxShape(size))
    node.createCollisionAspect()
    node.createDynamicsAspect()
    transform = dart.math.Isometry3()
    transform.set_translation(position)
    joint.setTransformFromParentBodyNode(transform)
    return skeleton


@pytest.mark.parametrize(
    "kind, backend",
    [("dantzig", "DantzigBoxedLcpSolver"), ("pgs", "PgsBoxedLcpSolver")],
)
def test_boxed_lcp_switch_preserves_solver_world_and_backend_borrowers(kind, backend):
    world = dart.simulation.World()
    skeleton = _box("box", [0, 0, 2], [1, 1, 1])
    world.addSkeleton(skeleton)
    world.setTimeStep(0.002)
    world.setNumSimulationThreads(2)
    original = world.getConstraintSolver()
    previous_backend = original.getBoxedLcpSolver()
    previous_type = previous_backend.getType()
    detector = original.getCollisionDetector()
    group = original.getCollisionGroup()
    secondary = original.getSecondaryBoxedLcpSolver()
    skeleton.setResting(True)
    assert set_boxed_lcp_solver(world, kind) is None
    assert world.getConstraintSolver() is original
    assert original.getBoxedLcpSolver().getType() == backend
    assert original.getBoxedLcpSolver() is not previous_backend
    assert previous_backend.getType() == previous_type
    assert original.getCollisionDetector() is detector
    assert original.getCollisionGroup() is group
    assert original.getSecondaryBoxedLcpSolver() is secondary
    assert original.getTimeStep() == world.getTimeStep() == 0.002
    assert original.getNumSimulationThreads() == world.getNumSimulationThreads() == 2
    assert world.getNumSkeletons() == 1 and world.getSkeleton(0) is skeleton
    assert not skeleton.isResting()
    world.step()
    assert np.isfinite(skeleton.getPositions()).all()


def test_boxed_lcp_switch_wakes_all_mobile_bodies_and_preserves_fixed_bodies():
    world = dart.simulation.World()
    for i in range(4):
        skeleton = _box(str(i), [i * 2, 0, 2], [1, 1, 1], fixed=i == 0)
        skeleton.set_sleep_candidate(i != 3)
        skeleton.setResting(i in (0, 2))
        skeleton.set_island_index(70 + i)
        world.addSkeleton(skeleton)
    set_boxed_lcp_solver(world, "pgs")
    assert world.getSkeleton(0).isResting()
    assert world.getSkeleton(0).is_sleep_candidate()
    assert world.getSkeleton(0).getIslandIndex() == 70
    for i in range(1, 4):
        assert not world.getSkeleton(i).isResting()
        assert not world.getSkeleton(i).is_sleep_candidate()
    assert [world.getSkeleton(i).getIslandIndex() for i in range(1, 4)] == [71, -1, 73]


@pytest.mark.parametrize("kind", ["", "Dantzig", "PGS", "other"])
def test_boxed_lcp_rejects_invalid_kinds_without_mutating_world(kind):
    world = dart.simulation.World()
    solver = world.getConstraintSolver()
    backend = solver.getBoxedLcpSolver()
    with pytest.raises(ValueError, match="kind must be"):
        set_boxed_lcp_solver(world, kind)
    assert world.getConstraintSolver() is solver
    assert solver.getBoxedLcpSolver() is backend


def test_boxed_lcp_rejects_null_world_and_wrong_types():
    with pytest.raises(ValueError, match="world must not be null"):
        set_boxed_lcp_solver(None, "dantzig")
    with pytest.raises(TypeError, match="world must be a World"):
        set_boxed_lcp_solver(object(), "dantzig")
    world = dart.simulation.World()
    solver = world.getConstraintSolver()
    backend = solver.getBoxedLcpSolver()
    for kind in (None, 1, ["dantzig"], b"dantzig"):
        with pytest.raises(TypeError, match="kind must be a string"):
            set_boxed_lcp_solver(world, kind)
        assert world.getConstraintSolver() is solver
        assert solver.getBoxedLcpSolver() is backend


def test_boxed_lcp_rejects_non_boxed_constraint_solver_before_mutation():
    class NonBoxedWorld(dart.simulation.World):
        def getConstraintSolver(self):
            return self.test_solver

    world = NonBoxedWorld()
    world.test_solver = object()
    original = world.test_solver
    for kind in ("dantzig", "pgs"):
        with pytest.raises(
            ValueError, match="world must use a BoxedLcpConstraintSolver"
        ):
            set_boxed_lcp_solver(world, kind)
        assert world.test_solver is original


def test_boxed_lcp_switch_restarts_dwell_for_awake_and_resting_mobile_bodies():
    world = dart.simulation.World()
    world.setNumSimulationThreads(1)
    world.setTimeStep(0.001)
    options = world.getDeactivationOptions()
    options.mEnabled = True
    options.mTimeUntilSleep = 0.05
    world.setDeactivationOptions(options)
    world.addSkeleton(_box("floor", [0, 0, -0.005], [10, 10, 0.01], fixed=True))
    skeleton = _box("box", [0, 0, 0.25], [1, 1, 0.5])
    world.addSkeleton(skeleton)
    # Wake after initial-equilibrium sleep credit, then accumulate a partial dwell.
    for _ in range(100):
        world.step()
    assert skeleton.isResting()
    set_boxed_lcp_solver(world, "dantzig")
    for _ in range(30):
        world.step()
    assert not skeleton.isResting()
    for kind in ("pgs", "dantzig", "pgs"):
        was_resting = skeleton.isResting()
        island = skeleton.getIslandIndex()
        set_boxed_lcp_solver(world, kind)
        assert not skeleton.isResting()
        assert not skeleton.is_sleep_candidate()
        assert skeleton.getIslandIndex() == (-1 if was_resting else island)
        for _ in range(50):
            world.step()
            assert not skeleton.isResting()
        world.step()
        assert skeleton.isResting()


@pytest.mark.parametrize("kind", ["dantzig", "pgs"])
def test_boxed_lcp_switch_repeats_finite_stacking_simulation(kind):
    states = []
    for _ in range(2):
        world = dart.simulation.World()
        world.setNumSimulationThreads(1)
        world.addSkeleton(_box("floor", [0, 0, -0.005], [10, 10, 0.01], fixed=True))
        for i in range(5):
            world.addSkeleton(_box(f"box{i}", [0, 0, 0.75 + i * 0.5], [1, 1, 0.5]))
        set_boxed_lcp_solver(world, kind)
        for _ in range(800):
            world.step()
        state = np.concatenate(
            [
                np.r_[
                    world.getSkeleton(i).getPositions(),
                    world.getSkeleton(i).getVelocities(),
                ]
                for i in range(1, 6)
            ]
        )
        assert np.isfinite(state).all()
        states.append(state)
    np.testing.assert_array_equal(*states)
