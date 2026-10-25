"""The World facade coexists with the legacy bindings during migration."""

import gc
import weakref

import dartpy as dart
import numpy as np
import pytest


def test_keyword_construction_and_legacy_overloads():
    detector = dart.collision.DARTCollisionDetector()
    world = dart.World(
        name="configured",
        time_step=0.002,
        gravity=(1, 2, 3),
        collision_detector=detector,
        threads=2,
    )
    assert dart.World is dart.simulation.World
    assert dart.World.__module__ == "dartpy.simulation"
    assert world.name == world.getName() == "configured"
    assert world.time_step == world.getTimeStep() == 0.002
    np.testing.assert_array_equal(world.gravity, (1, 2, 3))
    assert world.collision.detector is detector
    assert (
        world.solver.threads
        == world.getConstraintSolver().getNumSimulationThreads()
        == 2
    )
    assert not world.is_compiled
    world.name = "renamed"
    assert world.getName() == "renamed"
    assert dart.World("positional").name == "positional"
    assert dart.World(name="named").name == "named"
    assert isinstance(
        dart.World().collision.detector, dart.collision.FCLCollisionDetector
    )
    assert isinstance(
        dart.World(collision_detector=None).collision.detector,
        dart.collision.FCLCollisionDetector,
    )
    with pytest.raises(TypeError):
        dart.World("mixed", time_step=0.002)


@pytest.mark.parametrize("time_step", [0, -0.001, np.nan, np.inf, -np.inf])
def test_invalid_time_step_is_rejected_before_mutation(time_step):
    with pytest.raises(ValueError, match="positive and finite"):
        dart.World(time_step=time_step)
    world = dart.World(time_step=0.002)
    world.compile()
    with pytest.raises(ValueError, match="positive and finite"):
        world.time_step = time_step
    assert world.time_step == world.getConstraintSolver().getTimeStep() == 0.002
    assert world.is_compiled


@pytest.mark.parametrize("gravity", [(1, 2), (1, 2, 3, 4), [[1, 2], [3, 4]]])
def test_invalid_gravity_shape_is_rejected_before_mutation(gravity):
    with pytest.raises(ValueError, match="length 3"):
        dart.World(gravity=gravity)
    world = dart.World()
    before = world.gravity
    with pytest.raises(ValueError, match="length 3"):
        world.gravity = gravity
    np.testing.assert_array_equal(world.gravity, before)


def test_parameter_properties_update_attached_models():
    world = dart.World()
    model = dart.dynamics.Skeleton()
    model.createFreeJointAndBodyNodePair()
    world.addSkeleton(model)
    world.compile()
    world.gravity = (0, 0, -4)
    world.time_step = 0.003
    assert world.is_compiled
    assert model.getTimeStep() == 0.003
    np.testing.assert_array_equal(model.getGravity(), world.gravity)


@pytest.mark.parametrize("compiled", [False, True])
def test_nested_edits_preserve_lazy_or_compiled_mode(compiled):
    world = dart.World()
    if compiled:
        assert world.compile() is None
    with world.edit():
        assert not world.is_compiled
        first = dart.dynamics.Skeleton("first")
        world.addSkeleton(first)
        with world.edit():
            world.addSkeleton(dart.dynamics.Skeleton("second"))
        assert not world.is_compiled
        assert world.getNumSkeletons() == 2
    assert world.is_compiled == compiled
    assert world.getSkeleton(0) is first
    if compiled:
        with pytest.raises(RuntimeError, match="^ModelFrozenError: "):
            world.removeSkeleton(first)
        assert world.hasSkeleton(first)



def raise_error(error):
    """Raise through a call, so analyzers see that a with block can continue."""
    raise error

@pytest.mark.parametrize("compiled", [False, True])
def test_exception_retains_edits_without_recompiling(compiled):
    world = dart.World()
    if compiled:
        world.compile()
    model = dart.dynamics.Skeleton("retained")
    error = RuntimeError("body failed")
    with pytest.raises(RuntimeError) as raised:
        with world.edit():
            with world.edit():
                world.addSkeleton(model)
                raise_error(error)
    assert raised.value is error
    assert not world.is_compiled
    assert world.getSkeleton("retained") is model
    # Native EditScope does not roll back: further edits are still permitted.
    world.removeSkeleton(model)
    world.compile()
    assert world.is_compiled


def test_caught_inner_exception_allows_outer_recompile():
    world = dart.World()
    world.compile()
    with world.edit():
        with pytest.raises(ValueError, match="caught"):
            with world.edit():
                raise_error(ValueError("caught"))
        assert not world.is_compiled
    assert world.is_compiled


def test_detector_and_threads_are_live_and_check_frozen_topology():
    world = dart.World()
    collision, solver = world.collision, world.solver
    assert isinstance(collision, dart.simulation.WorldCollision)
    assert isinstance(solver, dart.simulation.WorldSolver)
    detector = dart.collision.DARTCollisionDetector()
    collision.detector = detector
    assert world.getCollisionDetector() is detector
    replacement = dart.collision.FCLCollisionDetector()
    world.setCollisionDetector(replacement)
    assert collision.detector is replacement
    solver.threads = 3
    assert world.getNumSimulationThreads() == 3
    solver.threads = 0
    assert solver.threads >= 1
    world.compile()
    with pytest.raises(RuntimeError, match="^ModelFrozenError: "):
        collision.detector = detector
    assert collision.detector is replacement
    with world.edit():
        collision.detector = detector
        solver.threads = 2
    assert world.is_compiled
    assert world.collision.detector is detector
    assert solver.threads == 2
    with pytest.raises(TypeError):
        collision.detector = None
    with pytest.raises(TypeError):
        dart.simulation.WorldCollision()
    with pytest.raises(TypeError):
        dart.simulation.WorldSolver()


def test_failed_scope_exit_recompile_is_reported_on_next_compile():
    world = dart.World(collision_detector=dart.collision.DARTCollisionDetector())
    world.compile()
    model = dart.dynamics.Skeleton()
    _, body = model.createFreeJointAndBodyNodePair()
    heightmap = dart.dynamics.HeightmapShapef()
    heightmap.setHeightField(np.zeros((2, 2), dtype=np.float32))
    body.createShapeNode(heightmap).createCollisionAspect()
    with world.edit():
        world.addSkeleton(model)
    assert not world.is_compiled
    with pytest.raises(RuntimeError, match="heightmap|Heightmap"):
        world.compile()
    with world.edit():
        world.removeSkeleton(model)
    assert not world.is_compiled
    world.compile()
    assert world.is_compiled


def test_edit_context_enters_once_and_can_be_reused():
    world = dart.World()
    world.compile()
    context = world.edit()
    assert world.is_compiled
    with context as entered:
        assert entered is context
        with pytest.raises(RuntimeError, match="already active"):
            context.__enter__()
        with pytest.raises(RuntimeError, match="^WorldBusyError: "):
            world.compile()
    assert world.is_compiled
    with pytest.raises(RuntimeError, match="not active"):
        context.__exit__(None, None, None)
    with context:
        assert not world.is_compiled
    assert world.is_compiled


@pytest.mark.parametrize(
    "accessor", [lambda w: w.collision, lambda w: w.solver, lambda w: w.edit()]
)
def test_handles_retain_world_and_python_subclass_state(accessor):
    class World(dart.World):
        pass

    world = World(name="owned")
    reference = weakref.ref(world)
    handle = accessor(world)
    del world
    gc.collect()
    assert reference() is not None
    assert reference().name == "owned"
    del handle
    gc.collect()
    assert reference() is None


@pytest.mark.parametrize(
    "accessor", [lambda w: w.collision, lambda w: w.solver, lambda w: w.edit()]
)
def test_handle_cycles_are_visible_to_gc(accessor):
    class World(dart.World):
        pass

    world = World()
    reference = weakref.ref(world)
    world.handle = accessor(world)
    del world
    gc.collect()
    assert reference() is None
