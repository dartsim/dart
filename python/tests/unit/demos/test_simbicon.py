"""SIMBICON rejects invalid Atlas models and retains its solver's World."""

import gc
import weakref

import dartpy as dart
import numpy as np
import pytest
from dartpy import _demos

simbicon = getattr(_demos, "simbicon", None)
soft_foot = getattr(_demos, "soft_foot_simbicon", None)
pytestmark = pytest.mark.skipif(
    simbicon is None, reason="dart-utils and dart-utils-urdf were compiled out"
)


def test_controller_state_changes_harnesses_and_reset():
    model = soft_foot.create_model(soft_foot.Feet.RIGID)
    controller = simbicon.Controller(model.atlas, model.world)
    controller.set_verbosity(False)
    initial_positions = model.atlas.getPositions().copy()
    assert controller.atlas is model.atlas
    assert controller.current_state_machine == "walking in place"
    assert isinstance(controller.current_state, str)
    for name in ("standing", "walking", "running", "walking in place"):
        controller.change_state_machine(name, model.world.getTime())
        assert controller.current_state_machine == name
        controller.update()
        model.world.step()

    solver = model.world.getConstraintSolver()
    original_count = solver.getNumConstraints()
    for body in ("pelvis", "left_foot", "right_foot"):
        getattr(controller, f"harness_{body}")()
        # Legacy foot harness methods allocate constraints without installing them.
        assert solver.getNumConstraints() == original_count + (body == "pelvis")
        getattr(controller, f"unharness_{body}")()
        assert solver.getNumConstraints() == original_count
    controller.reset_robot()
    np.testing.assert_array_equal(controller.atlas.getPositions(), initial_positions)
    assert isinstance(controller.is_allowing_control(), bool)


def test_unknown_state_machine_raises_value_error():
    model = soft_foot.create_model(soft_foot.Feet.RIGID)
    controller = model.controller
    previous = controller.current_state_machine
    with pytest.raises(ValueError, match="missing state machine"):
        controller.change_state_machine("missing state machine", 0.0)
    assert controller.current_state_machine == previous
    with pytest.raises(TypeError):
        controller.change_state_machine(4, 0.0)


@pytest.mark.parametrize("missing_argument", ["atlas", "world"])
def test_controller_rejects_null_inputs(missing_argument):
    model = soft_foot.create_model(soft_foot.Feet.RIGID)
    arguments = {"atlas": model.atlas, "world": model.world}
    arguments[missing_argument] = None
    with pytest.raises((TypeError, ValueError)):
        simbicon.Controller(**arguments)


@pytest.mark.parametrize("wrong_argument", ["atlas", "world"])
def test_controller_rejects_wrong_input_types(wrong_argument):
    model = soft_foot.create_model(soft_foot.Feet.RIGID)
    arguments = {"atlas": model.atlas, "world": model.world}
    arguments[wrong_argument] = object()
    with pytest.raises(TypeError):
        simbicon.Controller(**arguments)


def test_controller_rejects_empty_skeleton():
    with pytest.raises(ValueError):
        simbicon.Controller(dart.dynamics.Skeleton(), dart.simulation.World())


@pytest.mark.parametrize(
    "kind,name",
    [
        ("Dof", "l_leg_hpx"),
        ("Dof", "back_bky"),
        ("BodyNode", "pelvis"),
        ("BodyNode", "r_uleg"),
    ],
)
def test_controller_reports_missing_required_part(kind, name):
    model = soft_foot.create_model(soft_foot.Feet.RIGID)
    atlas = model.atlas.clone()
    getattr(atlas, f"get{kind}")(name).setName("renamed_required_part")
    with pytest.raises(ValueError, match=name):
        simbicon.Controller(atlas, model.world)


def test_controller_keeps_world_alive_after_gc():
    class World(dart.simulation.World):
        pass

    model = soft_foot.create_model(soft_foot.Feet.RIGID)
    atlas = model.atlas.clone()
    world = World()
    world.addSkeleton(atlas)
    controller = simbicon.Controller(atlas, world)
    reference = weakref.ref(world)
    del model, atlas, world
    gc.collect()
    assert reference() is not None
    for body in ("pelvis", "left_foot", "right_foot"):
        getattr(controller, f"harness_{body}")()
        getattr(controller, f"unharness_{body}")()
    controller.update()
    reference().step()
    assert np.isfinite(controller.atlas.getPositions()).all()
    del controller
    gc.collect()
    assert reference() is None


def test_model_controller_survives_dropping_model():
    model = soft_foot.create_model(soft_foot.Feet.SOFT)
    controller = model.controller
    del model
    gc.collect()
    controller.update()
    controller.world.step()
    controller.harness_pelvis()
    controller.unharness_pelvis()
    assert np.isfinite(controller.atlas.getPositions()).all()


def test_controller_world_python_backref_cycle_collects():
    class World(dart.simulation.World):
        pass

    model = soft_foot.create_model(soft_foot.Feet.RIGID)
    atlas = model.atlas.clone()
    world = World()
    world.addSkeleton(atlas)
    controller = simbicon.Controller(atlas, world)
    world.owner = controller
    references = weakref.ref(world), weakref.ref(controller)
    del model, atlas, world, controller
    gc.collect()
    collected = all(reference() is None for reference in references)
    if references[0]() is not None:
        references[0]().owner = None
    gc.collect()
    assert all(reference() is None for reference in references)
    assert collected, "Controller's retained World edge was invisible to GC"


def test_dropping_a_controller_removes_its_pelvis_harness():
    model = soft_foot.create_model(soft_foot.Feet.RIGID)
    solver = model.world.getConstraintSolver()
    original_count = solver.getNumConstraints()
    controller = simbicon.Controller(model.atlas, model.world)
    controller.harness_pelvis()
    assert solver.getNumConstraints() == original_count + 1
    del controller
    gc.collect()
    assert solver.getNumConstraints() == original_count
    # A model's own controller belongs to the model, which keeps its harness.
    model.controller.harness_pelvis()
    gc.collect()
    assert solver.getNumConstraints() == original_count + 1


def test_dropping_a_model_unharnesses_its_controller_in_a_surviving_world():
    model = soft_foot.create_model(soft_foot.Feet.RIGID)
    world = model.world
    solver = world.getConstraintSolver()
    original_count = solver.getNumConstraints()
    model.controller.harness_pelvis()
    assert solver.getNumConstraints() == original_count + 1
    del model
    gc.collect()
    assert solver.getNumConstraints() == original_count
    # No weld is left to reference the freed pelvis.
    world.removeAllSkeletons()
    world.step()
