"""SIMBICON rejects invalid Atlas models and retains its solver's World."""

import gc
import weakref

import dartpy as dart
import numpy as np
import pytest
from examples.demos.models import simbicon, soft_foot_simbicon as soft_foot

requires_sdf = pytest.mark.skipif(
    not hasattr(dart, "utils")
    or not hasattr(dart.utils.SdfParser, "Options")
    or not hasattr(dart.utils, "DartLoader"),
    reason="SDF or URDF loaders are unavailable",
)


@requires_sdf
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


@requires_sdf
def test_unknown_state_machine_raises_value_error():
    model = soft_foot.create_model(soft_foot.Feet.RIGID)
    controller = model.controller
    previous = controller.current_state_machine
    with pytest.raises(ValueError, match="missing state machine"):
        controller.change_state_machine("missing state machine", 0.0)
    assert controller.current_state_machine == previous
    with pytest.raises(TypeError):
        controller.change_state_machine(4, 0.0)


@requires_sdf
@pytest.mark.parametrize("missing_argument", ["atlas", "world"])
def test_controller_rejects_null_inputs(missing_argument):
    model = soft_foot.create_model(soft_foot.Feet.RIGID)
    arguments = {"atlas": model.atlas, "world": model.world}
    arguments[missing_argument] = None
    with pytest.raises(TypeError):
        simbicon.Controller(**arguments)


@requires_sdf
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


@requires_sdf
@pytest.mark.parametrize(
    "kind,name",
    [
        ("Dof", "l_leg_hpx"),
        ("Dof", "r_leg_hpx"),
        ("Dof", "l_leg_hpy"),
        ("Dof", "r_leg_hpy"),
        ("Dof", "back_bky"),
        ("Dof", "l_leg_kny"),
        ("Dof", "r_leg_kny"),
        ("Dof", "l_leg_aky"),
        ("Dof", "r_leg_aky"),
        ("Dof", "l_arm_shx"),
        ("Dof", "r_arm_shx"),
        ("Dof", "l_arm_shy"),
        ("Dof", "r_arm_shy"),
        ("BodyNode", "pelvis"),
        ("BodyNode", "l_foot"),
        ("BodyNode", "r_foot"),
        ("BodyNode", "l_uleg"),
        ("BodyNode", "r_uleg"),
    ],
)
def test_controller_reports_missing_required_part(kind, name):
    model = soft_foot.create_model(soft_foot.Feet.RIGID)
    atlas = model.atlas.clone()
    getattr(atlas, f"get{kind}")(name).setName("renamed_required_part")
    with pytest.raises(ValueError, match=name):
        simbicon.Controller(atlas, model.world)


@requires_sdf
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


@requires_sdf
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


@requires_sdf
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


@requires_sdf
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


@requires_sdf
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


@requires_sdf
@pytest.mark.parametrize(
    "name", [-1, 0, 3, 4, 2**31 - 1, 2**31, 2**32 - 1, 2**63 - 1, 2**64 - 1]
)
def test_name_only_controller_rejects_numeric_indices_and_integer_limits(name):
    model = soft_foot.create_model(soft_foot.Feet.RIGID)
    controller = model.controller
    previous = controller.current_state_machine, controller.current_state
    with pytest.raises(TypeError):
        controller.change_state_machine(name, 0.0)
    assert (controller.current_state_machine, controller.current_state) == previous


@requires_sdf
@pytest.mark.parametrize("argument", ["atlas", "world"])
def test_controller_rejects_spoofed_native_owner_types(argument):
    model = soft_foot.create_model(soft_foot.Feet.RIGID)
    expected = dart.dynamics.Skeleton if argument == "atlas" else dart.simulation.World

    class Masquerade:
        @property
        def __class__(self):
            return expected

    arguments = {"atlas": model.atlas, "world": model.world}
    arguments[argument] = Masquerade()
    with pytest.raises(TypeError):
        simbicon.Controller(**arguments)


@requires_sdf
@pytest.mark.parametrize("exception", [RuntimeError, KeyboardInterrupt, SystemExit])
@pytest.mark.parametrize("protocol", ["__float__", "__index__"])
def test_time_protocol_failures_are_type_errors_before_unknown_gait_lookup(
    exception, protocol
):
    def fail(self):
        raise exception("conversion failed")

    value = type("BrokenNumber", (), {protocol: fail})()
    model = soft_foot.create_model(soft_foot.Feet.RIGID)
    controller = model.controller
    with pytest.raises(TypeError):
        controller.change_state_machine("missing state machine", value)
    assert controller.current_state_machine == "walking in place"


@requires_sdf
@pytest.mark.parametrize("time", [np.nan, np.inf, -np.inf, -1.0])
def test_gait_change_accepts_native_nonfinite_and_negative_begin_times(time):
    model = soft_foot.create_model(soft_foot.Feet.RIGID)
    model.controller.change_state_machine("running", time)
    assert model.controller.current_state_machine == "running"
    model.controller.update()
    assert np.isfinite(model.atlas.getForces()).all()


@requires_sdf
def test_string_and_float_subclasses_use_stored_values():
    class Name(str):
        def __str__(self):
            return "missing"

    class Time(float):
        def __float__(self):
            return 99.0

    model = soft_foot.create_model(soft_foot.Feet.RIGID)
    model.controller.change_state_machine(Name("running"), Time(0.125))
    assert model.controller.current_state_machine == "running"
    assert model.controller._current.begin_time == 0.125


@requires_sdf
def test_running_timer_is_strict_and_returning_to_gait_keeps_state_clock():
    model = soft_foot.create_model(soft_foot.Feet.RIGID)
    controller = model.controller
    model.atlas.setTimeStep(0.15)
    controller.change_state_machine("running", 0.0)
    controller.update()
    assert controller.current_state == "0"
    controller.update()
    assert controller.current_state == "0"
    controller.update()
    assert controller.current_state == "1"
    controller.update()
    controller.change_state_machine("standing", 4.0)
    controller.change_state_machine("running", 5.0)
    assert controller.current_state == "1"
    assert controller._current.elapsed == 0.0
    assert controller._current.current_state.elapsed == 0.3
    controller.update()
    assert controller.current_state == "0"


@requires_sdf
def test_controller_reset_only_restores_q_dq_and_retains_forces_gait_clock_harness():
    model = soft_foot.create_model(soft_foot.Feet.RIGID)
    controller = model.controller
    q, dq = model.atlas.getPositions().copy(), model.atlas.getVelocities().copy()
    controller.change_state_machine("running", 0.0)
    controller.harness_pelvis()
    controller.update()
    model.world.step()
    elapsed = controller._current.elapsed
    forces = model.atlas.getForces().copy()
    controller.reset_robot()
    np.testing.assert_array_equal(model.atlas.getPositions(), q)
    np.testing.assert_array_equal(model.atlas.getVelocities(), dq)
    np.testing.assert_array_equal(model.atlas.getForces(), forces)
    assert controller.current_state_machine == "running"
    assert controller._current.elapsed == elapsed
    assert model.world.getTime() == 0.001
    assert model.world.getConstraintSolver().getNumConstraints() == 1


@requires_sdf
def test_reset_restores_captured_dof_indices_and_leaves_new_dof_state():
    model = soft_foot.create_model(soft_foot.Feet.RIGID)
    atlas, controller = model.atlas, model.controller
    initial_q = atlas.getPositions().copy()
    initial_dq = atlas.getVelocities().copy()
    atlas.createRevoluteJointAndBodyNodePair()
    atlas.setPositions(np.r_[initial_q + 0.1, 0.2])
    atlas.setVelocities(np.r_[initial_dq + 0.3, 0.4])
    atlas.setAccelerations(np.arange(atlas.getNumDofs(), dtype=float))
    atlas.setForces(np.arange(atlas.getNumDofs(), dtype=float) + 1.0)
    atlas.setCommands(np.arange(atlas.getNumDofs(), dtype=float) + 2.0)
    untouched = (
        atlas.getAccelerations().copy(),
        atlas.getForces().copy(),
        atlas.getCommands().copy(),
    )
    controller.reset_robot()
    np.testing.assert_array_equal(atlas.getPositions(), np.r_[initial_q, 0.2])
    np.testing.assert_array_equal(atlas.getVelocities(), np.r_[initial_dq, 0.4])
    for before, after in zip(
        untouched, (atlas.getAccelerations(), atlas.getForces(), atlas.getCommands())
    ):
        np.testing.assert_array_equal(before, after)


@requires_sdf
@pytest.mark.parametrize(
    "height,allows",
    [
        (-0.7000000000000001, False),
        (-0.7, True),
        (0.3, True),
        (0.30000000000000004, False),
        (np.nan, True),
    ],
)
def test_height_cutoff_boundaries_and_nan_match_native(height, allows):
    model = soft_foot.create_model(soft_foot.Feet.RIGID)
    model.atlas.setPosition(4, height)
    assert model.controller.is_allowing_control() is allows
    if not allows:
        forces = np.arange(model.atlas.getNumDofs(), dtype=float)
        model.atlas.setForces(forces)
        model.controller.update()
        np.testing.assert_array_equal(model.atlas.getForces(), forces)


@requires_sdf
def test_harnessed_controller_world_cycle_collects_without_retaining_solver_wrapper():
    class World(dart.simulation.World):
        pass

    model = soft_foot.create_model(soft_foot.Feet.RIGID)
    atlas = model.atlas.clone()
    world = World()
    world.addSkeleton(atlas)
    controller = simbicon.Controller(atlas, world)
    controller.harness_pelvis()
    world.owner = controller
    references = weakref.ref(world), weakref.ref(atlas), weakref.ref(controller)
    del model, atlas, world, controller
    gc.collect()
    assert all(reference() is None for reference in references)
