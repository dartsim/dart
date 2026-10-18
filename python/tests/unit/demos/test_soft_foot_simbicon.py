"""Soft-foot normalization, standing, reset, and exhaustive push recovery."""

import gc
import sys
import weakref
from pathlib import Path

import dartpy as dart
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
from examples.demos.models import soft_foot_simbicon as soft_foot


@pytest.mark.parametrize("feet_name", ["RIGID", "SOFT"])
def test_soft_foot_construction_push_steps_and_determinism(feet_name):
    feet = getattr(soft_foot.Feet, feet_name)
    lhs, rhs = [soft_foot.create_model(feet) for _ in range(2)]
    assert lhs.world.getNumSkeletons() == 2
    assert lhs.world.getTimeStep() == soft_foot.time_step
    assert lhs.feet == feet
    assert lhs.pelvis.getName() == "pelvis"
    assert lhs.left_foot.getName() == "l_foot"
    assert lhs.right_foot.getName() == "r_foot"
    assert soft_foot.is_upright(lhs)
    assert soft_foot.pelvis_height_y(lhs) - soft_foot.mean_foot_height_y(lhs) > (
        soft_foot.upright_gap
    )
    assert soft_foot.push_steps > 0
    assert soft_foot.default_push_magnitude > 0
    assert 0 < soft_foot.push_sweep_start < soft_foot.push_sweep_end
    assert soft_foot.push_sweep_step > 0
    assert callable(soft_foot.max_recoverable_push)
    initial_state = soft_foot.state_vector(lhs).copy()

    for model in (lhs, rhs):
        soft_foot.apply_push(model, [10.0, 0.0, 0.0], steps=3)
    for _ in range(8):
        soft_foot.step(lhs)
        soft_foot.prepare_step(rhs)
        rhs.world.step()
    assert lhs.force_duration == 0
    assert soft_foot.is_finite(lhs)
    assert soft_foot.is_finite(rhs)
    assert soft_foot.foot_contact_count(lhs) >= 0
    state = soft_foot.state_vector(lhs)
    assert state.ndim == 1
    assert state.dtype == np.dtype(np.float64)
    np.testing.assert_array_equal(state, soft_foot.state_vector(rhs))

    soft_foot.reset_model(lhs)
    assert lhs.world.getTime() == 0
    assert lhs.force_duration == 0
    np.testing.assert_array_equal(lhs.external_force, np.zeros(3))
    np.testing.assert_array_equal(soft_foot.state_vector(lhs), initial_state)


@pytest.mark.parametrize("property_name", ["pelvis", "left_foot", "right_foot"])
def test_soft_foot_borrowed_child_keeps_skeleton_alive(property_name):
    model = soft_foot.create_model(soft_foot.Feet.RIGID)
    body_name = {"pelvis": "pelvis", "left_foot": "l_foot", "right_foot": "r_foot"}[
        property_name
    ]
    existing = model.atlas.getBodyNode(body_name)
    child = getattr(model, property_name)
    assert child is existing
    del existing
    reference = weakref.ref(model)
    del model
    gc.collect()
    assert reference() is None
    assert child.getName()
    assert child.getSkeleton().getNumDofs() == 33
    assert np.isfinite(child.getWorldTransform().matrix()).all()
    del child
    gc.collect()
    assert reference() is None


@pytest.mark.parametrize("force, steps", [([np.nan, 0, 0], 10), ([1, 0, 0], -1)])
def test_apply_push_rejects_invalid_input(force, steps):
    model = soft_foot.create_model(soft_foot.Feet.RIGID)
    with pytest.raises(ValueError):
        soft_foot.apply_push(model, force, steps)
    assert model.force_duration == 0


def _full_state(model):
    values = [
        getattr(model.atlas, f"get{field}")()
        for field in ("Positions", "Velocities", "Accelerations", "Forces", "Commands")
    ]
    for body in model.atlas.getBodyNodes():
        values.append(body.getWorldTransform().matrix().ravel())
        if isinstance(body, dart.dynamics.SoftBodyNode):
            for i in range(body.getNumPointMasses()):
                point = body.get_point_mass(i)
                values.extend(
                    getattr(point, field)
                    for field in (
                        "positions",
                        "velocities",
                        "accelerations",
                        "forces",
                        "world_position",
                        "world_velocity",
                    )
                )
                values.append(np.array([point.is_colliding]))
    return np.concatenate(values).copy()


@pytest.fixture(scope="module")
def stands():
    outcomes = {}
    for feet in (soft_foot.Feet.RIGID, soft_foot.Feet.SOFT):
        traces, contacts, heights = [], [], []
        for _ in range(2):
            model = soft_foot.create_model(feet)
            trace = [_full_state(model)]
            counts = []
            for step in range(1, 1001):
                soft_foot.step(model)
                assert soft_foot.is_finite(model)
                assert soft_foot.is_upright(model)
                trace.append(_full_state(model))
                if step >= 500:
                    counts.append(soft_foot.foot_contact_count(model))
            traces.append(np.stack(trace))
            contacts.append(np.array(counts))
            heights.append(soft_foot.pelvis_height_y(model))
        np.testing.assert_array_equal(*traces)
        np.testing.assert_array_equal(*contacts)
        assert heights[0] == heights[1]
        assert len(contacts[0]) == 501
        outcomes[feet] = contacts[0]
    return outcomes


def test_both_feet_stand_repeat_exactly_and_soft_contacts_exceed_rigid(stands):
    rigid, soft = stands[soft_foot.Feet.RIGID], stands[soft_foot.Feet.SOFT]
    assert soft.sum() >= rigid.sum()
    assert soft.mean() >= 1.5 * rigid.mean()


def _live_collision_surfaces(model, foot):
    group = model.world.getConstraintSolver().getCollisionGroup()
    expected = []
    for i in range(model.world.getNumSkeletons()):
        for body in model.world.getSkeleton(i).getBodyNodes():
            for node in body.getShapeNodes():
                assert group.hasShapeFrame(node) == node.hasCollisionAspect()
                if node.hasCollisionAspect():
                    expected.append(node)
    # Every expected frame is present; exact cardinality excludes stale extras.
    assert group.getNumShapeFrames() == len(expected) == 30
    return [node for node in expected if node.getBodyNodePtr() is foot]


def _combined_rest_inertia(body):
    inertia = body.getInertia()
    mass = inertia.getMass()
    com = inertia.getLocalCOM().copy()
    origin = inertia.getMoment() + mass * (
        np.dot(com, com) * np.eye(3) - np.outer(com, com)
    )
    weighted = mass * com
    if isinstance(body, dart.dynamics.SoftBodyNode):
        for i in range(body.getNumPointMasses()):
            point = body.get_point_mass(i)
            rest, point_mass = point.rest_position, point.mass
            origin += point_mass * (
                np.dot(rest, rest) * np.eye(3) - np.outer(rest, rest)
            )
            weighted += point_mass * rest
            mass += point_mass
    com = weighted / mass
    moment = origin - mass * (np.dot(com, com) * np.eye(3) - np.outer(com, com))
    return mass, com, moment


def _observed_com(model):
    mass = 0.0
    weighted = np.zeros(3)
    for body in model.atlas.getBodyNodes():
        core_mass = dart.dynamics.BodyNode.getMass(body)
        weighted += core_mass * body.getCOM()
        mass += core_mass
        if isinstance(body, dart.dynamics.SoftBodyNode):
            for i in range(body.getNumPointMasses()):
                point = body.get_point_mass(i)
                weighted += point.mass * point.world_position
                mass += point.mass
    return weighted / mass


def test_rigid_and_soft_mass_live_surfaces_tessellation_inertia_and_controller_com_match():
    rigid = soft_foot.create_model(soft_foot.Feet.RIGID)
    soft = soft_foot.create_model(soft_foot.Feet.SOFT)
    assert (
        abs(
            sum(b.getMass() for b in rigid.atlas.getBodyNodes())
            - sum(b.getMass() for b in soft.atlas.getBodyNodes())
        )
        <= 1e-9
    )
    for model in (rigid, soft):
        assert (
            model.world.getConstraintSolver().getCollisionDetector().getType() == "fcl"
        )
        assert model.world.getNumSimulationThreads() == 1
        np.testing.assert_array_equal(model.world.getGravity(), [0, -9.81, 0])
        # Both variants retain the experiment's historical unbounded velocities.
        assert np.all(model.atlas.getVelocityLowerLimits() == -np.inf)
        assert np.all(model.atlas.getVelocityUpperLimits() == np.inf)
    for name in ("left_foot", "right_foot"):
        control, compliant = getattr(rigid, name), getattr(soft, name)
        assert abs(control.getMass() - compliant.getMass()) <= 1e-9
        rigid_nodes = _live_collision_surfaces(rigid, control)
        soft_nodes = _live_collision_surfaces(soft, compliant)
        assert len(rigid_nodes) == len(soft_nodes) == 1
        assert isinstance(rigid_nodes[0].getShape(), dart.dynamics.MeshShape)
        assert isinstance(soft_nodes[0].getShape(), dart.dynamics.SoftMeshShape)
        mesh = rigid_nodes[0].getShape().tri_mesh
        assert mesh.num_vertices == compliant.getNumPointMasses()
        assert mesh.num_triangles == compliant.num_faces
        matrix = rigid_nodes[0].getRelativeTransform().matrix()
        rest = np.array(
            [
                compliant.get_point_mass(i).rest_position
                for i in range(compliant.getNumPointMasses())
            ]
        )
        vertices = (matrix[:3, :3] @ mesh.vertices.T).T + matrix[:3, 3]
        assert np.max(np.abs(vertices - rest)) < 1e-9
        np.testing.assert_array_equal(
            mesh.triangles, [compliant.get_face(i) for i in range(compliant.num_faces)]
        )
        left, right = _combined_rest_inertia(control), _combined_rest_inertia(compliant)
        assert abs(left[0] - right[0]) <= 1e-9
        assert np.max(np.abs(left[1] - right[1])) < 1e-9
        assert np.max(np.abs(left[2] - right[2])) < 1e-9
    assert np.linalg.norm(_observed_com(soft) - soft.atlas.getCOM()) > 1e-3
    assert np.max(np.abs(_observed_com(soft) - _observed_com(rigid))) < 1e-9


def _ground_manifold(model):
    q = model.atlas.getPositions().copy()
    q[4] -= 0.002
    model.atlas.setPositions(q)
    option = dart.collision.CollisionOption()
    option.maxNumContacts = 10000
    result = dart.collision.CollisionResult()
    model.world.getConstraintSolver().getCollisionGroup().collide(option, result)
    points = []
    for contact in result.getContacts():
        bodies = [
            getattr(contact, f"collisionObject{i}")
            .getShapeFrame()
            .asShapeNode()
            .getBodyNodePtr()
            for i in (1, 2)
        ]
        if any(
            body is model.left_foot or body is model.right_foot for body in bodies
        ) and any(body.getSkeleton() is not model.atlas for body in bodies):
            points.append(contact.point.copy())
    return np.array(sorted(points, key=lambda row: tuple(row)))


def test_undeformed_ground_manifold_matches_one_to_one():
    rigid = _ground_manifold(soft_foot.create_model(soft_foot.Feet.RIGID))
    soft = _ground_manifold(soft_foot.create_model(soft_foot.Feet.SOFT))
    assert len(rigid) > 0 and rigid.shape == soft.shape
    used = np.zeros(len(soft), dtype=bool)
    for point in rigid:
        distance = np.linalg.norm(soft - point, axis=1)
        distance[used] = np.inf
        index = int(np.argmin(distance))
        assert distance[index] < 1e-6
        used[index] = True


def test_reset_clears_stale_soft_contacts_and_matches_fresh_for_300_steps():
    fresh = soft_foot.create_model(soft_foot.Feet.SOFT)
    model = soft_foot.create_model(soft_foot.Feet.SOFT)
    initial = soft_foot.state_vector(fresh)
    initial_gait = fresh.controller.current_state
    soft_foot.apply_push(model, [0, 0, 400], 100)
    for index in range(600):
        soft_foot.step(model)
        if index >= 200 and any(
            model.right_foot.get_point_mass(i).is_colliding
            for i in range(model.right_foot.getNumPointMasses())
        ):
            break
    else:
        pytest.fail("never reached a right-foot stance to reset from")
    assert np.max(np.abs(soft_foot.state_vector(model) - initial)) > 1e-6
    assert model.controller.current_state != initial_gait
    old_acceleration = model.atlas.getAccelerations().copy()
    soft_foot.reset_model(model)
    assert np.max(np.abs(soft_foot.state_vector(model) - initial)) < 1e-12
    assert model.controller.current_state == initial_gait == "1"
    assert model.force_duration == 0 and not model.external_force.any()
    assert model.world.getTime() == model.world.getSimFrames() == 0
    assert model.world.getLastCollisionResult().getNumContacts() == 0
    assert not model.left_foot.is_colliding and not model.right_foot.is_colliding
    # Native reset leaves generalized accelerations until the first new solve.
    np.testing.assert_array_equal(model.atlas.getAccelerations(), old_acceleration)
    for body in (model.left_foot, model.right_foot):
        for i in range(body.getNumPointMasses()):
            point = body.get_point_mass(i)
            assert not point.is_colliding
            for field in ("positions", "velocities", "accelerations", "forces"):
                np.testing.assert_array_equal(getattr(point, field), np.zeros(3))
    for _ in range(300):
        soft_foot.step(fresh)
        soft_foot.step(model)
        np.testing.assert_array_equal(_full_state(fresh), _full_state(model))
        assert model.controller.current_state == fresh.controller.current_state


def test_full_push_sweep_measures_every_nonmonotonic_trial(monkeypatch):
    create, advance, apply = (
        soft_foot.create_model,
        soft_foot.step,
        soft_foot.apply_push,
    )
    current = [None]
    outcomes = []
    magnitudes = []
    tail_pass = [True]

    def finish():
        if current[0] is not None:
            outcomes.append(
                current[0].world.getSimFrames() == 1300
                and tail_pass[0]
                and soft_foot.is_finite(current[0])
            )

    def record_create(feet):
        finish()
        current[0] = create(feet)
        tail_pass[0] = True
        return current[0]

    def record_apply(model, force, steps):
        np.testing.assert_array_equal(np.asarray(force)[:2], [0, 0])
        assert steps == 100
        magnitudes.append(force[2])
        apply(model, force, steps)

    def record_step(model):
        advance(model)
        if model.world.getSimFrames() >= 1051:
            tail_pass[0] &= soft_foot.is_upright(model)

    monkeypatch.setattr(soft_foot, "create_model", record_create)
    monkeypatch.setattr(soft_foot, "apply_push", record_apply)
    monkeypatch.setattr(soft_foot, "step", record_step)
    maxima = {}
    for feet in (soft_foot.Feet.RIGID, soft_foot.Feet.SOFT):
        current[0] = None
        outcomes.clear()
        magnitudes.clear()
        measured = soft_foot.max_recoverable_push(feet)
        finish()
        assert magnitudes == list(range(2000, 24001, 2000))
        assert len(outcomes) == len(magnitudes)
        maximum = max(
            (force for force, survived in zip(magnitudes, outcomes) if survived),
            default=0,
        )
        assert measured == maximum and 0 < measured < soft_foot.push_sweep_end
        maxima[feet] = measured
    assert maxima[soft_foot.Feet.SOFT] >= maxima[soft_foot.Feet.RIGID]


@pytest.mark.parametrize("feet", [2, -1, 1.0, "soft", None, np.bool_(True)])
def test_create_model_rejects_invalid_feet(feet):
    with pytest.raises(TypeError):
        soft_foot.create_model(feet)


@pytest.mark.parametrize("value", [np.int64(1), True])
def test_create_model_accepts_native_integral_feet(value):
    assert soft_foot.create_model(value).feet is soft_foot.Feet.SOFT


def test_push_returns_owned_force_copy_and_zero_duration_applies_no_force():
    model = soft_foot.create_model(soft_foot.Feet.RIGID)
    soft_foot.apply_push(model, [1, 2, 3], 0)
    model.external_force[:] = 99
    np.testing.assert_array_equal(model.external_force, [1, 2, 3])
    soft_foot.prepare_step(model)
    np.testing.assert_array_equal(model.pelvis.getExternalForceGlobal(), np.zeros(6))
    assert model.force_duration == 0


@pytest.mark.parametrize(
    "field",
    [
        "skeleton_positions",
        "skeleton_velocities",
        "point_positions",
        "point_velocities",
        "point_world_position",
    ],
)
def test_finite_guard_detects_independent_state_components(field):
    model = soft_foot.create_model(soft_foot.Feet.SOFT)
    if field.startswith("skeleton"):
        setter = (
            model.atlas.setPosition
            if field.endswith("positions")
            else model.atlas.setVelocity
        )
        setter(6, np.nan)
    elif field == "point_world_position":
        model.atlas.setPosition(3, 1e308)
        model.left_foot.get_point_mass(0).positions = [1e308, 0, 0]
    else:
        point = model.left_foot.get_point_mass(0)
        setattr(point, field.removeprefix("point_"), [np.nan, 0, 0])
    assert not soft_foot.is_finite(model)


@pytest.mark.parametrize("error", [ValueError, KeyboardInterrupt, SystemExit])
def test_integer_protocol_failures_match_native_type_errors(error):
    class Index:
        def __index__(self):
            raise error

    model = soft_foot.create_model(soft_foot.Feet.RIGID)
    with pytest.raises(TypeError):
        soft_foot.apply_push(model, [1, 0, 0], Index())
    with pytest.raises(TypeError):
        soft_foot.create_model(Index())


def test_model_argument_uses_actual_type_and_rejects_class_spoofing():
    class Object:
        @property
        def __class__(self):
            return soft_foot.Model

    for method in (
        soft_foot.prepare_step,
        soft_foot.state_vector,
        soft_foot.reset_model,
    ):
        with pytest.raises(TypeError):
            method(Object())


@pytest.mark.parametrize("feet", [soft_foot.Feet.RIGID, soft_foot.Feet.SOFT])
def test_cached_body_handles_survive_renames_and_drive_contact_diagnostics(feet):
    model = soft_foot.create_model(feet)
    expected = (model.pelvis, model.left_foot, model.right_foot)
    initial_heights = (
        soft_foot.pelvis_height_y(model),
        soft_foot.mean_foot_height_y(model),
    )
    for name, body in zip(("renamed_left", "renamed_right"), expected[1:]):
        body.setName(name)
    assert (model.pelvis, model.left_foot, model.right_foot) == expected
    assert (
        soft_foot.pelvis_height_y(model),
        soft_foot.mean_foot_height_y(model),
    ) == initial_heights
    for _ in range(500):
        soft_foot.step(model)
    assert soft_foot.is_finite(model) and soft_foot.is_upright(model)
    assert soft_foot.foot_contact_count(model) > 0
    mean_height = soft_foot.mean_foot_height_y(model)
    _, replacement = model.atlas.createWeldJointAndBodyNodePair(model.pelvis)
    replacement.setName("l_foot")
    assert model.atlas.getBodyNode("l_foot") is replacement
    assert model.left_foot is expected[1]
    assert soft_foot.mean_foot_height_y(model) == mean_height
    soft_foot.reset_model(model)
    assert (
        model.pelvis is expected[0]
        and model.left_foot is expected[1]
        and model.right_foot is expected[2]
    )
    height = soft_foot.pelvis_height_y(model)
    model.pelvis.setName("renamed_pelvis")
    assert model.pelvis is expected[0] and soft_foot.pelvis_height_y(model) == height
