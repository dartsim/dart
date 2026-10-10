"""Headless checks of the collisions tutorial's actual scene and event handler."""

import gc
import importlib.util
import math
from pathlib import Path

import dartpy as dart
import numpy as np
import pytest

pytestmark = pytest.mark.skipif(
    not hasattr(dart.gui, "osg"), reason="DART_BUILD_GUI_OSG is disabled"
)



def load_tutorial(filename="main_finished.py"):
    path = Path(__file__).parents[2] / "tutorials" / "collisions" / filename
    spec = importlib.util.spec_from_file_location("tutorial_collisions", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def tutorial():
    return load_tutorial()


@pytest.mark.parametrize(
    ("factory", "bodies", "dofs", "soft_bodies"),
    [
        ("create_ball", 1, 6, 0),
        ("create_soft_body", 1, 6, 1),
        ("create_hybrid_body", 2, 6, 1),
        ("create_rigid_chain", 3, 12, 0),
        ("create_rigid_ring", 6, 21, 0),
    ],
)
def test_prototypes_clone_with_geometry_and_inertia(
    tutorial, factory, bodies, dofs, soft_bodies
):
    original = getattr(tutorial, factory)()
    original_shape_count = original.getNumShapeNodes()
    clone = original.clone()
    del original
    gc.collect()
    assert clone.getNumBodyNodes() == bodies
    assert clone.getNumDofs() == dofs
    assert clone.getNumShapeNodes() == original_shape_count
    count = 0
    for i in range(bodies):
        body = clone.getBodyNode(i)
        assert body.getNumShapeNodes() > 0
        for j in range(body.getNumShapeNodes()):
            assert body.getShapeNode(j).getShape() is not None
        assert np.isfinite(body.getInertia().getMoment()).all()
        assert body.getMass() > 0
        if isinstance(body, dart.dynamics.SoftBodyNode):
            count += 1
            assert body.getNumPointMasses() > 0
        if factory in ("create_ball", "create_rigid_chain", "create_rigid_ring"):
            shape_node = body.getShapeNode(0)
            shape = shape_node.getShape()
            mass = tutorial.default_shape_density * shape.getVolume()
            assert body.getMass() == pytest.approx(mass)
            np.testing.assert_allclose(
                body.getInertia().getMoment(), shape.computeInertia(mass)
            )
            assert (
                shape_node.getDynamicsAspect().getRestitutionCoeff()
                == pytest.approx(tutorial.default_restitution)
            )
            np.testing.assert_allclose(
                body.getWorldTransform().translation(),
                [0, 0, i * tutorial.default_shape_height],
                atol=1e-12,
            )
            if i > 0:
                joint = body.getParentJoint()
                for j in range(joint.getNumDofs()):
                    assert joint.getDampingCoefficient(j) == pytest.approx(
                        tutorial.default_damping_coefficient
                    )
    assert count == soft_bodies


@pytest.mark.parametrize("shape_type", [0, 1, 2])
def test_soft_surface_factories_clone_and_step(tutorial, shape_type):
    skeleton = dart.dynamics.Skeleton("soft_surface")
    tutorial.add_soft_body(skeleton, "skin", shape_type)
    clone = skeleton.clone()
    body = clone.getBodyNode(0)
    assert isinstance(body, dart.dynamics.SoftBodyNode)
    assert body.getNumPointMasses() > 0
    assert body.getVertexSpringStiffness() == tutorial.default_vertex_stiffness
    assert body.getEdgeSpringStiffness() == tutorial.default_edge_stiffness
    assert body.getDampingCoefficient() == tutorial.default_soft_damping
    assert body.getInertia().getMass() == pytest.approx(1e-8)
    np.testing.assert_allclose(body.getInertia().getMoment(), 1e-8 * np.eye(3))
    radius = tutorial.default_shape_height / 2
    height = 2 * tutorial.default_shape_width
    width = tutorial.default_shape_height
    areas = [
        2 * width**2 + 4 * width * height,
        2 * math.pi * radius * height + 2 * math.pi * radius**2,
        4 * math.pi * radius**2,
    ]
    expected_mass = (
        tutorial.default_shape_density
        * tutorial.default_skin_thickness
        * areas[shape_type]
        + 1e-8
    )
    assert body.getMass() == pytest.approx(expected_mass)
    world = dart.simulation.World()
    positions = np.zeros(6)
    positions[5] = tutorial.default_start_height
    clone.setPositions(positions)
    world.addSkeleton(clone)
    for _ in range(5):
        world.step()
    assert np.isfinite(clone.getPositions()).all()
    assert np.isfinite(clone.getVelocities()).all()
    assert np.isfinite(clone.getCOM()).all()


@pytest.mark.parametrize("key", "1234")
def test_spawn_controls_and_finite_simulation(tutorial, key):
    node = tutorial.build_scene()
    handler = node.handler
    handler.randomize = False

    class KeyEvent:
        def getEventType(self):
            return dart.gui.osg.GUIEventAdapter.KEYDOWN

        def getKey(self):
            return ord(key)

    assert handler.handle(KeyEvent(), None)
    assert node.world.getNumSkeletons() == 3
    spawned = node.world.getSkeleton(2)
    assert spawned.getPositions()[5] == pytest.approx(tutorial.default_start_height)
    for _ in range(30):
        node.customPreStep()
        node.world.step()
        node.customPostStep()
    assert np.isfinite(spawned.getPositions()).all()
    assert np.isfinite(spawned.getVelocities()).all()
    assert np.isfinite(spawned.getCOM()).all()


def test_launch_velocity_is_specified_at_com_and_overlap_is_rejected(tutorial):
    node = tutorial.build_scene()
    handler = node.handler
    handler.randomize = False
    chain = handler.originals[3].clone()
    assert handler.add_object(chain)
    linear = tutorial.default_start_v * np.array(
        [
            math.cos(tutorial.default_launch_angle),
            0,
            math.sin(tutorial.default_launch_angle),
        ]
    )
    angular = np.array([0, tutorial.default_start_w, 0])
    np.testing.assert_allclose(chain.getCOMLinearVelocity(), linear, atol=1e-10)
    root = chain.getBodyNode(0)
    np.testing.assert_allclose(root.getAngularVelocity(), angular, atol=1e-10)
    root_position = root.getWorldTransform().translation()
    expected_root_linear = linear + np.cross(angular, root_position - chain.getCOM())
    np.testing.assert_allclose(
        root.getLinearVelocity(), expected_root_linear, atol=1e-10
    )
    before = node.world.getNumSkeletons()
    assert not handler.add_object(handler.originals[0].clone())
    assert node.world.getNumSkeletons() == before


def test_ring_closure_springs_and_constraint_cleanup(tutorial):
    node = tutorial.build_scene()
    handler = node.handler
    handler.randomize = False
    ring = handler.originals[4].clone()
    assert handler.add_ring(ring)
    solver = node.world.getConstraintSolver()
    assert solver.getNumConstraints() == 1
    assert len(handler.joint_constraints) == 1
    for i in range(6, ring.getNumDofs()):
        dof = ring.getDof(i)
        assert dof.getSpringStiffness() == pytest.approx(tutorial.ring_spring_stiffness)
        assert dof.getDampingCoefficient() == pytest.approx(
            tutorial.ring_damping_coefficient
        )
        assert dof.getPosition() == pytest.approx(dof.getRestPosition())
    head = ring.getBodyNode(0)
    tail = ring.getBodyNode(ring.getNumBodyNodes() - 1)
    head_endpoint = head.getWorldTransform().multiply(
        [0, 0, -tutorial.default_shape_height / 2]
    )
    tail_endpoint = tail.getWorldTransform().multiply(
        [0, 0, tutorial.default_shape_height / 2]
    )
    np.testing.assert_allclose(head_endpoint, tail_endpoint, atol=1e-10)
    assert not handler.add_ring(handler.originals[4].clone())
    assert solver.getNumConstraints() == 1
    for _ in range(20):
        node.world.step()
    assert np.isfinite(ring.getPositions()).all()
    assert np.isfinite(ring.getVelocities()).all()
    head_endpoint = head.getWorldTransform().multiply(
        [0, 0, -tutorial.default_shape_height / 2]
    )
    tail_endpoint = tail.getWorldTransform().multiply(
        [0, 0, tutorial.default_shape_height / 2]
    )
    np.testing.assert_allclose(head_endpoint, tail_endpoint, atol=1e-3)
    handler.remove_skeleton(ring)
    del ring
    gc.collect()
    assert solver.getNumConstraints() == 0
    assert not handler.joint_constraints
    assert node.world.getNumSkeletons() == 2
    node.world.step()


def test_randomization_and_oldest_object_deletion(tutorial):
    node = tutorial.build_scene()
    handler = node.handler

    class KeyEvent:
        def __init__(self, key):
            self.key = key

        def getEventType(self):
            return dart.gui.osg.GUIEventAdapter.KEYDOWN

        def getKey(self):
            return ord(self.key)

    assert handler.randomize
    assert handler.handle(KeyEvent("r"), None)
    assert not handler.randomize
    handler.handle(KeyEvent("1"), None)
    oldest = node.world.getSkeleton(2)
    oldest.setPosition(3, -0.5)
    handler.handle(KeyEvent("1"), None)
    newest = node.world.getSkeleton(3)
    handler.handle(KeyEvent("d"), None)
    assert node.world.getNumSkeletons() == 3
    assert node.world.getSkeleton(2) == newest
    handler.handle(KeyEvent("d"), None)
    handler.handle(KeyEvent("d"), None)
    assert node.world.getNumSkeletons() == 2
    assert not handler.handle(KeyEvent("?"), None)


def test_randomized_launch_stays_in_original_ranges(tutorial):
    node = tutorial.build_scene()
    handler = node.handler
    handler.rng.seed(17)
    ball = handler.originals[0].clone()
    assert handler.add_object(ball)
    assert abs(ball.getPosition(4)) <= tutorial.default_spawn_range
    velocity = ball.getCOMLinearVelocity()
    speed = np.linalg.norm(velocity)
    angle = math.atan2(velocity[2], velocity[0])
    assert tutorial.minimum_start_v <= speed <= tutorial.maximum_start_v
    assert tutorial.minimum_launch_angle <= angle <= tutorial.maximum_launch_angle
    angular_velocity = ball.getBodyNode(0).getAngularVelocity()
    assert abs(angular_velocity[1]) <= tutorial.maximum_start_w
    np.testing.assert_allclose(angular_velocity[[0, 2]], [0, 0], atol=1e-10)


def test_exercise_reports_the_first_incomplete_lesson():
    tutorial = load_tutorial("main.py")
    with pytest.raises(NotImplementedError, match="Lesson 1a-f"):
        tutorial.build_scene()
    with pytest.raises(NotImplementedError, match="Lesson 2a-e"):
        tutorial.create_soft_body()
    with pytest.raises(NotImplementedError, match="Lesson 4a-c"):
        tutorial.setup_ring(None)
