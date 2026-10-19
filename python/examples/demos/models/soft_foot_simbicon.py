"""Comparable rigid and deformable Atlas feet driven by Python SIMBICON."""

import math
import operator
from dataclasses import dataclass, field
from enum import Enum

import dartpy as dart
import numpy as np

from .simbicon import Controller

time_step = 0.001
push_steps = 100
default_push_magnitude = 500.0
upright_gap = 0.40
push_sweep_start = 2000.0
push_sweep_end = 24000.0
push_sweep_step = 2000.0


class Feet(Enum):
    RIGID = 0
    SOFT = 1


@dataclass(frozen=True, eq=False)
class Model:
    world: dart.simulation.World
    atlas: dart.dynamics.Skeleton
    controller: Controller
    feet: Feet
    pelvis: dart.dynamics.BodyNode
    left_foot: dart.dynamics.BodyNode
    right_foot: dart.dynamics.BodyNode
    _external_force: np.ndarray = field(default_factory=lambda: np.zeros(3), repr=False)
    _force_duration: int = field(default=0, repr=False)

    @property
    def external_force(self):
        return self._external_force.copy()

    @property
    def force_duration(self):
        return self._force_duration


def _model(value):
    if not issubclass(type(value), Model):
        raise TypeError("model must be a Model")
    return value


def _int32(value):
    try:
        value = operator.index(value)
    except BaseException:
        raise TypeError("expected an integer") from None
    if not -(2**31) <= value < 2**31:
        raise TypeError("integer must fit a signed int")
    return value


def _feet(value):
    if type(value) is Feet:
        return value
    try:
        return Feet(_int32(value))
    except (TypeError, ValueError):
        raise TypeError("feet must be Feet.RIGID or Feet.SOFT") from None


def _copy_inertia(body):
    inertia = body.getInertia()
    # getInertia() borrows the live tensor; mutate a value copy before setInertia().
    return dart.dynamics.Inertia(
        inertia.getMass(), inertia.getLocalCOM(), inertia.getMoment()
    )


def _normalize_soft_foot(foot):
    if not issubclass(type(foot), dart.dynamics.SoftBodyNode):
        raise RuntimeError(f"{foot.getName()}: expected a SoftBodyNode")
    for node in foot.getShapeNodes():
        if node.hasCollisionAspect() and not isinstance(
            node.getShape(), dart.dynamics.SoftMeshShape
        ):
            node.removeCollisionAspect()
    link_mass = dart.dynamics.BodyNode.getMass(foot)
    point_mass = foot.getMass() - link_mass
    remaining = link_mass - point_mass
    if not remaining > 0:
        raise RuntimeError(f"{foot.getName()}: soft point masses exceed the link mass")
    inertia = _copy_inertia(foot)
    inertia.setMoment(inertia.getMoment() * (remaining / link_mass))
    inertia.setMass(remaining)
    foot.setInertia(inertia)


def _shift_to_origin(mass, com):
    squared = com[0] * com[0] + com[1] * com[1] + com[2] * com[2]
    return mass * (squared * np.eye(3) - np.outer(com, com))


def _normalize_rigid_foot(foot):
    friction = None
    for node in foot.getShapeNodes():
        if node.hasCollisionAspect():
            if friction is None and node.hasDynamicsAspect():
                friction = node.getDynamicsAspect().getFrictionCoeff()
            node.removeCollisionAspect()
    if friction is None:
        raise RuntimeError(f"{foot.getName()}: no collision surface to replace")
    transform = dart.math.Isometry3()
    transform.set_translation([0.05, 0, -0.06])
    properties = dart.dynamics.SoftBodyNodeHelper.makeBoxProperties(
        [0.275, 0.15, 0.075], transform, [3, 3, 3], 0.5
    )
    positions, masses = properties.point_rest_positions, properties.point_masses
    mesh = dart.math.TriMesh(positions, properties.faces)
    node = foot.createShapeNode(dart.dynamics.MeshShape([1, 1, 1], mesh))
    node.createCollisionAspect()
    node.getDynamicsAspect(True).setFrictionCoeff(friction)

    point_mass = 0.0
    for mass in masses:
        point_mass += float(mass)
    inertia = _copy_inertia(foot)
    sdf_mass = inertia.getMass()
    link_mass = sdf_mass - point_mass
    if not link_mass > 0:
        raise RuntimeError(f"{foot.getName()}: soft point masses exceed the link mass")
    link_com = inertia.getLocalCOM().copy()
    link_moment = inertia.getMoment() * (link_mass / sdf_mass)
    moment = link_moment + _shift_to_origin(link_mass, link_com)
    weighted_com = link_mass * link_com
    for mass, position in zip(masses, positions):
        moment += _shift_to_origin(float(mass), position)
        weighted_com += float(mass) * position
    combined_com = weighted_com / sdf_mass
    combined = dart.dynamics.Inertia()
    combined.setMass(sdf_mass)
    combined.setLocalCOM(combined_com)
    combined.setMoment(moment - _shift_to_origin(sdf_mass, combined_com))
    foot.setInertia(combined)


def create_model(feet):
    feet = _feet(feet)
    world = dart.simulation.World()
    world.setTimeStep(time_step)
    world.setNumSimulationThreads(1)
    world.getConstraintSolver().setCollisionDetector(
        dart.collision.FCLCollisionDetector()
    )
    ground_uri = "dart://sample/sdf/atlas/ground.urdf"
    atlas_uri = (
        "dart://sample/sdf/atlas/atlas_v3_no_head_soft_feet.sdf"
        if feet is Feet.SOFT
        else "dart://sample/sdf/atlas/atlas_v3_no_head.sdf"
    )
    ground = dart.utils.DartLoader().parseSkeleton(ground_uri)
    atlas = dart.utils.SdfParser.readSkeleton(atlas_uri)
    if ground is None or atlas is None:
        raise RuntimeError(
            f"failed to load {ground_uri if ground is None else atlas_uri}; "
            "these demo models load DART's sample data (dart://sample), "
            "which a DART source checkout provides"
        )
    # Preserve the experiment's historical unbounded velocities in both variants.
    dofs = atlas.getNumDofs()
    atlas.setVelocityLowerLimits(np.full(dofs, -math.inf))
    atlas.setVelocityUpperLimits(np.full(dofs, math.inf))
    if any(atlas.getBodyNode(name) is None for name in ("pelvis", "l_foot", "r_foot")):
        raise RuntimeError(f"{atlas_uri}: missing 'pelvis', 'l_foot', or 'r_foot'")
    normalize = _normalize_soft_foot if feet is Feet.SOFT else _normalize_rigid_foot
    # Normalize before subscribing collision objects to the world's solver.
    for name in ("l_foot", "r_foot"):
        normalize(atlas.getBodyNode(name))
    world.addSkeleton(ground)
    world.addSkeleton(atlas)
    atlas.setPosition(0, -0.5 * math.pi)
    world.setGravity([0, -9.81, 0])
    return Model(
        world,
        atlas,
        Controller(atlas, world),
        feet,
        atlas.getBodyNode("pelvis"),
        atlas.getBodyNode("l_foot"),
        atlas.getBodyNode("r_foot"),
    )


def apply_push(model, force, steps):
    model = _model(model)
    try:
        force = np.asarray(force, dtype=np.float64)
    except BaseException:
        raise TypeError("force must be a three-dimensional vector") from None
    if force.shape not in ((3,), (3, 1)):
        raise TypeError("force must be a three-dimensional vector")
    steps = _int32(steps)
    if not np.isfinite(force).all():
        raise ValueError("force must be finite")
    if steps < 0:
        raise ValueError("steps must not be negative")
    model._external_force[:] = force.reshape(3)
    object.__setattr__(model, "_force_duration", steps)


def prepare_step(model):
    model = _model(model)
    if model._force_duration > 0:
        model.pelvis.addExtForce(model._external_force)
        object.__setattr__(model, "_force_duration", model._force_duration - 1)
        if model._force_duration == 0:
            model._external_force[:] = 0
    model.controller.update()


def step(model):
    prepare_step(model)
    model.world.step()


def pelvis_height_y(model):
    return float(_model(model).pelvis.getTransform().translation()[1])


def mean_foot_height_y(model):
    model = _model(model)
    return float(
        0.5
        * (
            model.left_foot.getTransform().translation()[1]
            + model.right_foot.getTransform().translation()[1]
        )
    )


def is_upright(model):
    return pelvis_height_y(model) - mean_foot_height_y(model) > upright_gap


def _contact_bodies(contact):
    bodies = []
    for collision_object in (contact.collisionObject1, contact.collisionObject2):
        node = (
            None
            if collision_object is None
            else collision_object.getShapeFrame().asShapeNode()
        )
        bodies.append(None if node is None else node.getBodyNodePtr())
    return bodies


def _soft_bodies(atlas):
    return [
        body
        for body in atlas.getBodyNodes()
        if isinstance(body, dart.dynamics.SoftBodyNode)
    ]


def foot_contact_count(model):
    model = _model(model)
    feet = (model.left_foot, model.right_foot)
    return sum(
        any(body is feet[0] or body is feet[1] for body in _contact_bodies(contact))
        for contact in model.world.getLastCollisionResult().getContacts()
    )


def is_finite(model):
    atlas = _model(model).atlas
    if (
        not np.isfinite(atlas.getPositions()).all()
        or not np.isfinite(atlas.getVelocities()).all()
    ):
        return False
    for body in _soft_bodies(atlas):
        if not np.isfinite(body.getTransform().matrix()).all():
            return False
        for i in range(body.getNumPointMasses()):
            point = body.get_point_mass(i)
            if not all(
                np.isfinite(getattr(point, name)).all()
                for name in ("positions", "velocities", "world_position")
            ):
                return False
    return True


def state_vector(model):
    atlas = _model(model).atlas
    values = [atlas.getPositions(), atlas.getVelocities()]
    for body in _soft_bodies(atlas):
        for i in range(body.getNumPointMasses()):
            point = body.get_point_mass(i)
            values.extend((point.positions, point.velocities))
    return np.concatenate(values).copy()


def reset_model(model):
    model = _model(model)
    model.controller.reset_robot()
    for body in _soft_bodies(model.atlas):
        for i in range(body.getNumPointMasses()):
            point = body.get_point_mass(i)
            point.reset_positions()
            point.reset_velocities()
            point.reset_accelerations()
            point.reset_forces()
    model.atlas.clear_colliding_bodies()
    model.atlas.clearExternalForces()
    model.atlas.clearInternalForces()
    model.world.reset()
    solver = model.world.getConstraintSolver()
    solver.setCollisionDetector(
        solver.getCollisionDetector().cloneWithoutCollisionObjects()
    )
    model.controller._restart_gait(model.world.getTime())
    model._external_force[:] = 0
    object.__setattr__(model, "_force_duration", 0)


def survives_push(feet, magnitude):
    """Measure one fresh trial; recovery is not monotone in push magnitude."""
    model = create_model(feet)
    for _ in range(500):
        step(model)
        if not is_finite(model):
            return False
    apply_push(model, [0, 0, magnitude], push_steps)
    for index in range(800):
        step(model)
        if not is_finite(model) or (index >= 550 and not is_upright(model)):
            return False
    return True
