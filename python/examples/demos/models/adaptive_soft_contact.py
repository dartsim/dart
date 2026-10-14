"""Programmatic soft contact with periodic pushing and activation estimates."""

import math
import operator
from dataclasses import dataclass

import dartpy as dart
import numpy as np

time_step = 0.001
soft_body_radius = 0.36
compare_tolerance = 0.25
default_ring_count = 1
default_linger_steps = 12


@dataclass(frozen=True)
class Model:
    world: dart.simulation.World
    soft_skeleton: dart.dynamics.Skeleton
    pusher_skeleton: dart.dynamics.Skeleton

    @property
    def soft_body(self):
        return self.soft_skeleton.getBodyNode(0)

    @property
    def pusher_joint(self):
        return self.pusher_skeleton.getJoint(0)


@dataclass(frozen=True)
class Checksum:
    skeleton_position_l1: float
    skeleton_position_squared: float
    point_world_position_l1: float
    point_world_position_squared: float
    active: int
    total: int
    contacts: int
    finite: bool


def _model(value):
    if not isinstance(value, Model):
        raise TypeError("model must be a Model")
    return value


def _settings(adaptive_enabled, ring_count, linger_steps):
    if not isinstance(adaptive_enabled, bool):
        raise TypeError("adaptive_enabled must be a bool")
    counts = [operator.index(value) for value in (ring_count, linger_steps)]
    if any(value < 0 or value > int(np.iinfo(np.uintp).max) for value in counts):
        raise TypeError("ring_count and linger_steps must be unsigned integers")
    return adaptive_enabled, *counts


def create_model(
    *,
    adaptive_enabled=True,
    ring_count=default_ring_count,
    linger_steps=default_linger_steps,
):
    adaptive_enabled, ring_count, linger_steps = _settings(
        adaptive_enabled, ring_count, linger_steps
    )
    world = dart.simulation.World("adaptive_soft_contact")
    world.setTimeStep(time_step)
    world.setGravity([0, 0, -9.81])

    soft_skeleton = dart.dynamics.Skeleton("adaptive_ellipsoid")
    joint_properties = dart.dynamics.FreeJointProperties()
    joint_properties.mName = "adaptive_ellipsoid_joint"
    shape = dart.dynamics.EllipsoidShape([0.72, 0.54, 0.60])
    body_properties = dart.dynamics.BodyNodeProperties(
        dart.dynamics.BodyNodeAspectProperties(
            "adaptive_ellipsoid_body",
            dart.dynamics.Inertia(0.8, [0, 0, 0], shape.computeInertia(0.8)),
        )
    )
    soft_properties = dart.dynamics.SoftBodyNodeHelper.makeEllipsoidProperties(
        [0.72, 0.54, 0.60], 12, 8, 1.6
    )
    soft_properties.mKv = 4000.0
    soft_properties.mKe = 400.0
    soft_properties.mDampCoeff = 500.0
    joint, soft_body = soft_skeleton.createFreeJointAndSoftBodyNodePair(
        None,
        joint_properties,
        dart.dynamics.SoftBodyNodeProperties(body_properties, soft_properties),
    )
    soft_body.getShapeNode(0).getDynamicsAspect().setFrictionCoeff(1.1)
    soft_body.adaptive_contact_activation_ring_count = ring_count
    soft_body.adaptive_contact_activation_linger_steps = linger_steps
    soft_body.adaptive_contact_activation_velocity_tolerance = 0.035
    soft_body.adaptive_contact_activation_position_tolerance = 0.018
    soft_body.adaptive_contact_activation_enabled = adaptive_enabled
    transform = dart.math.Isometry3()
    transform.set_translation([0, 0, 1.10])
    joint.setPositions(dart.dynamics.FreeJoint.convertToPositions(transform))
    soft_body.getShapeNode(0).getVisualAspect().setColor([0.20, 0.48, 0.95, 0.58])

    pusher = dart.dynamics.Skeleton("periodic_pusher")
    _, body = pusher.createFreeJointAndBodyNodePair()
    shape = dart.dynamics.BoxShape([0.20, 0.70, 0.34])
    node = body.createShapeNode(shape)
    node.createCollisionAspect()
    node.getVisualAspect(True).setColor([0.96, 0.48, 0.16])
    node.getDynamicsAspect(True).setFrictionCoeff(0.8)
    body.setInertia(dart.dynamics.Inertia(20.0, [0, 0, 0], shape.computeInertia(20.0)))
    pusher.setMobile(False)

    ground = dart.dynamics.Skeleton("ground")
    _, body = ground.createWeldJointAndBodyNodePair()
    node = body.createShapeNode(dart.dynamics.BoxShape([6.0, 4.0, 0.1]))
    node.createCollisionAspect()
    node.setRelativeTranslation([0, 0, -0.05])
    node.getVisualAspect(True).setColor([0.62, 0.65, 0.69])
    node.getDynamicsAspect(True).setFrictionCoeff(1.0)
    for skeleton in (ground, soft_skeleton, pusher):
        world.addSkeleton(skeleton)
    model = Model(world, soft_skeleton, pusher)
    set_pusher_pose(model, 0.0)
    return model


def configure(model, *, adaptive_enabled, ring_count, linger_steps):
    body = _model(model).soft_body
    enabled, rings, linger = _settings(adaptive_enabled, ring_count, linger_steps)
    if body.adaptive_contact_activation_ring_count != rings:
        body.adaptive_contact_activation_ring_count = rings
    if body.adaptive_contact_activation_linger_steps != linger:
        body.adaptive_contact_activation_linger_steps = linger
    if body.adaptive_contact_activation_enabled != enabled:
        body.adaptive_contact_activation_enabled = enabled


def set_pusher_pose(model, time):
    model = _model(model)
    try:
        if isinstance(time, float):
            time = float.__float__(time)
        elif hasattr(type(time), "__float__") or hasattr(type(time), "__index__"):
            time = float(time)
        else:
            raise TypeError
    except Exception:
        raise TypeError("time must be a real number") from None
    phase = 2.0 * math.pi * time / 1.4
    # Preserve the native nonfinite pusher pose for invalid supplied times.
    cosine = math.cos(phase) if math.isfinite(phase) else math.nan
    transform = dart.math.Isometry3()
    transform.set_translation([-1.10 + 0.325 * (1.0 - cosine), 0, 0.37])
    model.pusher_joint.setPositions(
        dart.dynamics.FreeJoint.convertToPositions(transform)
    )


def prepare_step(model):
    set_pusher_pose(model, _model(model).world.getTime())


def step(model):
    prepare_step(model)
    model.world.step()
    return is_finite(model)


def _points(model):
    body = _model(model).soft_body
    return [body.get_point_mass(index) for index in range(body.getNumPointMasses())]


def point_positions(model):
    return np.array(
        [point.world_position for point in _points(model)], dtype=float
    ).reshape(-1, 3)


def compute_checksum(model):
    model = _model(model)
    positions = model.soft_skeleton.getPositions()
    finite = (
        np.isfinite(positions).all()
        and np.isfinite(model.soft_skeleton.getVelocities()).all()
    )
    point_l1 = point_squared = 0.0
    for point in _points(model):
        world_position = point.world_position
        point_l1 += float(np.abs(world_position).sum())
        point_squared += float(world_position.dot(world_position))
        finite = (
            finite
            and np.isfinite(world_position).all()
            and np.isfinite(point.positions).all()
            and np.isfinite(point.velocities).all()
        )
    body = model.soft_body
    return Checksum(
        float(np.abs(positions).sum()),
        float(positions.dot(positions)),
        point_l1,
        point_squared,
        body.num_active_point_masses,
        body.getNumPointMasses(),
        model.world.getLastCollisionResult().getNumContacts(),
        bool(finite),
    )


def is_finite(model):
    return compute_checksum(model).finite


def position_checksum(model):
    model = _model(model)
    checksum = np.longdouble(0)
    component = 1
    # Keep the original long-double accumulation order, including each axis.
    for values in (model.soft_skeleton.getPositions(), *point_positions(model)):
        for value in values:
            checksum += np.longdouble(component) * np.longdouble(value)
            component += 1
    return float(checksum)


def surface_pose_delta(lhs, rhs):
    left = _model(lhs).soft_body.getWorldTransform()
    right = _model(rhs).soft_body.getWorldTransform()
    translation_delta = np.linalg.norm(left.translation() - right.translation())
    rotation_delta = dart.math.AngleAxis(left.rotation().T @ right.rotation()).angle()
    return float(translation_delta + soft_body_radius * rotation_delta)


def active_flags(model):
    positions = point_positions(model)
    finite = np.isfinite(positions).all(axis=1)
    contacts = [
        contact.point
        for contact in model.world.getLastCollisionResult().getContacts()
        if np.isfinite(contact.point).all()
    ]
    distances = np.full(len(positions), np.inf)
    if contacts:
        with np.errstate(over="ignore"):
            for point in contacts:
                distances[finite] = np.minimum(
                    distances[finite], np.square(positions[finite] - point).sum(axis=1)
                )
    else:
        distances[finite] = positions[finite, 2]
    flags = np.zeros(len(positions), dtype=bool)
    count = min(model.soft_body.num_active_point_masses, len(positions))
    flags[np.argsort(distances, kind="stable")[:count]] = True
    return flags
