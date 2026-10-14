"""Five soft links driven by the catalog's traveling-wave servo gait."""

import math

import dartpy as dart
import numpy as np

num_links = 5
gait_amplitude = 0.56
gait_frequency_hz = 1.35

_LINK_SIZE = (0.52, 0.34, 0.28)
_COLORS = (
    (0.18, 0.66, 0.83, 0.88),
    (0.24, 0.77, 0.63, 0.88),
    (0.78, 0.82, 0.31, 0.88),
    (0.95, 0.62, 0.25, 0.88),
    (0.88, 0.35, 0.42, 0.88),
)


def _inertia():
    inertia = dart.dynamics.Inertia()
    inertia.setMass(0.16)
    inertia.setMoment(dart.dynamics.BoxShape(_LINK_SIZE).computeInertia(0.16))
    return inertia


def _link_properties(index):
    body = dart.dynamics.BodyNodeProperties(
        dart.dynamics.BodyNodeAspectProperties(f"soft_link_{index}", _inertia())
    )
    flesh = dart.dynamics.SoftBodyNodeHelper.makeBoxProperties(
        _LINK_SIZE, dart.math.Isometry3(), [3, 3, 3], 0.54
    )
    flesh.mKv = 850.0
    flesh.mKe = 120.0
    flesh.mDampCoeff = 8.0
    return dart.dynamics.SoftBodyNodeProperties(body, flesh)


def _style_link(body, index):
    body.setInertia(_inertia())
    for node in body.getShapeNodes():
        if node.hasDynamicsAspect():
            node.getDynamicsAspect().setFrictionCoeff(1.25)
            node.getDynamicsAspect().setRestitutionCoeff(0.0)
        if node.hasVisualAspect() and node.getShape().getType() == "SoftMeshShape":
            node.getVisualAspect().setRGBA(_COLORS[index])


def create_world():
    world = dart.simulation.World("soft_worm_world")
    world.setTimeStep(0.001)
    world.setGravity([0, 0, -9.81])
    ground = dart.dynamics.Skeleton("ground")
    joint, body = ground.createWeldJointAndBodyNodePair()
    node = body.createShapeNode(dart.dynamics.BoxShape([14, 5, 0.08]))
    node.createVisualAspect()
    node.createCollisionAspect()
    node.createDynamicsAspect()
    node.getVisualAspect().setColor([0.33, 0.36, 0.40])
    node.getDynamicsAspect().setFrictionCoeff(1.25)
    node.getDynamicsAspect().setRestitutionCoeff(0.0)
    transform = dart.math.Isometry3()
    transform.set_translation([0, 0, -0.04])
    joint.setTransformFromParentBodyNode(transform)
    ground.setMobile(False)
    world.addSkeleton(ground)

    worm = dart.dynamics.Skeleton("soft_worm")
    properties = dart.dynamics.FreeJointProperties()
    properties.mName = "root_free_joint"
    joint, parent = worm.createFreeJointAndSoftBodyNodePair(
        None, properties, _link_properties(0)
    )
    joint.setActuatorType(dart.dynamics.ActuatorType.PASSIVE)
    _style_link(parent, 0)
    transform = dart.math.Isometry3()
    transform.set_translation([0, 0, 0.19])
    joint.setPositions(dart.dynamics.FreeJoint.convertToPositions(transform))
    for index in range(1, num_links):
        properties = dart.dynamics.RevoluteJointProperties()
        properties.mName = f"spine_joint_{index}"
        properties.mAxis = [0, 1, 0]
        transform = dart.math.Isometry3()
        transform.set_translation([0.26, 0, 0])
        properties.mT_ParentBodyToJoint = transform
        transform = dart.math.Isometry3()
        transform.set_translation([-0.26, 0, 0])
        properties.mT_ChildBodyToJoint = transform
        properties.mPositionLowerLimits = [-0.92]
        properties.mPositionUpperLimits = [0.92]
        joint, parent = worm.create_revolute_joint_and_soft_body_node_pair(
            parent, properties, _link_properties(index)
        )
        joint.setActuatorType(dart.dynamics.ActuatorType.SERVO)
        joint.setLimitEnforcement(True)
        joint.setVelocityLowerLimit(0, -4.0)
        joint.setVelocityUpperLimit(0, 4.0)
        _style_link(parent, index)
    world.addSkeleton(worm)
    return world, worm


def _require_worm(worm):
    if not isinstance(worm, dart.dynamics.Skeleton):
        raise TypeError("worm must be a Skeleton")


def _soft_bodies(worm):
    return (
        body
        for body in worm.getBodyNodes()
        if isinstance(body, dart.dynamics.SoftBodyNode)
    )


def apply_gait(worm, time, enabled):
    _require_worm(worm)
    try:
        if isinstance(time, float):
            time = float.__float__(time)
        elif hasattr(type(time), "__float__") or hasattr(type(time), "__index__"):
            time = float(time)
        else:
            raise TypeError
    except Exception:
        raise TypeError("time must be a real number") from None
    if not isinstance(enabled, bool):
        raise TypeError("enabled must be a bool")
    omega = 2.0 * math.pi * gait_frequency_hz
    for index in range(1, num_links):
        joint = worm.getJoint(f"spine_joint_{index}")
        if joint is None:
            continue
        command = 0.0
        if enabled:
            phase = omega * time - 1.05 * (index - 1)
            if math.isfinite(phase):
                target = gait_amplitude * math.sin(phase)
                target_velocity = gait_amplitude * omega * math.cos(phase)
                command = target_velocity + 7.0 * (target - joint.getPosition(0))
                if command < -4.0:
                    command = -4.0
                elif command > 4.0:
                    command = 4.0
            else:
                # Native trig produces NaN; the joint rejects that command.
                command = math.nan
        joint.setCommand(0, command)


def root_x(worm):
    _require_worm(worm)
    if worm.getNumBodyNodes() == 0:
        raise ValueError("worm must have a root body")
    return float(worm.getRootBodyNode().getTransform().translation()[0])


def position_checksum(worm):
    _require_worm(worm)
    # Match the native checksum's ordered long-double accumulation.
    checksum = np.longdouble(0)
    component = 1
    for value in worm.getPositions():
        checksum += np.longdouble(component) * value
        component += 1
    for body in _soft_bodies(worm):
        for index in range(body.getNumPointMasses()):
            for value in body.get_point_mass(index).world_position:
                checksum += np.longdouble(component) * value
                component += 1
    return float(checksum)


def is_finite(worm):
    _require_worm(worm)
    if (
        not np.isfinite(worm.getPositions()).all()
        or not np.isfinite(worm.getVelocities()).all()
    ):
        return False
    for body in _soft_bodies(worm):
        if not np.isfinite(body.getTransform().matrix()).all():
            return False
        for index in range(body.getNumPointMasses()):
            point = body.get_point_mass(index)
            if (
                not np.isfinite(point.positions).all()
                or not np.isfinite(point.velocities).all()
                or not np.isfinite(point.world_position).all()
            ):
                return False
    return True
