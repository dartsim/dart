"""Build ssik arm chains and ghost solutions using DART's dynamics API."""

import math
import operator
from dataclasses import dataclass

import dartpy as dart
import numpy as np


@dataclass(frozen=True)
class ArmSpec:
    label: str
    module: str


def get_ssik_arm_specs():
    return [
        ArmSpec("Universal Robots UR5", "ur5_ik"),
        ArmSpec("KUKA Puma 560", "puma560_ik"),
        ArmSpec("Kinova JACO 2", "jaco2_ik"),
        ArmSpec("KUKA iiwa14", "iiwa14_ik"),
        ArmSpec("Kinova Gen3", "gen3_ik"),
        ArmSpec("Franka Panda", "franka_panda_ik"),
        ArmSpec("UFactory xArm7", "xarm7_ik"),
        ArmSpec("UFactory xArm6", "xarm6_ik"),
        ArmSpec("Unitree Z1", "z1_ik"),
        ArmSpec("AgileX PiPER", "piper_ik"),
        ArmSpec("Flexiv Rizon 4", "rizon4_ik"),
        ArmSpec("Kassow KR810", "kassow_kr810_ik"),
        ArmSpec("Flexiv Rizon 10", "rizon10_ik"),
        ArmSpec("FANUC CRX-10iA/L", "fanuc_crx10ial_ik"),
        ArmSpec("I2RT YAM", "yam_ik"),
        ArmSpec("I2RT big_yam", "big_yam_ik"),
        ArmSpec("Franka Research 3", "fr3_ik"),
        ArmSpec("OpenArm left", "openarm_left_ik"),
        ArmSpec("OpenArm right", "openarm_right_ik"),
    ]


def _rigid_matrix(value):
    matrix = np.asarray(value, dtype=float)
    if matrix.shape != (4, 4):
        raise TypeError("transform must be a 4x4 matrix")
    rotation = matrix[:3, :3]
    orthogonal = rotation.T @ rotation
    if (
        not np.isfinite(matrix).all()
        or np.linalg.norm(matrix[3] - [0, 0, 0, 1]) > 1e-8
        or np.linalg.norm(orthogonal - np.eye(3))
        > 1e-8 * min(np.linalg.norm(orthogonal), np.sqrt(3))
        or abs(np.linalg.det(rotation) - 1) > 1e-8
    ):
        raise ValueError("transform must be a finite rigid 4x4 matrix")
    return matrix.copy()


def _vector(value, size):
    vector = np.asarray(value, dtype=float)
    if vector.shape not in ((size,), (size, 1)):
        raise TypeError(f"expected a vector with {size} elements")
    return vector.reshape(size).copy()


class JointSpec:
    def __init__(self, t_left, t_right, axis, *, prismatic=False):
        self._t_left = _rigid_matrix(t_left)
        self._t_right = _rigid_matrix(t_right)
        self._axis = _vector(axis, 3)
        with np.errstate(over="ignore"):
            length = np.linalg.norm(self._axis)
        if not np.isfinite(self._axis).all() or not np.isfinite(length) or length <= 1e-9:
            raise ValueError("axis must be finite and nonzero")
        if not isinstance(prismatic, bool):
            raise TypeError("prismatic must be a bool")
        self._prismatic = prismatic

    @property
    def t_left(self):
        return self._t_left.copy()

    @property
    def t_right(self):
        return self._t_right.copy()

    @property
    def axis(self):
        return self._axis.copy()

    @property
    def prismatic(self):
        return self._prismatic


def _float_controls(value):
    vector = _vector(value, 3)
    with np.errstate(over="ignore"):
        controls = vector.astype(np.float32).astype(float)
    if not np.isfinite(controls).all():
        raise ValueError("target controls must be finite floats")
    return controls


def make_ssik_target_transform(xyz, rpy_degrees):
    xyz = _float_controls(xyz)
    roll, pitch, yaw = _float_controls(rpy_degrees) * math.pi / 180
    rotation = (
        dart.math.AngleAxis(yaw, [0, 0, 1])
        .quaternion()
        .multiply(dart.math.AngleAxis(pitch, [0, 1, 0]).quaternion())
        .multiply(dart.math.AngleAxis(roll, [1, 0, 0]).quaternion())
    )
    return dart.math.Isometry3(rotation, xyz)


def ssik_target_controls(transform):
    matrix = _rigid_matrix(transform.matrix())
    xyz = _float_controls(matrix[:3, 3])
    rotation = matrix[:3, :3]
    # Match Eigen's eulerAngles(2, 1, 0) branch, whose yaw lies in [0, pi].
    yaw = math.atan2(rotation[1, 0], rotation[0, 0])
    cosine_pitch = math.hypot(rotation[2, 2], rotation[2, 1])
    if yaw < 0:
        yaw += math.pi
        pitch = math.atan2(-rotation[2, 0], -cosine_pitch)
    else:
        pitch = math.atan2(-rotation[2, 0], cosine_pitch)
    sine, cosine = math.sin(yaw), math.cos(yaw)
    roll = math.atan2(
        sine * rotation[0, 2] - cosine * rotation[1, 2],
        cosine * rotation[1, 1] - sine * rotation[0, 1],
    )
    return xyz, _float_controls(np.array([roll, pitch, yaw]) * 180 / math.pi)


class ArmSkeleton:
    def __init__(self, world):
        if world is None:
            raise ValueError("world must not be None")
        if not isinstance(world, dart.simulation.World):
            raise TypeError("world must be a World")
        self._world = world
        self._chain = []
        self._instances = []
        self._instance_counter = 0

    @property
    def world(self):
        return self._world

    def __del__(self):
        if hasattr(self, "_instances"):
            self._clear_instances()

    def _clear_instances(self):
        for skeleton in self._instances:
            self.world.removeSkeleton(skeleton)
        self._instances.clear()

    def build(self, chain):
        chain = list(chain)
        if any(not isinstance(spec, JointSpec) for spec in chain):
            raise TypeError("chain must contain JointSpec objects")
        self._clear_instances()
        self._chain = chain

    def show_solutions(self, configs, selected=0):
        configs = list(configs)
        selected = operator.index(selected)
        if (
            selected < -1
            or (configs and selected >= len(configs))
            or (not configs and selected > 0)
        ):
            raise ValueError("selected solution is out of range")
        dofs = len(self._chain)
        poses = []
        for config in configs:
            config = np.asarray(config, dtype=float)
            if config.ndim == 2 and config.shape[1] == 1:
                config = config[:, 0]
            if config.ndim != 1:
                raise TypeError("solution must be a vector")
            if not np.isfinite(config).all() or (dofs and config.size != dofs):
                raise ValueError("solution must be finite and match the arm DOFs")
            poses.append(config)
        if not self._chain:
            self._clear_instances()
            return
        wanted = min(len(poses), 16) if poses else 1
        while len(self._instances) < wanted:
            self._instances.append(self._build_instance())
        while len(self._instances) > wanted:
            self.world.removeSkeleton(self._instances.pop())
        for index, skeleton in enumerate(self._instances):
            skeleton.setPositions(poses[index] if poses else np.zeros(dofs))
            color = (
                [0.55, 0.58, 0.62, 0.5]
                if not poses
                else [0.90, 0.20, 0.18, 1.0]
                if index == selected
                else [0.85, 0.32, 0.30, 0.22]
            )
            for body in skeleton.getBodyNodes():
                for node in body.getShapeNodes():
                    node.getVisualAspect().setRGBA(color)

    def _build_instance(self):
        skeleton = dart.dynamics.Skeleton(f"ssik_arm_{self._instance_counter}")
        self._instance_counter += 1
        joint, parent = skeleton.createWeldJointAndBodyNodePair()
        joint.setName("base_joint")
        parent.setName("base")
        self._add_sphere(parent)
        self._add_cylinder(parent, np.zeros(3), self._chain[0].t_left[:3, 3])
        for index, spec in enumerate(self._chain):
            properties = (
                dart.dynamics.PrismaticJointProperties()
                if spec.prismatic
                else dart.dynamics.RevoluteJointProperties()
            )
            properties.mName = f"j{index}"
            properties.mAxis = spec.axis / np.linalg.norm(spec.axis)
            properties.mT_ParentBodyToJoint = dart.math.Isometry3(spec.t_left)
            right_inverse = dart.math.Isometry3(spec.t_right).inverse()
            properties.mT_ChildBodyToJoint = right_inverse
            create = (
                skeleton.createPrismaticJointAndBodyNodePair
                if spec.prismatic
                else skeleton.createRevoluteJointAndBodyNodePair
            )
            _, body = create(parent, properties)
            body.setName(f"b{index}")
            self._add_sphere(body)
            self._add_cylinder(body, right_inverse.translation(), np.zeros(3))
            if index + 1 < len(self._chain):
                self._add_cylinder(
                    body, np.zeros(3), self._chain[index + 1].t_left[:3, 3]
                )
            parent = body
        self.world.addSkeleton(skeleton)
        return skeleton

    @staticmethod
    def _add_sphere(body):
        node = body.createShapeNode(dart.dynamics.SphereShape(0.045))
        node.createVisualAspect()

    @staticmethod
    def _add_cylinder(body, a, b):
        delta = b - a
        length = np.linalg.norm(delta)
        if length < 1e-5:
            return
        direction = delta / length
        quaternion = dart.math.Quaternion.from_two_vectors([0, 0, 1], direction)
        node = body.createShapeNode(dart.dynamics.CylinderShape(0.028, length))
        node.createVisualAspect()
        node.setRelativeTransform(dart.math.Isometry3(quaternion, (a + b) * 0.5))
