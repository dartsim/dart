from __future__ import annotations

from enum import Enum
from typing import Sequence, overload

import numpy as np
from numpy.typing import ArrayLike, NDArray

__all__: list[str] = [
    "FixedFrame",
    "Frame",
    "FreeFrame",
    "Joint",
    "JointSpec",
    "JointType",
    "Link",
    "ClosureDynamicsPolicy",
    "ClosureKinematicsPolicy",
    "LoopClosure",
    "LoopClosureFamily",
    "LoopClosureResidual",
    "LoopClosureResidualCoordinates",
    "LoopClosureRuntimePolicy",
    "LoopClosureSpec",
    "MultiBody",
    "RigidBody",
    "RigidBodyOptions",
    "StateSpace",
    "StateVariable",
    "World",
    "WorldSyncStage",
]


class JointType(Enum):
    FIXED = 0
    REVOLUTE = 1
    PRISMATIC = 2
    SCREW = 3
    UNIVERSAL = 4
    BALL = 5
    PLANAR = 6
    FREE = 7
    CUSTOM = 8


class ActuatorType(Enum):
    FORCE = 0
    PASSIVE = 1
    SERVO = 2
    VELOCITY = 3
    ACCELERATION = 4
    LOCKED = 5
    MIMIC = 6


class LoopClosureFamily(Enum):
    RIGID = 0
    POINT = 1
    DISTANCE = 2


class LoopClosureResidualCoordinates(Enum):
    WORLD = 0


class WorldSyncStage(Enum):
    KINEMATICS = 0


class CollisionShapeType(Enum):
    SPHERE = 0
    BOX = 1


class CollisionShape:
    @staticmethod
    def sphere(radius: float) -> CollisionShape:
        ...

    @staticmethod
    def box(half_extents: ArrayLike) -> CollisionShape:
        ...

    type: CollisionShapeType
    radius: float
    half_extents: NDArray[np.float64]


class CollisionBody:
    name: str
    is_rigid_body: bool
    is_link: bool
    is_valid: bool

    def as_rigid_body(self) -> RigidBody | None:
        ...

    def as_link(self) -> Link | None:
        ...


class Contact:
    body_a: CollisionBody
    body_b: CollisionBody
    point: NDArray[np.float64]
    normal: NDArray[np.float64]
    depth: float


class ClosureKinematicsPolicy(Enum):
    RESIDUAL_ONLY = 0
    PROJECT = 1


class ClosureDynamicsPolicy(Enum):
    RESIDUAL_ONLY = 0
    SOLVE = 1


class StateVariable:
    name: str
    start_index: int
    dimension: int
    lower_bound: float
    upper_bound: float


class StateSpace:
    def __init__(self) -> None:
        ...

    dimension: int
    num_variables: int
    is_finalized: bool
    variables: list[StateVariable]
    variable_names: list[str]
    lower_bounds: NDArray[np.float64]
    upper_bounds: NDArray[np.float64]

    def add_variable(
        self,
        name: str,
        dimension: int,
        *,
        lower: float = ...,
        upper: float = ...,
    ) -> StateSpace:
        ...

    def add_variables(
        self,
        names: Sequence[str],
        *,
        lower: float = ...,
        upper: float = ...,
    ) -> StateSpace:
        ...

    def finalize(self) -> None:
        ...

    def has_variable(self, name: str) -> bool:
        ...

    def get_variable(self, name: str) -> StateVariable | None:
        ...

    def get_variable_index(self, name: str) -> int | None:
        ...


class JointSpec:
    def __init__(
        self,
        name: str = "",
        type: JointType = ...,
        axis: ArrayLike | None = None,
        axis2: ArrayLike | None = None,
        transform_from_parent: ArrayLike | None = None,
    ) -> None:
        ...

    name: str
    type: JointType
    axis: NDArray[np.float64]
    axis2: NDArray[np.float64]
    transform_from_parent: NDArray[np.float64]


class LoopClosureSpec:
    def __init__(
        self,
        frame_a: Frame,
        frame_b: Frame,
        family: LoopClosureFamily = ...,
        *,
        offset_a: ArrayLike | None = None,
        offset_b: ArrayLike | None = None,
    ) -> None:
        ...

    frame_a: Frame
    frame_b: Frame
    family: LoopClosureFamily
    offset_a: NDArray[np.float64]
    offset_b: NDArray[np.float64]


class LoopClosureRuntimePolicy:
    def __init__(
        self,
        enabled: bool = True,
        kinematics: ClosureKinematicsPolicy = ...,
        dynamics: ClosureDynamicsPolicy = ...,
    ) -> None:
        ...

    enabled: bool
    kinematics: ClosureKinematicsPolicy
    dynamics: ClosureDynamicsPolicy


class LoopClosureResidual:
    value: NDArray[np.float64]
    norm: float
    enabled: bool
    active: bool
    coordinates: LoopClosureResidualCoordinates
    force_available: bool


class RigidBodyOptions:
    def __init__(
        self,
        mass: float = 1.0,
        position: ArrayLike | None = None,
        orientation: ArrayLike | None = None,
        linear_velocity: ArrayLike | None = None,
        angular_velocity: ArrayLike | None = None,
        inertia: ArrayLike | None = None,
    ) -> None:
        ...

    mass: float
    position: NDArray[np.float64]
    orientation: NDArray[np.float64]
    linear_velocity: NDArray[np.float64]
    angular_velocity: NDArray[np.float64]
    inertia: NDArray[np.float64]
    is_static: bool


class Frame:
    name: str
    parent_frame: Frame
    local_transform: NDArray[np.float64]
    translation: NDArray[np.float64]
    rotation: NDArray[np.float64]
    quaternion: NDArray[np.float64]
    transform: NDArray[np.float64]
    is_valid: bool
    is_world: bool

    @staticmethod
    def world() -> Frame:
        ...

    @overload
    def relative_transform(self, relative_to: Frame) -> NDArray[np.float64]:
        ...

    @overload
    def relative_transform(
        self, relative_to: Frame, expressed_in: Frame
    ) -> NDArray[np.float64]:
        ...

    def is_same_instance_as(self, other: Frame) -> bool:
        ...


class FreeFrame(Frame):
    local_transform: NDArray[np.float64]


class FixedFrame(Frame):
    local_transform: NDArray[np.float64]


class MultiBody:
    name: str
    num_links: int
    num_joints: int
    num_dofs: int
    links: list[Link]
    joints: list[Joint]
    link_names: list[str]
    joint_names: list[str]
    mass_matrix: NDArray[np.float64]
    inverse_mass_matrix: NDArray[np.float64]
    coriolis_forces: NDArray[np.float64]
    gravity_forces: NDArray[np.float64]
    coriolis_and_gravity_forces: NDArray[np.float64]
    is_valid: bool

    def compute_inverse_dynamics(
        self, desired_acceleration: ArrayLike
    ) -> NDArray[np.float64]:
        ...

    def compute_impulse_response(
        self, joint_impulse: ArrayLike
    ) -> NDArray[np.float64]:
        ...

    def get_jacobian(self, link: Link) -> NDArray[np.float64]:
        ...

    def get_world_jacobian(self, link: Link) -> NDArray[np.float64]:
        ...

    @overload
    def add_link(self, name: str = "") -> Link:
        ...

    @overload
    def add_link(
        self, name: str, *, parent: Link, joint: JointSpec = ...
    ) -> Link:
        ...

    def get_link(self, name: str) -> Link | None:
        ...

    def get_joint(self, name: str) -> Joint | None:
        ...


class Link(Frame):
    name: str
    parent_joint: Joint
    mass: float
    inertia: NDArray[np.float64]
    center_of_mass: NDArray[np.float64]
    translation: NDArray[np.float64]
    rotation: NDArray[np.float64]
    quaternion: NDArray[np.float64]
    transform: NDArray[np.float64]
    collision_shape: CollisionShape | None
    has_collision_shape: bool
    is_valid: bool

    def set_collision_shape(self, shape: CollisionShape) -> None:
        ...


class Joint:
    name: str
    type: JointType
    actuator_type: ActuatorType
    command_velocity: NDArray[np.float64]
    axis: NDArray[np.float64]
    axis2: NDArray[np.float64]
    pitch: float
    num_dofs: int
    position: NDArray[np.float64]
    velocity: NDArray[np.float64]
    force: NDArray[np.float64]
    acceleration: NDArray[np.float64]
    spring_stiffness: NDArray[np.float64]
    rest_position: NDArray[np.float64]
    damping_coefficient: NDArray[np.float64]
    armature: NDArray[np.float64]
    coulomb_friction: NDArray[np.float64]
    position_lower_limits: NDArray[np.float64]
    position_upper_limits: NDArray[np.float64]
    velocity_lower_limits: NDArray[np.float64]
    velocity_upper_limits: NDArray[np.float64]
    effort_lower_limits: NDArray[np.float64]
    effort_upper_limits: NDArray[np.float64]
    parent_link: Link
    child_link: Link
    is_valid: bool

    def set_position_limits(self, lower: ArrayLike, upper: ArrayLike) -> None:
        ...

    def set_velocity_limits(self, lower: ArrayLike, upper: ArrayLike) -> None:
        ...

    def set_effort_limits(self, lower: ArrayLike, upper: ArrayLike) -> None:
        ...


class LoopClosure:
    name: str
    family: LoopClosureFamily
    frame_a: Frame
    frame_b: Frame
    offset_a: NDArray[np.float64]
    offset_b: NDArray[np.float64]
    runtime_policy: LoopClosureRuntimePolicy
    enabled: bool
    kinematics: ClosureKinematicsPolicy
    dynamics: ClosureDynamicsPolicy
    is_valid: bool

    def compute_residual(self) -> LoopClosureResidual:
        ...


class RigidBody(Frame):
    name: str
    translation: NDArray[np.float64]
    rotation: NDArray[np.float64]
    quaternion: NDArray[np.float64]
    transform: NDArray[np.float64]
    linear_velocity: NDArray[np.float64]
    angular_velocity: NDArray[np.float64]
    mass: float
    inertia: NDArray[np.float64]
    force: NDArray[np.float64]
    torque: NDArray[np.float64]
    linear_momentum: NDArray[np.float64]
    angular_momentum: NDArray[np.float64]
    kinetic_energy: float
    potential_energy: float
    is_static: bool
    restitution: float
    friction: float
    collision_shape: CollisionShape | None
    has_collision_shape: bool

    def set_collision_shape(self, shape: CollisionShape) -> None:
        ...

    def apply_force(self, force: ArrayLike) -> None:
        ...

    def clear_force(self) -> None:
        ...

    def apply_torque(self, torque: ArrayLike) -> None:
        ...

    def clear_torque(self) -> None:
        ...


class World:
    def __init__(self, time_step: float = 0.001) -> None:
        ...

    time_step: float
    time: float
    gravity: NDArray[np.float64]
    frame: int
    is_simulation_mode: bool
    num_multi_bodies: int
    num_loop_closures: int
    num_rigid_bodies: int

    def add_free_frame(
        self, name: str = "", *, parent: Frame | None = None
    ) -> FreeFrame:
        ...

    def add_fixed_frame(
        self,
        name: str,
        parent: Frame,
        offset: ArrayLike | None = None,
    ) -> FixedFrame:
        ...

    def add_multi_body(self, name: str) -> MultiBody:
        ...

    def has_multi_body(self, name: str) -> bool:
        ...

    def get_multi_body(self, name: str) -> MultiBody | None:
        ...

    @overload
    def add_loop_closure(
        self, spec: LoopClosureSpec, *, name: str | None = ...
    ) -> LoopClosure:
        ...

    @overload
    def add_loop_closure(
        self, name: str | None, spec: LoopClosureSpec
    ) -> LoopClosure:
        ...

    @overload
    def add_loop_closure(
        self,
        name: str | None = ...,
        *,
        frame_a: Frame,
        frame_b: Frame,
        family: LoopClosureFamily = ...,
        offset_a: ArrayLike | None = None,
        offset_b: ArrayLike | None = None,
    ) -> LoopClosure:
        ...

    def has_loop_closure(self, name: str) -> bool:
        ...

    def get_loop_closure(self, name: str) -> LoopClosure | None:
        ...

    def add_rigid_body(
        self,
        name: str,
        options: RigidBodyOptions = ...,
        *,
        mass: float | None = None,
        position: ArrayLike | None = None,
        orientation: ArrayLike | None = None,
        linear_velocity: ArrayLike | None = None,
        angular_velocity: ArrayLike | None = None,
        inertia: ArrayLike | None = None,
    ) -> RigidBody:
        ...

    def has_rigid_body(self, name: str) -> bool:
        ...

    def get_rigid_body(self, name: str) -> RigidBody | None:
        ...

    def enter_simulation_mode(self) -> None:
        ...

    def update_kinematics(self) -> None:
        ...

    def sync(self, stage: WorldSyncStage = ...) -> None:
        ...

    def step(self, n: int = 1) -> None:
        ...

    def collide(self) -> list[Contact]:
        ...

    def clear(self) -> None:
        ...
