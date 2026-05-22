from __future__ import annotations

from enum import Enum
from typing import overload

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
    "LoopClosure",
    "LoopClosureFamily",
    "LoopClosureSpec",
    "MultiBody",
    "RigidBody",
    "RigidBodyOptions",
    "World",
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


class LoopClosureFamily(Enum):
    RIGID = 0
    POINT = 1
    DISTANCE = 2


class JointSpec:
    def __init__(
        self,
        name: str = "",
        type: JointType = ...,
        axis: ArrayLike | None = None,
    ) -> None:
        ...

    name: str
    type: JointType
    axis: NDArray[np.float64]


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

    def get_name(self) -> str:
        ...

    def get_parent_frame(self) -> Frame:
        ...

    def set_parent_frame(self, parent: Frame) -> None:
        ...

    def get_local_transform(self) -> NDArray[np.float64]:
        ...

    @overload
    def get_transform(self) -> NDArray[np.float64]:
        ...

    @overload
    def get_transform(self, relative_to: Frame) -> NDArray[np.float64]:
        ...

    @overload
    def get_transform(
        self, to: Frame, expressed_in: Frame
    ) -> NDArray[np.float64]:
        ...

    def is_valid_handle(self) -> bool:
        ...

    def is_same_instance_as(self, other: Frame) -> bool:
        ...


class FreeFrame(Frame):
    local_transform: NDArray[np.float64]

    def set_local_transform(self, transform: ArrayLike) -> None:
        ...

    def get_local_transform(self) -> NDArray[np.float64]:
        ...


class FixedFrame(Frame):
    local_transform: NDArray[np.float64]

    def set_local_transform(self, transform: ArrayLike) -> None:
        ...

    def get_local_transform(self) -> NDArray[np.float64]:
        ...


class MultiBody:
    name: str
    num_links: int
    num_joints: int
    num_dofs: int

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
    translation: NDArray[np.float64]
    rotation: NDArray[np.float64]
    quaternion: NDArray[np.float64]
    transform: NDArray[np.float64]
    is_valid: bool

    def get_name(self) -> str:
        ...

    def get_parent_joint(self) -> Joint:
        ...


class Joint:
    name: str
    type: JointType
    axis: NDArray[np.float64]
    num_dofs: int
    position: NDArray[np.float64]
    velocity: NDArray[np.float64]
    parent_link: Link
    child_link: Link
    is_valid: bool

    def get_name(self) -> str:
        ...

    def get_type(self) -> JointType:
        ...

    def get_axis(self) -> NDArray[np.float64]:
        ...

    def get_num_dofs(self) -> int:
        ...

    def get_position(self) -> NDArray[np.float64]:
        ...

    def set_position(self, position: ArrayLike) -> None:
        ...

    def get_velocity(self) -> NDArray[np.float64]:
        ...

    def set_velocity(self, velocity: ArrayLike) -> None:
        ...

    def get_parent_link(self) -> Link:
        ...

    def get_child_link(self) -> Link:
        ...


class LoopClosure:
    name: str
    family: LoopClosureFamily
    frame_a: Frame
    frame_b: Frame
    offset_a: NDArray[np.float64]
    offset_b: NDArray[np.float64]
    is_valid: bool

    def get_name(self) -> str:
        ...

    def get_family(self) -> LoopClosureFamily:
        ...

    def get_frame_a(self) -> Frame:
        ...

    def get_frame_b(self) -> Frame:
        ...

    def get_offset_a(self) -> NDArray[np.float64]:
        ...

    def get_offset_b(self) -> NDArray[np.float64]:
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

    def get_name(self) -> str:
        ...

    def set_transform(self, transform: ArrayLike) -> None:
        ...

    def get_linear_velocity(self) -> NDArray[np.float64]:
        ...

    def set_linear_velocity(self, velocity: ArrayLike) -> None:
        ...

    def get_angular_velocity(self) -> NDArray[np.float64]:
        ...

    def set_angular_velocity(self, velocity: ArrayLike) -> None:
        ...

    def get_mass(self) -> float:
        ...

    def set_mass(self, mass: float) -> None:
        ...

    def get_inertia(self) -> NDArray[np.float64]:
        ...

    def set_inertia(self, inertia: ArrayLike) -> None:
        ...

    def get_force(self) -> NDArray[np.float64]:
        ...

    def set_force(self, force: ArrayLike) -> None:
        ...

    def apply_force(self, force: ArrayLike) -> None:
        ...

    def clear_force(self) -> None:
        ...

    def get_torque(self) -> NDArray[np.float64]:
        ...

    def set_torque(self, torque: ArrayLike) -> None:
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

    def get_multi_body(self, name: str) -> MultiBody | None:
        ...

    def get_multi_body_count(self) -> int:
        ...

    @overload
    def add_loop_closure(
        self, name: str, spec: LoopClosureSpec
    ) -> LoopClosure:
        ...

    @overload
    def add_loop_closure(
        self,
        name: str,
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

    def get_loop_closure_count(self) -> int:
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

    def get_rigid_body_count(self) -> int:
        ...

    def enter_simulation_mode(self) -> None:
        ...

    def update_kinematics(self) -> None:
        ...

    def step(self, n: int = 1) -> None:
        ...

    def clear(self) -> None:
        ...
