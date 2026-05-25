from __future__ import annotations

__all__: list[str] = [
    "Bullet",
    "CollisionDetectorType",
    "ConstraintSolver",
    "Dantzig",
    "Dart",
    "Experimental",
    "Fcl",
    "LcpSolverType",
    "Lemke",
    "Ode",
    "Pgs",
    "World",
    "WorldConfig",
]


import enum
from typing import Any, overload


class CollisionDetectorType(enum.Enum):
    Dart = 0

    Fcl = 1

    Bullet = 2

    Ode = 3

    Experimental = 4

Dart: CollisionDetectorType = CollisionDetectorType.Dart

Fcl: CollisionDetectorType = CollisionDetectorType.Fcl

Bullet: CollisionDetectorType = CollisionDetectorType.Bullet

Ode: CollisionDetectorType = CollisionDetectorType.Ode

Experimental: CollisionDetectorType = CollisionDetectorType.Experimental

class LcpSolverType(enum.Enum):
    Dantzig = 0

    Pgs = 1

    Lemke = 2

Dantzig: LcpSolverType = LcpSolverType.Dantzig

Pgs: LcpSolverType = LcpSolverType.Pgs

Lemke: LcpSolverType = LcpSolverType.Lemke

class WorldConfig:
    @overload
    def __init__(self) -> None: ...

    @overload
    def __init__(self, name: str) -> None: ...

    @property
    def name(self) -> str: ...

    @name.setter
    def name(self, arg: str, /) -> None: ...

    @property
    def collision_detector(self) -> CollisionDetectorType: ...

    @collision_detector.setter
    def collision_detector(self, arg: CollisionDetectorType, /) -> None: ...

    @property
    def primary_lcp_solver(self) -> LcpSolverType: ...

    @primary_lcp_solver.setter
    def primary_lcp_solver(self, arg: LcpSolverType, /) -> None: ...

    @property
    def secondary_lcp_solver(self) -> object: ...

    def set_secondary_lcp_solver(self, solver: LcpSolverType) -> None: ...

    def clear_secondary_lcp_solver(self) -> None: ...

class World:
    @overload
    def __init__(self) -> None: ...

    @overload
    def __init__(self, name: str) -> None: ...

    @overload
    def __init__(self, config: WorldConfig) -> None: ...

    @overload
    @staticmethod
    def create(name: str = ...) -> World: ...

    @overload
    @staticmethod
    def create(config: WorldConfig) -> World: ...

    def clone(self) -> World: ...

    def setName(*args, **kwargs) -> Any: ...

    def getName(*args, **kwargs) -> Any: ...

    def setGravity(*args, **kwargs) -> Any: ...

    def getGravity(*args, **kwargs) -> Any: ...

    def setTimeStep(*args, **kwargs) -> Any: ...

    def getTimeStep(*args, **kwargs) -> Any: ...

    def getSkeleton(*args, **kwargs) -> Any: ...

    def getNumSkeletons(*args, **kwargs) -> Any: ...

    def addSkeleton(*args, **kwargs) -> Any: ...

    def removeSkeleton(*args, **kwargs) -> Any: ...

    def removeAllSkeletons(*args, **kwargs) -> Any: ...

    def hasSkeleton(*args, **kwargs) -> Any: ...

    def getIndex(*args, **kwargs) -> Any: ...

    def getNumSimpleFrames(*args, **kwargs) -> Any: ...

    def getSimpleFrame(*args, **kwargs) -> Any: ...

    def addSimpleFrame(*args, **kwargs) -> Any: ...

    def removeSimpleFrame(*args, **kwargs) -> Any: ...

    def removeAllSimpleFrames(*args, **kwargs) -> Any: ...

    def checkCollision(*args, **kwargs) -> Any: ...

    def checkCollisionResult(*args, **kwargs) -> Any: ...

    def getLastCollisionResult(*args, **kwargs) -> Any: ...

    def reset(self) -> None: ...

    @overload
    def step(self) -> None: ...

    @overload
    def step(self, reset_command: bool) -> None: ...

    def setTime(*args, **kwargs) -> Any: ...

    def getTime(*args, **kwargs) -> Any: ...

    def getSimFrames(*args, **kwargs) -> Any: ...

    def getConstraintSolver(*args, **kwargs) -> Any: ...

    def bake(self) -> None: ...

    def __repr__(self) -> str: ...

    add_simple_frame = addSimpleFrame

    add_skeleton = addSkeleton

    check_collision = checkCollision

    check_collision_result = checkCollisionResult

    get_constraint_solver = getConstraintSolver

    get_gravity = getGravity

    get_index = getIndex

    get_last_collision_result = getLastCollisionResult

    get_name = getName

    get_num_simple_frames = getNumSimpleFrames

    get_num_skeletons = getNumSkeletons

    get_sim_frames = getSimFrames

    get_simple_frame = getSimpleFrame

    get_skeleton = getSkeleton

    get_time = getTime

    get_time_step = getTimeStep

    has_skeleton = hasSkeleton

    remove_all_simple_frames = removeAllSimpleFrames

    remove_all_skeletons = removeAllSkeletons

    remove_simple_frame = removeSimpleFrame

    remove_skeleton = removeSkeleton

    set_gravity = setGravity

    set_name = setName

    set_time = setTime

    set_time_step = setTimeStep

class ConstraintSolver:
    def getCollisionDetector(*args, **kwargs) -> Any: ...

    def setCollisionDetector(*args, **kwargs) -> Any: ...

    def getCollisionGroup(*args, **kwargs) -> Any: ...

    def getCollisionOption(*args, **kwargs) -> Any: ...

    def addConstraint(*args, **kwargs) -> Any: ...

    def removeConstraint(*args, **kwargs) -> Any: ...

    add_constraint = addConstraint

    get_collision_detector = getCollisionDetector

    get_collision_group = getCollisionGroup

    get_collision_option = getCollisionOption

    remove_constraint = removeConstraint

    set_collision_detector = setCollisionDetector
