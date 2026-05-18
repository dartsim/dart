"""Simulation utilities backed by nanobind"""

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
    def create(name: str = 'world') -> World: ...

    @overload
    @staticmethod
    def create(config: WorldConfig) -> World: ...

    def clone(self) -> World: ...

    def setName(*args, **kwargs) -> Any:
        """setName(self, new_name: str) -> str"""

    def getName(*args, **kwargs) -> Any:
        """getName(self) -> str"""

    def setGravity(*args, **kwargs) -> Any:
        """
        setGravity(self, gravity: numpy.ndarray[dtype=float64, shape=(3), order='C']) -> None
        setGravity(self, gravity: object) -> None
        setGravity(self, x: float, y: float, z: float) -> None
        """

    def getGravity(*args, **kwargs) -> Any:
        """getGravity(self) -> numpy.ndarray[dtype=float64, shape=(3), order='C']"""

    def setTimeStep(*args, **kwargs) -> Any:
        """setTimeStep(self, time_step: float) -> None"""

    def getTimeStep(*args, **kwargs) -> Any:
        """getTimeStep(self) -> float"""

    def getSkeleton(*args, **kwargs) -> Any:
        """
        getSkeleton(self, index: int) -> dartpy._dartpy.dynamics.Skeleton
        getSkeleton(self, name: str) -> dartpy._dartpy.dynamics.Skeleton
        """

    def getNumSkeletons(*args, **kwargs) -> Any:
        """getNumSkeletons(self) -> int"""

    def addSkeleton(*args, **kwargs) -> Any:
        """addSkeleton(self, skeleton: dartpy._dartpy.dynamics.Skeleton) -> str"""

    def removeSkeleton(*args, **kwargs) -> Any:
        """
        removeSkeleton(self, skeleton: dartpy._dartpy.dynamics.Skeleton) -> None
        """

    def removeAllSkeletons(*args, **kwargs) -> Any:
        """removeAllSkeletons(self) -> set[dartpy._dartpy.dynamics.Skeleton]"""

    def hasSkeleton(*args, **kwargs) -> Any:
        """
        hasSkeleton(self, skeleton: dartpy._dartpy.dynamics.Skeleton) -> bool
        hasSkeleton(self, skeleton_name: str) -> bool
        """

    def getIndex(*args, **kwargs) -> Any:
        """getIndex(self, index: int) -> int"""

    def getNumSimpleFrames(*args, **kwargs) -> Any:
        """getNumSimpleFrames(self) -> int"""

    def getSimpleFrame(*args, **kwargs) -> Any:
        """
        getSimpleFrame(self, index: int) -> dartpy._dartpy.dynamics.SimpleFrame
        getSimpleFrame(self, name: str) -> dartpy._dartpy.dynamics.SimpleFrame
        """

    def addSimpleFrame(*args, **kwargs) -> Any:
        """
        addSimpleFrame(self, frame: dartpy._dartpy.dynamics.SimpleFrame) -> str
        """

    def removeSimpleFrame(*args, **kwargs) -> Any:
        """
        removeSimpleFrame(self, frame: dartpy._dartpy.dynamics.SimpleFrame) -> None
        """

    def removeAllSimpleFrames(*args, **kwargs) -> Any:
        """
        removeAllSimpleFrames(self) -> set[dartpy._dartpy.dynamics.SimpleFrame]
        """

    def checkCollision(*args, **kwargs) -> Any:
        """
        checkCollision(self) -> bool
        checkCollision(self, option: dartpy._dartpy.collision.CollisionOption) -> bool
        checkCollision(self, option: dartpy._dartpy.collision.CollisionOption, result: dartpy._dartpy.collision.CollisionResult) -> bool
        """

    def checkCollisionResult(*args, **kwargs) -> Any:
        """
        checkCollisionResult(self, option: dartpy._dartpy.collision.CollisionOption = CollisionOption(enable_contact=True, max_contacts=1000, allow_negative_penetration_depth_contacts=False, has_filter=False)) -> dartpy._dartpy.collision.CollisionResult
        """

    def getLastCollisionResult(*args, **kwargs) -> Any:
        """
        getLastCollisionResult(self) -> dartpy._dartpy.collision.CollisionResult
        """

    def reset(self) -> None: ...

    @overload
    def step(self) -> None: ...

    @overload
    def step(self, reset_command: bool) -> None: ...

    def setTime(*args, **kwargs) -> Any:
        """setTime(self, time: float) -> None"""

    def getTime(*args, **kwargs) -> Any:
        """getTime(self) -> float"""

    def getSimFrames(*args, **kwargs) -> Any:
        """getSimFrames(self) -> int"""

    def getConstraintSolver(*args, **kwargs) -> Any:
        """
        getConstraintSolver(self) -> dartpy._dartpy.simulation.ConstraintSolver
        """

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
    def getCollisionDetector(*args, **kwargs) -> Any:
        """
        getCollisionDetector(self) -> dartpy._dartpy.collision.CollisionDetector
        """

    def setCollisionDetector(*args, **kwargs) -> Any:
        """
        setCollisionDetector(self, detector: dartpy._dartpy.collision.CollisionDetector) -> None
        """

    def getCollisionGroup(*args, **kwargs) -> Any:
        """getCollisionGroup(self) -> dartpy._dartpy.collision.CollisionGroup"""

    def getCollisionOption(*args, **kwargs) -> Any:
        """getCollisionOption(self) -> dartpy._dartpy.collision.CollisionOption"""

    def addConstraint(*args, **kwargs) -> Any:
        """
        addConstraint(self, constraint: dartpy._dartpy.constraint.ConstraintBase) -> None
        """

    def removeConstraint(*args, **kwargs) -> Any:
        """
        removeConstraint(self, constraint: dartpy._dartpy.constraint.ConstraintBase) -> None
        """

    add_constraint = addConstraint

    get_collision_detector = getCollisionDetector

    get_collision_group = getCollisionGroup

    get_collision_option = getCollisionOption

    remove_constraint = removeConstraint

    set_collision_detector = setCollisionDetector
