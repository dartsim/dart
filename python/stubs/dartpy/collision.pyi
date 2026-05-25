from __future__ import annotations

__all__: list[str] = [
    "BodyNodeCollisionFilter",
    "CollisionDetector",
    "CollisionFilter",
    "CollisionGroup",
    "CollisionObject",
    "CollisionOption",
    "CollisionResult",
    "ContinuousCollisionAdvancement",
    "ContinuousCollisionHit",
    "ContinuousCollisionOption",
    "ContinuousCollisionResult",
    "DartCollisionDetector",
    "RayHit",
    "RaycastOption",
    "RaycastResult",
]


from collections.abc import Sequence
import enum
from typing import Annotated, Any, overload

import numpy
from numpy.typing import NDArray


class CollisionFilter:
    pass

class BodyNodeCollisionFilter(CollisionFilter):
    def __init__(self) -> None: ...

    def addBodyNodePairToBlackList(*args, **kwargs) -> Any: ...

    def removeBodyNodePairFromBlackList(*args, **kwargs) -> Any: ...

    def removeAllBodyNodePairsFromBlackList(*args, **kwargs) -> Any: ...

    add_body_node_pair_to_black_list = addBodyNodePairToBlackList

    remove_all_body_node_pairs_from_black_list = removeAllBodyNodePairsFromBlackList

    remove_body_node_pair_from_black_list = removeBodyNodePairFromBlackList

class CollisionOption:
    def __init__(self, enable_contact: bool = ..., max_num_contacts: int = ..., collision_filter: CollisionFilter | None = ..., allow_negative_penetration_depth_contacts: bool = ...) -> None: ...

    @property
    def enableContact(self) -> bool: ...

    @enableContact.setter
    def enableContact(self, arg: bool, /) -> None: ...

    @property
    def maxNumContacts(self) -> int: ...

    @maxNumContacts.setter
    def maxNumContacts(self, arg: int, /) -> None: ...

    @property
    def allowNegativePenetrationDepthContacts(self) -> bool: ...

    @allowNegativePenetrationDepthContacts.setter
    def allowNegativePenetrationDepthContacts(self, arg: bool, /) -> None: ...

    @property
    def collisionFilter(self) -> CollisionFilter: ...

    @collisionFilter.setter
    def collisionFilter(self, arg: CollisionFilter, /) -> None: ...

    def __repr__(self) -> str: ...

    @property
    def allow_negative_penetration_depth_contacts(self) -> bool: ...

    @allow_negative_penetration_depth_contacts.setter
    def allow_negative_penetration_depth_contacts(self, arg: bool, /) -> None: ...

    @property
    def collision_filter(self) -> CollisionFilter: ...

    @collision_filter.setter
    def collision_filter(self, arg: CollisionFilter, /) -> None: ...

    @property
    def enable_contact(self) -> bool: ...

    @enable_contact.setter
    def enable_contact(self, arg: bool, /) -> None: ...

    @property
    def max_num_contacts(self) -> int: ...

    @max_num_contacts.setter
    def max_num_contacts(self, arg: int, /) -> None: ...

class CollisionResult:
    def __init__(self) -> None: ...

    def clear(self) -> None: ...

    def isCollision(*args, **kwargs) -> Any: ...

    def getNumContacts(*args, **kwargs) -> Any: ...

    def __repr__(self) -> str: ...

    get_num_contacts = getNumContacts

    is_collision = isCollision

class CollisionObject:
    def getCollisionDetector(*args, **kwargs) -> Any: ...

    def getShapeFrame(*args, **kwargs) -> Any: ...

    def getShape(*args, **kwargs) -> Any: ...

    def getTransform(*args, **kwargs) -> Any: ...

    get_collision_detector = getCollisionDetector

    get_shape = getShape

    get_shape_frame = getShapeFrame

    get_transform = getTransform

class CollisionDetector:
    def getType(*args, **kwargs) -> Any: ...

    def createCollisionGroup(*args, **kwargs) -> Any: ...

    create_collision_group = createCollisionGroup

    get_type = getType

class DartCollisionDetector(CollisionDetector):
    def __init__(self) -> None: ...

    def getStaticType(*args, **kwargs): ...

    get_static_type = getStaticType

class RaycastOption:
    def __init__(self) -> None: ...

    @property
    def mEnableAllHits(self) -> bool: ...

    @mEnableAllHits.setter
    def mEnableAllHits(self, arg: bool, /) -> None: ...

    @property
    def m_enable_all_hits(self) -> bool: ...

    @m_enable_all_hits.setter
    def m_enable_all_hits(self, arg: bool, /) -> None: ...

class RayHit:
    def __init__(self) -> None: ...

    @property
    def mCollisionObject(self) -> CollisionObject: ...

    @mCollisionObject.setter
    def mCollisionObject(self, arg: CollisionObject, /) -> None: ...

    @property
    def mPoint(self) -> Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')]: ...

    @mPoint.setter
    def mPoint(self, arg: Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')], /) -> None: ...

    @property
    def mFraction(self) -> float: ...

    @mFraction.setter
    def mFraction(self, arg: float, /) -> None: ...

    @property
    def mNormal(self) -> Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')]: ...

    @mNormal.setter
    def mNormal(self, arg: Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')], /) -> None: ...

    @property
    def m_collision_object(self) -> CollisionObject: ...

    @m_collision_object.setter
    def m_collision_object(self, arg: CollisionObject, /) -> None: ...

    @property
    def m_fraction(self) -> float: ...

    @m_fraction.setter
    def m_fraction(self, arg: float, /) -> None: ...

    @property
    def m_normal(self) -> Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')]: ...

    @m_normal.setter
    def m_normal(self, arg: Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')], /) -> None: ...

    @property
    def m_point(self) -> Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')]: ...

    @m_point.setter
    def m_point(self, arg: Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')], /) -> None: ...

class RaycastResult:
    def __init__(self) -> None: ...

    def clear(self) -> None: ...

    def hasHit(*args, **kwargs) -> Any: ...

    @property
    def mRayHits(self) -> list[RayHit]: ...

    @mRayHits.setter
    def mRayHits(self, arg: Sequence[RayHit], /) -> None: ...

    has_hit = hasHit

    @property
    def m_ray_hits(self) -> list[RayHit]: ...

    @m_ray_hits.setter
    def m_ray_hits(self, arg: Sequence[RayHit], /) -> None: ...

class ContinuousCollisionAdvancement(enum.Enum):
    Conservative = 0

    Fast = 1

class ContinuousCollisionOption:
    def __init__(self, enable_all_hits: bool = ..., sort_by_time_of_impact: bool = ..., tolerance: float = ..., max_iterations: int = ..., advancement: ContinuousCollisionAdvancement = ContinuousCollisionAdvancement.Conservative) -> None: ...

    @property
    def mEnableAllHits(self) -> bool: ...

    @mEnableAllHits.setter
    def mEnableAllHits(self, arg: bool, /) -> None: ...

    @property
    def mSortByTimeOfImpact(self) -> bool: ...

    @mSortByTimeOfImpact.setter
    def mSortByTimeOfImpact(self, arg: bool, /) -> None: ...

    @property
    def mTolerance(self) -> float: ...

    @mTolerance.setter
    def mTolerance(self, arg: float, /) -> None: ...

    @property
    def mMaxIterations(self) -> int: ...

    @mMaxIterations.setter
    def mMaxIterations(self, arg: int, /) -> None: ...

    @property
    def mAdvancement(self) -> ContinuousCollisionAdvancement: ...

    @mAdvancement.setter
    def mAdvancement(self, arg: ContinuousCollisionAdvancement, /) -> None: ...

    def __repr__(self) -> str: ...

    @property
    def m_advancement(self) -> ContinuousCollisionAdvancement: ...

    @m_advancement.setter
    def m_advancement(self, arg: ContinuousCollisionAdvancement, /) -> None: ...

    @property
    def m_enable_all_hits(self) -> bool: ...

    @m_enable_all_hits.setter
    def m_enable_all_hits(self, arg: bool, /) -> None: ...

    @property
    def m_max_iterations(self) -> int: ...

    @m_max_iterations.setter
    def m_max_iterations(self, arg: int, /) -> None: ...

    @property
    def m_sort_by_time_of_impact(self) -> bool: ...

    @m_sort_by_time_of_impact.setter
    def m_sort_by_time_of_impact(self, arg: bool, /) -> None: ...

    @property
    def m_tolerance(self) -> float: ...

    @m_tolerance.setter
    def m_tolerance(self, arg: float, /) -> None: ...

class ContinuousCollisionHit:
    def __init__(self) -> None: ...

    @property
    def mCollisionObject(self) -> CollisionObject: ...

    @mCollisionObject.setter
    def mCollisionObject(self, arg: CollisionObject, /) -> None: ...

    @property
    def mTimeOfImpact(self) -> float: ...

    @mTimeOfImpact.setter
    def mTimeOfImpact(self, arg: float, /) -> None: ...

    @property
    def mPoint(self) -> Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')]: ...

    @mPoint.setter
    def mPoint(self, arg: Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')], /) -> None: ...

    @property
    def mNormal(self) -> Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')]: ...

    @mNormal.setter
    def mNormal(self, arg: Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')], /) -> None: ...

    @property
    def m_collision_object(self) -> CollisionObject: ...

    @m_collision_object.setter
    def m_collision_object(self, arg: CollisionObject, /) -> None: ...

    @property
    def m_normal(self) -> Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')]: ...

    @m_normal.setter
    def m_normal(self, arg: Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')], /) -> None: ...

    @property
    def m_point(self) -> Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')]: ...

    @m_point.setter
    def m_point(self, arg: Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')], /) -> None: ...

    @property
    def m_time_of_impact(self) -> float: ...

    @m_time_of_impact.setter
    def m_time_of_impact(self, arg: float, /) -> None: ...

class ContinuousCollisionResult:
    def __init__(self) -> None: ...

    def clear(self) -> None: ...

    def hasHit(*args, **kwargs) -> Any: ...

    @property
    def mHits(self) -> list[ContinuousCollisionHit]: ...

    @mHits.setter
    def mHits(self, arg: Sequence[ContinuousCollisionHit], /) -> None: ...

    def __repr__(self) -> str: ...

    has_hit = hasHit

    @property
    def m_hits(self) -> list[ContinuousCollisionHit]: ...

    @m_hits.setter
    def m_hits(self, arg: Sequence[ContinuousCollisionHit], /) -> None: ...

class CollisionGroup:
    def addShapeFrame(*args, **kwargs) -> Any: ...

    def addShapeFramesOf(*args, **kwargs) -> Any: ...

    def removeAllShapeFrames(*args, **kwargs) -> Any: ...

    def getNumShapeFrames(*args, **kwargs) -> Any: ...

    @overload
    def collide(self, option: CollisionOption = ..., result: CollisionResult | None = ...) -> bool: ...

    @overload
    def collide(self, other: CollisionGroup, option: CollisionOption = ..., result: CollisionResult | None = ...) -> bool: ...

    def collideResult(*args, **kwargs) -> Any: ...

    def raycast(self, from_: object, to: object, option: RaycastOption = ..., result: RaycastResult | None = ...) -> bool: ...

    def raycastResult(*args, **kwargs) -> Any: ...

    def sphereCast(*args, **kwargs) -> Any: ...

    def sphereCastResult(*args, **kwargs) -> Any: ...

    def capsuleCast(*args, **kwargs) -> Any: ...

    def capsuleCastResult(*args, **kwargs) -> Any: ...

    add_shape_frame = addShapeFrame

    add_shape_frames_of = addShapeFramesOf

    capsule_cast = capsuleCast

    capsule_cast_result = capsuleCastResult

    collide_result = collideResult

    get_num_shape_frames = getNumShapeFrames

    raycast_result = raycastResult

    remove_all_shape_frames = removeAllShapeFrames

    sphere_cast = sphereCast

    sphere_cast_result = sphereCastResult
