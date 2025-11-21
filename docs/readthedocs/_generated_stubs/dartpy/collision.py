from __future__ import annotations
import dartpy.dynamics
import dartpy.math
import numpy
import typing
__all__: list[str] = ['BodyNodeCollisionFilter', 'BulletCollisionDetector', 'BulletCollisionGroup', 'CollisionDetector', 'CollisionFilter', 'CollisionGroup', 'CollisionObject', 'CollisionOption', 'CollisionResult', 'CompositeCollisionFilter', 'Contact', 'DARTCollisionDetector', 'DARTCollisionGroup', 'DistanceOption', 'DistanceResult', 'FCLCollisionDetector', 'FCLCollisionGroup', 'OdeCollisionDetector', 'OdeCollisionGroup', 'RayHit', 'RaycastOption', 'RaycastResult']
class BodyNodeCollisionFilter(CollisionFilter):
    def __init__(self) -> None_:
        ...
    def addBodyNodePairToBlackList(self, bodyNode1: dartpy.dynamics.BodyNode, bodyNode2: dartpy.dynamics.BodyNode) -> None_:
        """
        Add a BodyNode pair to the blacklist.
        """
    def ignoresCollision(self, object1: ..., object2: ...) -> bool:
        """
        Returns true if the given two CollisionObjects should be checked by the collision detector, false otherwise.
        """
    def removeAllBodyNodePairsFromBlackList(self) -> None_:
        """
        Remove all the BodyNode pairs from the blacklist.
        """
    def removeBodyNodePairFromBlackList(self, bodyNode1: dartpy.dynamics.BodyNode, bodyNode2: dartpy.dynamics.BodyNode) -> None_:
        """
        Remove a BodyNode pair from the blacklist.
        """
class BulletCollisionDetector(CollisionDetector):
    @staticmethod
    def getStaticType() -> str:
        ...
    def __init__(self) -> None_:
        ...
    def cloneWithoutCollisionObjects(self) -> CollisionDetector:
        ...
    def createCollisionGroup(self) -> CollisionGroup:
        ...
    def getType(self) -> str:
        ...
class BulletCollisionGroup(CollisionGroup):
    def __init__(self, collisionDetector: CollisionDetector) -> None_:
        ...
class CollisionDetector:
    def cloneWithoutCollisionObjects(self) -> CollisionDetector:
        ...
    def createCollisionGroup(self) -> ...:
        ...
    def getType(self) -> str:
        ...
class CollisionFilter:
    pass
class CollisionGroup:
    def addShapeFrame(self, shapeFrame: dartpy.dynamics.ShapeFrame) -> None_:
        ...
    def addShapeFrames(self, shapeFrames: ..., std_param_1: ...) -> None_:
        ...
    @typing.overload
    def addShapeFramesOf(self, shapeFrame: dartpy.dynamics.ShapeFrame) -> None_:
        """
        Adds a ShapeFrame
        """
    @typing.overload
    def addShapeFramesOf(self, shapeFrames: ..., std_param_2: ...) -> None_:
        """
        Adds ShapeFrames
        """
    @typing.overload
    def addShapeFramesOf(self, otherGroup: CollisionGroup) -> None_:
        """
        Adds ShapeFrames of other CollisionGroup
        """
    @typing.overload
    def addShapeFramesOf(self, body: dartpy.dynamics.BodyNode) -> None_:
        """
        Adds ShapeFrames of BodyNode
        """
    @typing.overload
    def addShapeFramesOf(self, skeleton: dartpy.dynamics.MetaSkeleton) -> None_:
        """
        Adds ShapeFrames of MetaSkeleton
        """
    @typing.overload
    def collide(self, option: CollisionOption = ..., result: CollisionResult = None) -> bool:
        """
        Performs collision check within this CollisionGroup
        """
    @typing.overload
    def collide(self, otherGroup: CollisionGroup, option: CollisionOption = ..., result: CollisionResult = None) -> bool:
        """
        Perform collision check against other CollisionGroup
        """
    def distance(self, option: DistanceOption = ..., result: DistanceResult = None) -> float:
        ...
    def getAutomaticUpdate(self) -> bool:
        ...
    @typing.overload
    def getCollisionDetector(self) -> CollisionDetector:
        ...
    @typing.overload
    def getCollisionDetector(self) -> CollisionDetector:
        ...
    def getNumShapeFrames(self) -> int:
        ...
    def hasShapeFrame(self, shapeFrame: dartpy.dynamics.ShapeFrame) -> bool:
        ...
    @typing.overload
    def raycast(self, from_: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], to: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> bool:
        ...
    @typing.overload
    def raycast(self, from_: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], to: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], option: RaycastOption) -> bool:
        ...
    @typing.overload
    def raycast(self, from_: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], to: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], option: RaycastOption, result: RaycastResult) -> bool:
        ...
    def removeAllShapeFrames(self) -> None_:
        ...
    def removeDeletedShapeFrames(self) -> None_:
        ...
    def removeShapeFrame(self, shapeFrame: dartpy.dynamics.ShapeFrame) -> None_:
        ...
    def removeShapeFrames(self, shapeFrames: ..., std_param_3: ...) -> None_:
        ...
    @typing.overload
    def removeShapeFramesOf(self, shapeFrame: dartpy.dynamics.ShapeFrame) -> None_:
        """
        Removes a ShapeFrame
        """
    @typing.overload
    def removeShapeFramesOf(self, shapeFrames: ..., std_param_4: ...) -> None_:
        """
        Removes ShapeFrames
        """
    @typing.overload
    def removeShapeFramesOf(self, otherGroup: CollisionGroup) -> None_:
        """
        Removes ShapeFrames of other CollisionGroup
        """
    @typing.overload
    def removeShapeFramesOf(self, body: dartpy.dynamics.BodyNode) -> None_:
        """
        Removes ShapeFrames of BodyNode
        """
    @typing.overload
    def removeShapeFramesOf(self, skeleton: dartpy.dynamics.MetaSkeleton) -> None_:
        """
        Removes ShapeFrames of MetaSkeleton
        """
    @typing.overload
    def setAutomaticUpdate(self) -> None_:
        ...
    @typing.overload
    def setAutomaticUpdate(self, automatic: bool) -> None_:
        ...
    def subscribeTo(self) -> None_:
        ...
    def update(self) -> None_:
        ...
class CollisionObject:
    @typing.overload
    def getCollisionDetector(self) -> ...:
        """
        Return collision detection engine associated with this CollisionObject.
        """
    @typing.overload
    def getCollisionDetector(self) -> ...:
        """
        Return collision detection engine associated with this CollisionObject.
        """
    def getShape(self) -> dartpy.dynamics.Shape:
        """
        Return the associated Shape.
        """
    def getShapeFrame(self) -> dartpy.dynamics.ShapeFrame:
        """
        Return the associated ShapeFrame.
        """
    def getTransform(self) -> dartpy.math.Isometry3:
        """
        Return the transformation of this CollisionObject in world coordinates.
        """
class CollisionOption:
    collisionFilter: CollisionFilter
    enableContact: bool
    maxNumContacts: int
    @typing.overload
    def __init__(self) -> None_:
        ...
    @typing.overload
    def __init__(self, enableContact: bool) -> None_:
        ...
    @typing.overload
    def __init__(self, enableContact: bool, maxNumContacts: int) -> None_:
        ...
    @typing.overload
    def __init__(self, enableContact: bool, maxNumContacts: int, collisionFilter: CollisionFilter) -> None_:
        ...
class CollisionResult:
    def __init__(self) -> None_:
        ...
    def addContact(self, contact: Contact) -> None_:
        """
        Add one contact.
        """
    def clear(self) -> None_:
        """
        Clear all the contacts.
        """
    def getCollidingBodyNodes(self) -> set[dartpy.dynamics.BodyNode]:
        """
        Return the set of BodyNodes that are in collision.
        """
    def getCollidingShapeFrames(self) -> set[dartpy.dynamics.ShapeFrame]:
        """
        Return the set of ShapeFrames that are in collision.
        """
    @typing.overload
    def getContact(self, index: int) -> Contact:
        """
        Return the index-th contact.
        """
    @typing.overload
    def getContact(self, index: int) -> Contact:
        """
        Return (const) the index-th contact.
        """
    def getContacts(self) -> list[Contact]:
        """
        Return contacts.
        """
    def getNumContacts(self) -> int:
        """
        Return number of contacts.
        """
    @typing.overload
    def inCollision(self, bn: dartpy.dynamics.BodyNode) -> bool:
        """
        Returns true if the given BodyNode is in collision.
        """
    @typing.overload
    def inCollision(self, frame: dartpy.dynamics.ShapeFrame) -> bool:
        """
        Returns true if the given ShapeFrame is in collision.
        """
    def isCollision(self) -> bool:
        """
        Return binary collision result.
        """
class CompositeCollisionFilter(CollisionFilter):
    def __init__(self) -> None_:
        ...
    def addCollisionFilter(self, filter: CollisionFilter) -> None_:
        """
        Adds a collision filter to this CompositeCollisionFilter.
        """
    def removeAllCollisionFilters(self) -> None_:
        """
        Removes all the collision filters from this CompositeCollisionFilter.
        """
    def removeCollisionFilter(self, filter: CollisionFilter) -> None_:
        """
        Removes a collision filter from this CompositeCollisionFilter.
        """
class Contact:
    collisionObject1: ...
    collisionObject2: ...
    force: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]
    normal: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]
    penetrationDepth: float
    point: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]
    triID1: int
    triID2: int
    userData: capsule
    @staticmethod
    def getNormalEpsilon() -> float:
        ...
    @staticmethod
    def getNormalEpsilonSquared() -> float:
        ...
    @staticmethod
    def isNonZeroNormal(normal: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> bool:
        ...
    @staticmethod
    def isZeroNormal(normal: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> bool:
        ...
    def __init__(self) -> None_:
        ...
class DARTCollisionDetector(CollisionDetector):
    @staticmethod
    def getStaticType() -> str:
        ...
    def __init__(self) -> None_:
        ...
    def cloneWithoutCollisionObjects(self) -> CollisionDetector:
        ...
    def createCollisionGroup(self) -> ...:
        ...
    def getType(self) -> str:
        ...
class DARTCollisionGroup(CollisionGroup):
    def __init__(self, collisionDetector: CollisionDetector) -> None_:
        ...
class DistanceOption:
    distanceFilter: ...
    distanceLowerBound: float
    enableNearestPoints: bool
    @typing.overload
    def __init__(self) -> None_:
        ...
    @typing.overload
    def __init__(self, enableNearestPoints: bool) -> None_:
        ...
    @typing.overload
    def __init__(self, enableNearestPoints: bool, distanceLowerBound: float) -> None_:
        ...
    @typing.overload
    def __init__(self, enableNearestPoints: bool, distanceLowerBound: float, distanceFilter: ...) -> None_:
        ...
class DistanceResult:
    minDistance: float
    nearestPoint1: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]
    nearestPoint2: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]
    shapeFrame1: dartpy.dynamics.ShapeFrame
    shapeFrame2: dartpy.dynamics.ShapeFrame
    unclampedMinDistance: float
    def __init__(self) -> None_:
        ...
    def clear(self) -> None_:
        ...
    def found(self) -> bool:
        ...
    def isMinDistanceClamped(self) -> bool:
        ...
class FCLCollisionDetector(CollisionDetector):
    class ContactPointComputationMethod:
        """
        Members:
        
          FCL
        
          DART
        """
        DART: typing.ClassVar[FCLCollisionDetector.ContactPointComputationMethod]  # value = <ContactPointComputationMethod.DART: 1>
        FCL: typing.ClassVar[FCLCollisionDetector.ContactPointComputationMethod]  # value = <ContactPointComputationMethod.FCL: 0>
        __members__: typing.ClassVar[dict[str, FCLCollisionDetector.ContactPointComputationMethod]]  # value = {'FCL': <ContactPointComputationMethod.FCL: 0>, 'DART': <ContactPointComputationMethod.DART: 1>}
        def __eq__(self, other: typing.Any) -> bool:
            ...
        def __getstate__(self) -> int:
            ...
        def __hash__(self) -> int:
            ...
        def __index__(self) -> int:
            ...
        def __init__(self, value: int) -> None_:
            ...
        def __int__(self) -> int:
            ...
        def __ne__(self, other: typing.Any) -> bool:
            ...
        def __repr__(self) -> str:
            ...
        def __setstate__(self, state: int) -> None_:
            ...
        def __str__(self) -> str:
            ...
        @property
        def name(self) -> str:
            ...
        @property
        def value(self) -> int:
            ...
    class PrimitiveShape:
        """
        Members:
        
          PRIMITIVE
        
          MESH
        """
        MESH: typing.ClassVar[FCLCollisionDetector.PrimitiveShape]  # value = <PrimitiveShape.MESH: 1>
        PRIMITIVE: typing.ClassVar[FCLCollisionDetector.PrimitiveShape]  # value = <PrimitiveShape.PRIMITIVE: 0>
        __members__: typing.ClassVar[dict[str, FCLCollisionDetector.PrimitiveShape]]  # value = {'PRIMITIVE': <PrimitiveShape.PRIMITIVE: 0>, 'MESH': <PrimitiveShape.MESH: 1>}
        def __eq__(self, other: typing.Any) -> bool:
            ...
        def __getstate__(self) -> int:
            ...
        def __hash__(self) -> int:
            ...
        def __index__(self) -> int:
            ...
        def __init__(self, value: int) -> None_:
            ...
        def __int__(self) -> int:
            ...
        def __ne__(self, other: typing.Any) -> bool:
            ...
        def __repr__(self) -> str:
            ...
        def __setstate__(self, state: int) -> None_:
            ...
        def __str__(self) -> str:
            ...
        @property
        def name(self) -> str:
            ...
        @property
        def value(self) -> int:
            ...
    DART: typing.ClassVar[FCLCollisionDetector.ContactPointComputationMethod]  # value = <ContactPointComputationMethod.DART: 1>
    FCL: typing.ClassVar[FCLCollisionDetector.ContactPointComputationMethod]  # value = <ContactPointComputationMethod.FCL: 0>
    MESH: typing.ClassVar[FCLCollisionDetector.PrimitiveShape]  # value = <PrimitiveShape.MESH: 1>
    PRIMITIVE: typing.ClassVar[FCLCollisionDetector.PrimitiveShape]  # value = <PrimitiveShape.PRIMITIVE: 0>
    @staticmethod
    def getStaticType() -> str:
        ...
    def __init__(self) -> None_:
        ...
    def cloneWithoutCollisionObjects(self) -> CollisionDetector:
        ...
    def createCollisionGroup(self) -> ...:
        ...
    def getContactPointComputationMethod(self) -> ...:
        ...
    def getPrimitiveShapeType(self) -> ...:
        ...
    def getType(self) -> str:
        ...
    def setContactPointComputationMethod(self, method: ...) -> None_:
        ...
    def setPrimitiveShapeType(self, type: ...) -> None_:
        ...
class FCLCollisionGroup(CollisionGroup):
    def __init__(self, collisionDetector: CollisionDetector) -> None_:
        ...
class OdeCollisionDetector(CollisionDetector):
    @staticmethod
    def getStaticType() -> str:
        ...
    def __init__(self) -> None_:
        ...
    def cloneWithoutCollisionObjects(self) -> CollisionDetector:
        ...
    def createCollisionGroup(self) -> CollisionGroup:
        ...
    def getType(self) -> str:
        ...
class OdeCollisionGroup(CollisionGroup):
    def __init__(self, collisionDetector: CollisionDetector) -> None_:
        ...
class RayHit:
    mCollisionObject: CollisionObject
    mFraction: float
    mNormal: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]
    mPoint: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]
    def __init__(self) -> None_:
        ...
class RaycastOption:
    mEnableAllHits: bool
    mSortByClosest: bool
    @typing.overload
    def __init__(self) -> None_:
        ...
    @typing.overload
    def __init__(self, enableAllHits: bool) -> None_:
        ...
    @typing.overload
    def __init__(self, enableAllHits: bool, sortByClosest: bool) -> None_:
        ...
class RaycastResult:
    mRayHits: list[RayHit]
    def __init__(self) -> None_:
        ...
    def clear(self) -> None_:
        ...
    def hasHit(self) -> bool:
        ...
