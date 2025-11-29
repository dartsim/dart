from __future__ import annotations
import dartpy.common
import dartpy.dynamics
import dartpy.simulation
import typing
from . import MjcfParser
from . import SdfParser
from . import SkelParser
__all__: list[str] = ['CompositeResourceRetriever', 'UrdfParser', 'UrdfParserOptions', 'UrdfParserRootJointType', 'DartLoader', 'DartLoaderOptions', 'DartLoaderRootJointType', 'DartResourceRetriever', 'MjcfParser', 'PackageResourceRetriever', 'SdfParser', 'SkelParser']
class CompositeResourceRetriever(dartpy.common.ResourceRetriever):
    def __init__(self) -> None:
        ...
    def addDefaultRetriever(self, resourceRetriever: dartpy.common.ResourceRetriever) -> None:
        ...
    def addSchemaRetriever(self, schema: str, resourceRetriever: dartpy.common.ResourceRetriever) -> bool:
        ...
class UrdfParserRootJointType:
    """
    Members:
    
      Floating
    
      Fixed
    """
    Fixed: typing.ClassVar[UrdfParserRootJointType]  # value = <UrdfParserRootJointType.Fixed: 1>
    Floating: typing.ClassVar[UrdfParserRootJointType]  # value = <UrdfParserRootJointType.Floating: 0>
    __members__: typing.ClassVar[dict[str, UrdfParserRootJointType]]  # value = {'Floating': <UrdfParserRootJointType.Floating: 0>, 'Fixed': <UrdfParserRootJointType.Fixed: 1>}
    def __eq__(self, other: typing.Any) -> bool:
        ...
    def __getstate__(self) -> int:
        ...
    def __hash__(self) -> int:
        ...
    def __index__(self) -> int:
        ...
    def __init__(self, value: int) -> None:
        ...
    def __int__(self) -> int:
        ...
    def __ne__(self, other: typing.Any) -> bool:
        ...
    def __repr__(self) -> str:
        ...
    def __setstate__(self, state: int) -> None:
        ...
    def __str__(self) -> str:
        ...
    @property
    def name(self) -> str:
        ...
    @property
    def value(self) -> int:
        ...
class UrdfParserOptions:
    mDefaultInertia: dartpy.dynamics.Inertia
    mDefaultRootJointType: UrdfParserRootJointType
    mResourceRetriever: dartpy.common.ResourceRetriever
    def __init__(self, resourceRetriever: dartpy.common.ResourceRetriever = None, defaultRootJointType: UrdfParserRootJointType = ..., defaultInertia: dartpy.dynamics.Inertia = ...) -> None:
        ...
class UrdfParser:
    Options = UrdfParserOptions
    RootJointType = UrdfParserRootJointType
    def __init__(self) -> None:
        ...
    def addPackageDirectory(self, packageName: str, packageDirectory: str) -> None:
        ...
    def getOptions(self) -> UrdfParserOptions:
        ...
    @typing.overload
    def parseSkeleton(self, uri: dartpy.common.Uri) -> dartpy.dynamics.Skeleton:
        ...
    @typing.overload
    def parseSkeletonString(self, urdfString: str, baseUri: dartpy.common.Uri) -> dartpy.dynamics.Skeleton:
        ...
    @typing.overload
    def parseWorld(self, uri: dartpy.common.Uri) -> dartpy.simulation.World:
        ...
    @typing.overload
    def parseWorldString(self, urdfString: str, baseUri: dartpy.common.Uri) -> dartpy.simulation.World:
        ...
    def setOptions(self, options: UrdfParserOptions = ...) -> None:
        ...
DartLoader = UrdfParser
DartLoaderOptions = UrdfParserOptions
DartLoaderRootJointType = UrdfParserRootJointType
class DartResourceRetriever(dartpy.common.ResourceRetriever):
    def __init__(self) -> None:
        ...
class PackageResourceRetriever(dartpy.common.ResourceRetriever):
    def __init__(self, localRetriever: dartpy.common.ResourceRetriever) -> None:
        ...
    def addPackageDirectory(self, packageName: str, packageDirectory: str) -> None:
        ...
