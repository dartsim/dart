from __future__ import annotations
import dartpy.common
import dartpy.dynamics
import dartpy.simulation
import typing
__all__: list[str] = ['Options', 'RootJointType', 'readSkeleton', 'readWorld']
class Options:
    mDefaultRootJointType: RootJointType
    mResourceRetriever: dartpy.common.ResourceRetriever
    def __init__(self, resourceRetriever: dartpy.common.ResourceRetriever = None, defaultRootJointType: RootJointType = ...) -> None:
        ...
class RootJointType:
    """
    Members:
    
      Floating
    
      Fixed
    """
    Fixed: typing.ClassVar[RootJointType]  # value = <RootJointType.Fixed: 1>
    Floating: typing.ClassVar[RootJointType]  # value = <RootJointType.Floating: 0>
    __members__: typing.ClassVar[dict[str, RootJointType]]  # value = {'Floating': <RootJointType.Floating: 0>, 'Fixed': <RootJointType.Fixed: 1>}
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
@typing.overload
def readSkeleton(uri: dartpy.common.Uri, options: Options = ...) -> dartpy.dynamics.Skeleton:
    ...
@typing.overload
def readWorld(uri: dartpy.common.Uri, options: Options = ...) -> dartpy.simulation.World:
    ...
