from __future__ import annotations

__all__: list[str] = [
    "Options",
    "RootJointType",
    "readSkeleton",
    "read_skeleton",
]


import enum

import dartpy.common


class RootJointType(enum.Enum):
    Floating = 0

    Fixed = 1

    FLOATING = 0

    FIXED = 1

class Options:
    def __init__(self, resource_retriever: dartpy.common.ResourceRetriever | None = ..., default_root_joint_type: RootJointType = RootJointType.Floating) -> None: ...

    @property
    def mResourceRetriever(self) -> dartpy.common.ResourceRetriever: ...

    @mResourceRetriever.setter
    def mResourceRetriever(self, arg: dartpy.common.ResourceRetriever, /) -> None: ...

    @property
    def mDefaultRootJointType(self) -> RootJointType: ...

    @mDefaultRootJointType.setter
    def mDefaultRootJointType(self, arg: RootJointType, /) -> None: ...

    @property
    def m_default_root_joint_type(self) -> RootJointType: ...

    @m_default_root_joint_type.setter
    def m_default_root_joint_type(self, arg: RootJointType, /) -> None: ...

    @property
    def m_resource_retriever(self) -> dartpy.common.ResourceRetriever: ...

    @m_resource_retriever.setter
    def m_resource_retriever(self, arg: dartpy.common.ResourceRetriever, /) -> None: ...

def readSkeleton(*args, **kwargs): ...

read_skeleton = readSkeleton
