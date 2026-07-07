from __future__ import annotations

__all__: list[str] = [
    "CompositeResourceRetriever",
    "DartResourceRetriever",
    "ModelFormat",
    "MjcfParser",
    "PackageResourceRetriever",
    "ReadOptions",
    "RootJointType",
    "SdfParser",
    "UrdfParser",
    "UrdfParserOptions",
    "UrdfParserRootJointType",
    "readSkeleton",
    "read_skeleton",
]


import enum
from typing import Any, TypeAlias

from . import (
    MjcfParser as MjcfParser,
    SdfParser as SdfParser,
)
import dartpy.common
import dartpy.dynamics


class ModelFormat(enum.Enum):
    AUTO = 0

    SDF = 1

    URDF = 2

    MJCF = 3

    USD = 4

class RootJointType(enum.Enum):
    FLOATING = 0

    FIXED = 1

class ReadOptions:
    def __init__(self) -> None: ...

    @property
    def format(self) -> ModelFormat: ...

    @format.setter
    def format(self, arg: ModelFormat, /) -> None: ...

    @property
    def resourceRetriever(self) -> dartpy.common.ResourceRetriever: ...

    @resourceRetriever.setter
    def resourceRetriever(self, arg: dartpy.common.ResourceRetriever, /) -> None: ...

    @property
    def resource_retriever(self) -> dartpy.common.ResourceRetriever: ...

    @resource_retriever.setter
    def resource_retriever(self, arg: dartpy.common.ResourceRetriever, /) -> None: ...

    @property
    def sdfDefaultRootJointType(self) -> RootJointType: ...

    @sdfDefaultRootJointType.setter
    def sdfDefaultRootJointType(self, arg: RootJointType, /) -> None: ...

    @property
    def sdf_default_root_joint_type(self) -> RootJointType: ...

    @sdf_default_root_joint_type.setter
    def sdf_default_root_joint_type(self, arg: RootJointType, /) -> None: ...

    def addPackageDirectory(self, package_name: str, package_directory: str) -> None: ...

    add_package_directory = addPackageDirectory

class CompositeResourceRetriever(dartpy.common.ResourceRetriever):
    def __init__(self) -> None: ...

    def addDefaultRetriever(*args, **kwargs) -> Any: ...

    def addSchemaRetriever(*args, **kwargs) -> Any: ...

    add_default_retriever = addDefaultRetriever

    add_schema_retriever = addSchemaRetriever

class DartResourceRetriever(dartpy.common.ResourceRetriever):
    def __init__(self) -> None: ...

class PackageResourceRetriever(dartpy.common.ResourceRetriever):
    def __init__(self, local_retriever: dartpy.common.ResourceRetriever) -> None: ...

    def addPackageDirectory(*args, **kwargs) -> Any: ...

    add_package_directory = addPackageDirectory

class UrdfParserRootJointType(enum.Enum):
    Floating = 0

    Fixed = 1

    FLOATING = 0

    FIXED = 1

class UrdfParserOptions:
    def __init__(self, resource_retriever: dartpy.common.ResourceRetriever | None = ..., default_root_joint_type: UrdfParserRootJointType = UrdfParserRootJointType.Floating, default_inertia: dartpy.dynamics.Inertia = ...) -> None: ...

    @property
    def mResourceRetriever(self) -> dartpy.common.ResourceRetriever: ...

    @mResourceRetriever.setter
    def mResourceRetriever(self, arg: dartpy.common.ResourceRetriever, /) -> None: ...

    @property
    def mDefaultRootJointType(self) -> UrdfParserRootJointType: ...

    @mDefaultRootJointType.setter
    def mDefaultRootJointType(self, arg: UrdfParserRootJointType, /) -> None: ...

    @property
    def mDefaultInertia(self) -> dartpy.dynamics.Inertia: ...

    @mDefaultInertia.setter
    def mDefaultInertia(self, arg: dartpy.dynamics.Inertia, /) -> None: ...

    @property
    def m_default_inertia(self) -> dartpy.dynamics.Inertia: ...

    @m_default_inertia.setter
    def m_default_inertia(self, arg: dartpy.dynamics.Inertia, /) -> None: ...

    @property
    def m_default_root_joint_type(self) -> UrdfParserRootJointType: ...

    @m_default_root_joint_type.setter
    def m_default_root_joint_type(self, arg: UrdfParserRootJointType, /) -> None: ...

    @property
    def m_resource_retriever(self) -> dartpy.common.ResourceRetriever: ...

    @m_resource_retriever.setter
    def m_resource_retriever(self, arg: dartpy.common.ResourceRetriever, /) -> None: ...

class UrdfParser:
    def __init__(self) -> None: ...

    def setOptions(*args, **kwargs) -> Any: ...

    def getOptions(*args, **kwargs) -> Any: ...

    def addPackageDirectory(*args, **kwargs) -> Any: ...

    def parseSkeleton(*args, **kwargs) -> Any: ...

    def parseSkeletonString(*args, **kwargs) -> Any: ...

    RootJointType: TypeAlias = UrdfParserRootJointType

    Options: TypeAlias = UrdfParserOptions

    add_package_directory = addPackageDirectory

    get_options = getOptions

    parse_skeleton = parseSkeleton

    parse_skeleton_string = parseSkeletonString

    set_options = setOptions

def readSkeleton(uri: dartpy.common.Uri | str, options: ReadOptions = ...) -> dartpy.dynamics.Skeleton | None: ...

read_skeleton = readSkeleton
