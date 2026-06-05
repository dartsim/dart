from __future__ import annotations

__all__: list[str] = [
    "Composite",
    "LocalResource",
    "LocalResourceRetriever",
    "Observer",
    "Resource",
    "ResourceRetriever",
    "ResourceSeekType",
    "Stopwatch",
    "Subject",
    "Uri",
    "debug",
    "error",
    "fatal",
    "getProfileSummaryText",
    "get_profile_summary_text",
    "info",
    "isProfileEnabled",
    "isTextProfileEnabled",
    "is_profile_enabled",
    "is_text_profile_enabled",
    "markProfileFrame",
    "mark_profile_frame",
    "resetProfile",
    "reset_profile",
    "split",
    "tic",
    "toLower",
    "toUpper",
    "to_lower",
    "to_upper",
    "toc",
    "tocMS",
    "tocNS",
    "tocS",
    "tocUS",
    "toc_ms",
    "toc_ns",
    "toc_s",
    "toc_us",
    "trace",
    "trim",
    "trimLeft",
    "trimRight",
    "trim_left",
    "trim_right",
    "warn",
]


import enum
from typing import Any, overload

import typing_extensions


def trace(message: str) -> None: ...

def debug(message: str) -> None: ...

def info(message: str) -> None: ...

def warn(message: str) -> None: ...

def error(message: str) -> None: ...

def fatal(message: str) -> None: ...

class Observer:
    pass

class Subject:
    pass

class Composite:
    def __init__(self) -> None: ...

    def setCompositeState(*args, **kwargs) -> Any: ...

    def getCompositeState(*args, **kwargs) -> Any: ...

    def copyCompositeStateTo(*args, **kwargs) -> Any: ...

    def setCompositeProperties(*args, **kwargs) -> Any: ...

    def getCompositeProperties(*args, **kwargs) -> Any: ...

    def copyCompositePropertiesTo(*args, **kwargs) -> Any: ...

    def duplicateAspects(*args, **kwargs) -> Any: ...

    def matchAspects(*args, **kwargs) -> Any: ...

    copy_composite_properties_to = copyCompositePropertiesTo

    copy_composite_state_to = copyCompositeStateTo

    duplicate_aspects = duplicateAspects

    get_composite_properties = getCompositeProperties

    get_composite_state = getCompositeState

    match_aspects = matchAspects

    set_composite_properties = setCompositeProperties

    set_composite_state = setCompositeState

def isProfileEnabled(*args, **kwargs): ...

def isTextProfileEnabled(*args, **kwargs): ...

def markProfileFrame(*args, **kwargs): ...

def resetProfile(*args, **kwargs): ...

def getProfileSummaryText(*args, **kwargs): ...

class ResourceSeekType(enum.Enum):
    SEEKTYPE_CUR = 0

    SEEKTYPE_END = 1

    SEEKTYPE_SET = 2

class Resource:
    def getSize(*args, **kwargs) -> Any: ...

    def tell(self) -> int: ...

    def seek(self, offset: int, origin: ResourceSeekType) -> bool: ...

    def read(self, buffer: typing_extensions.CapsuleType, size: int, count: int) -> int: ...

    def readAll(*args, **kwargs) -> Any: ...

    get_size = getSize

    read_all = readAll

class LocalResource(Resource):
    def __init__(self, path: str) -> None: ...

    def isGood(*args, **kwargs) -> Any: ...

    get_size = getSize

    def getSize(*args, **kwargs) -> Any: ...

    is_good = isGood

    read_all = readAll

    def readAll(*args, **kwargs) -> Any: ...

class ResourceRetriever:
    def exists(self, uri: Uri) -> bool: ...

    def retrieve(self, uri: Uri) -> Resource: ...

    def readAll(*args, **kwargs) -> Any: ...

    def getFilePath(*args, **kwargs) -> Any: ...

    get_file_path = getFilePath

    read_all = readAll

class LocalResourceRetriever(ResourceRetriever):
    def __init__(self) -> None: ...

    get_file_path = getFilePath

    def getFilePath(*args, **kwargs) -> Any: ...

    read_all = readAll

    def readAll(*args, **kwargs) -> Any: ...

class Stopwatch:
    def __init__(self, start: bool = ...) -> None: ...

    def isStarted(*args, **kwargs) -> Any: ...

    def start(self) -> None: ...

    def stop(self) -> None: ...

    def reset(self) -> None: ...

    def elapsedS(*args, **kwargs) -> Any: ...

    def elapsedMS(*args, **kwargs) -> Any: ...

    def elapsedUS(*args, **kwargs) -> Any: ...

    def elapsedNS(*args, **kwargs) -> Any: ...

    def print(self) -> None: ...

    elapsed_ms = elapsedMS

    elapsed_ns = elapsedNS

    elapsed_s = elapsedS

    elapsed_us = elapsedUS

    is_started = isStarted

def tic() -> None: ...

def toc(print: bool = ...) -> float: ...

def tocS(*args, **kwargs): ...

def tocMS(*args, **kwargs): ...

def tocUS(*args, **kwargs): ...

def tocNS(*args, **kwargs): ...

def toUpper(*args, **kwargs): ...

def toLower(*args, **kwargs): ...

def trim(str: str, whitespaces: str = ...) -> str: ...

def trimLeft(*args, **kwargs): ...

def trimRight(*args, **kwargs): ...

def split(str: str, delimiters: str = ...) -> list[str]: ...

class Uri:
    @overload
    def __init__(self) -> None: ...

    @overload
    def __init__(self, input: str) -> None: ...

    @overload
    def __init__(self, input: str) -> None: ...

    def clear(self) -> None: ...

    def fromString(*args, **kwargs) -> Any: ...

    def fromPath(*args, **kwargs) -> Any: ...

    def fromStringOrPath(*args, **kwargs) -> Any: ...

    def fromRelativeUri(*args, **kwargs) -> Any: ...

    def toString(*args, **kwargs) -> Any: ...

    def getPath(*args, **kwargs) -> Any: ...

    def getFilesystemPath(*args, **kwargs) -> Any: ...

    def createFromString(*args, **kwargs): ...

    def createFromPath(*args, **kwargs): ...

    def createFromStringOrPath(*args, **kwargs): ...

    def createFromRelativeUri(*args, **kwargs): ...

    def getUri(*args, **kwargs): ...

    def getRelativeUri(*args, **kwargs): ...

    @property
    def mScheme(self) -> "dart::common::UriComponent": ...

    @mScheme.setter
    def mScheme(self, arg: "dart::common::UriComponent", /) -> None: ...

    @property
    def mAuthority(self) -> "dart::common::UriComponent": ...

    @mAuthority.setter
    def mAuthority(self, arg: "dart::common::UriComponent", /) -> None: ...

    @property
    def mPath(self) -> "dart::common::UriComponent": ...

    @mPath.setter
    def mPath(self, arg: "dart::common::UriComponent", /) -> None: ...

    @property
    def mQuery(self) -> "dart::common::UriComponent": ...

    @mQuery.setter
    def mQuery(self, arg: "dart::common::UriComponent", /) -> None: ...

    @property
    def mFragment(self) -> "dart::common::UriComponent": ...

    @mFragment.setter
    def mFragment(self, arg: "dart::common::UriComponent", /) -> None: ...

    create_from_path = createFromPath

    create_from_relative_uri = createFromRelativeUri

    create_from_string = createFromString

    create_from_string_or_path = createFromStringOrPath

    from_path = fromPath

    from_relative_uri = fromRelativeUri

    from_string = fromString

    from_string_or_path = fromStringOrPath

    get_filesystem_path = getFilesystemPath

    get_path = getPath

    get_relative_uri = getRelativeUri

    get_uri = getUri

    @property
    def m_authority(self) -> "dart::common::UriComponent": ...

    @m_authority.setter
    def m_authority(self, arg: "dart::common::UriComponent", /) -> None: ...

    @property
    def m_fragment(self) -> "dart::common::UriComponent": ...

    @m_fragment.setter
    def m_fragment(self, arg: "dart::common::UriComponent", /) -> None: ...

    @property
    def m_path(self) -> "dart::common::UriComponent": ...

    @m_path.setter
    def m_path(self, arg: "dart::common::UriComponent", /) -> None: ...

    @property
    def m_query(self) -> "dart::common::UriComponent": ...

    @m_query.setter
    def m_query(self, arg: "dart::common::UriComponent", /) -> None: ...

    @property
    def m_scheme(self) -> "dart::common::UriComponent": ...

    @m_scheme.setter
    def m_scheme(self, arg: "dart::common::UriComponent", /) -> None: ...

    to_string = toString

get_profile_summary_text = getProfileSummaryText

is_profile_enabled = isProfileEnabled

is_text_profile_enabled = isTextProfileEnabled

mark_profile_frame = markProfileFrame

reset_profile = resetProfile

to_lower = toLower

to_upper = toUpper

toc_ms = tocMS

toc_ns = tocNS

toc_s = tocS

toc_us = tocUS

trim_left = trimLeft

trim_right = trimRight
