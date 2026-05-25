from __future__ import annotations

__all__: list[str] = [
    "readSkeleton",
    "readWorld",
    "readWorldXML",
]




def readWorld(*args, **kwargs): ...

def readWorldXML(*args, **kwargs): ...

def readSkeleton(*args, **kwargs): ...

read_skeleton = readSkeleton

read_world = readWorld

read_world_xml = readWorldXML
