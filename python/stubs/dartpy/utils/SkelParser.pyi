from __future__ import annotations
import dartpy.common
import dartpy.dynamics
import dartpy.simulation
__all__: list[str] = ['readSkeleton', 'readWorld', 'readWorldXML']
def readSkeleton(uri: dartpy.common.Uri, retriever: dartpy.common.ResourceRetriever = None) -> dartpy.dynamics.Skeleton:
    ...
def readWorld(uri: dartpy.common.Uri, retriever: dartpy.common.ResourceRetriever = None) -> dartpy.simulation.World:
    ...
def readWorldXML(xmlString: str, baseUri: dartpy.common.Uri = '', retriever: dartpy.common.ResourceRetriever = None) -> dartpy.simulation.World:
    ...
