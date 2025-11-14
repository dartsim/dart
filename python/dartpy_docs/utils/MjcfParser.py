from __future__ import annotations
import dartpy.common
import dartpy.simulation
__all__: list[str] = ['Options', 'readWorld']
class Options:
    mGeomSkeletonNamePrefix: str
    mRetriever: dartpy.common.ResourceRetriever
    mSiteSkeletonNamePrefix: str
    def __init__(self, resourceRetretrieverOrNullptrriever: dartpy.common.ResourceRetriever = None, geomSkeletonNamePrefix: str = '__geom_skel__', siteSkeletonNamePrefix: str = '__site_skel__') -> None:
        ...
def readWorld(uri: dartpy.common.Uri, options: Options = ...) -> dartpy.simulation.World:
    ...
