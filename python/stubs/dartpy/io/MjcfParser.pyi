from __future__ import annotations

__all__: list[str] = [
    "Options",
]


import dartpy.common


class Options:
    def __init__(self, resource_retriever: dartpy.common.ResourceRetriever | None = ..., geom_skeleton_name_prefix: str = ..., site_skeleton_name_prefix: str = ...) -> None: ...

    @property
    def mRetriever(self) -> dartpy.common.ResourceRetriever: ...

    @mRetriever.setter
    def mRetriever(self, arg: dartpy.common.ResourceRetriever, /) -> None: ...

    @property
    def mGeomSkeletonNamePrefix(self) -> str: ...

    @mGeomSkeletonNamePrefix.setter
    def mGeomSkeletonNamePrefix(self, arg: str, /) -> None: ...

    @property
    def mSiteSkeletonNamePrefix(self) -> str: ...

    @mSiteSkeletonNamePrefix.setter
    def mSiteSkeletonNamePrefix(self, arg: str, /) -> None: ...

    @property
    def m_geom_skeleton_name_prefix(self) -> str: ...

    @m_geom_skeleton_name_prefix.setter
    def m_geom_skeleton_name_prefix(self, arg: str, /) -> None: ...

    @property
    def m_retriever(self) -> dartpy.common.ResourceRetriever: ...

    @m_retriever.setter
    def m_retriever(self, arg: dartpy.common.ResourceRetriever, /) -> None: ...

    @property
    def m_site_skeleton_name_prefix(self) -> str: ...

    @m_site_skeleton_name_prefix.setter
    def m_site_skeleton_name_prefix(self, arg: str, /) -> None: ...
