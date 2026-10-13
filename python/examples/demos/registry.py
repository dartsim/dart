"""Python scene modules in the native C++ catalog's order."""

import dartpy as dart

from .scenes import (
    atlas_puppet,
    biped_stand,
    drag_and_drop,
    modern_humanoids,
    operational_space_control,
    rigid_chain,
    rigid_cubes,
    rigid_loop,
)


def make_demo_scenes():
    modules = [
        rigid_cubes,
        rigid_chain,
        rigid_loop,
        biped_stand,
        operational_space_control,
        atlas_puppet,
    ]
    if hasattr(dart.utils, "ModelResourceRetriever"):
        modules.append(modern_humanoids)
    modules.append(drag_and_drop)
    return [scene for module in modules for scene in module.scenes()]
