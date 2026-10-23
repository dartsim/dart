# WAM sample model

The URDF packaging and all 16 STL meshes come from
[herb_description](https://github.com/personalrobotics/herb_description/tree/177eff2f6eb22fe3b396e681cf755bc36f7a457e),
which exported them from Barrett Technology's SolidWorks models.

Copyright (c) 2013, Carnegie Mellon University

They are BSD-3-Clause under the unmodified upstream
[LICENSE.bsd](LICENSE.bsd). See also the root
[NOTICE](../../../NOTICE).

DART expanded the xacro, added the fixed world joint, and changed `/j1` limits
to `[-2.6, 2.6]`. Visuals now use the eight upstream visual STLs
(`wam_base.STL` and `wam1.STL` through `wam7.STL`); collision STLs, joint
transforms, and inertias are unchanged. DART colors the links dark graphite
and the base light gray to approximate the removed textures. The STLs differ
slightly in geometry: the largest link-frame bounding-box difference from
the former COLLADA visuals is 12.09 mm at `/wam7`'s upper Z bound.

The eight COLLADA meshes and six JPEG textures formerly shipped here were
GPL-3.0-or-later (2010 RE2, Inc.), as the upstream README and GPL license
explain. They have been removed. DART had removed source-file metadata from
seven of those COLLADA files; none of those modified files remain.
