# Atlas Sample Models

This directory contains Atlas sample models used by DART examples and tests.

`atlas_v5_no_head.urdf` and the files under `meshes_unplugged/` and
`materials/textures/` are derived from the DRCSim `atlas_description` package.
The original package metadata lists the `atlas_description` license as Apache
2.0. See
`LICENSE-DRCSIM-APACHE-2.0.txt`.

Source files:

- https://github.com/Hurisa/drcsim/blob/master/atlas_description/urdf/atlas_v5.urdf
- https://github.com/Hurisa/drcsim/tree/master/atlas_description/meshes_unplugged
- https://github.com/Hurisa/drcsim/tree/master/atlas_description/materials/textures

The v5 sample keeps the existing DART no-head convention used by the Atlas
examples: the upstream `neck_ry` joint was removed because the standalone
DRCSim `atlas_v5.urdf` references a `head` link supplied by a separate
multisense description include.
