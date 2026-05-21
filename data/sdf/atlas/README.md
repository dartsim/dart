# Atlas Sample Model

This directory contains the Atlas sample model used by DART examples and tests.

`atlas_v5_no_head.urdf` and the files under `meshes_unplugged/` and
`materials/textures/` are derived from the DRCSim `atlas_description` package.
The original package metadata lists the `atlas_description` license as Apache
2.0. See
`LICENSE-DRCSIM-APACHE-2.0.txt`.

Source files:

- https://github.com/Hurisa/drcsim/blob/a2a606ae475e682df1d5214e54de2cbd4f9b016f/atlas_description/urdf/atlas_v5.urdf
- https://github.com/Hurisa/drcsim/tree/a2a606ae475e682df1d5214e54de2cbd4f9b016f/atlas_description/meshes_unplugged
- https://github.com/Hurisa/drcsim/tree/a2a606ae475e682df1d5214e54de2cbd4f9b016f/atlas_description/materials/textures

The v5 sample keeps the existing DART no-head convention used by the Atlas
examples: the upstream `neck_ry` joint was removed because the standalone
DRCSim `atlas_v5.urdf` references a `head` link supplied by a separate
multisense description include. Mesh references were also rewritten from
`package://atlas_description/...` URIs to paths relative to this directory so
the sample can load from DART's installed `data/` tree without a ROS package
index.

The older Atlas v3 sample files were removed for DART 7 so maintained examples
and parser tests use current, purpose-specific fixtures instead of preserving
unused legacy sample paths.
