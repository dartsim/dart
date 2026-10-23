# DART manipulation parser fixture

Copyright (c) 2026, The DART development contributors.
All files in this directory are DART-authored under DART's BSD-2-Clause license
(the repository's top-level `LICENSE`). No geometry or model text is derived
from Fetch or Roboti.

`scene.xml` includes defaults and the four-vertex tetrahedron mesh from
`shared.xml`, then a body hierarchy from `bodies.xml`. The unit-mass tetrahedron
has center of mass (0.01, 0.01, 0.01), diagonal moments 0.00012 and off-diagonal
moments 0.00002, from its uniform-volume integrals. Two root geoms and four
root bodies produce six skeletons. The arm has two hinge joints and a welded
tip; the object and goal each have two slide joints; the ball has a free joint.
The goal and its heavy marker are visual-only, including where they overlap
the table.
Friction is inherited as zero on the table and 0.8 on the floor, and explicitly
overridden to 0.9 on the wrist. This replaces the Fetch include/mesh fixture
and the manipulation models used for collision-mask and friction regressions.
