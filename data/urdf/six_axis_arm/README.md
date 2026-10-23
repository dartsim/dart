# Six-axis arm

An independently authored DART teaching and regression model, named for its
six revolute axes rather than a manufacturer. The URDF, ground, and meshes
are original DART assets distributed under DART's BSD-2-Clause license (the
repository's top-level `LICENSE`). No vendor CAD or robot dimensions were used.

The fixed base supports base yaw, shoulder and elbow pitch, and a roll/pitch/roll
wrist. The links, from the base, are `base`, `shoulder`, `upper_arm` (the
vertical segment), `elbow` (the forward segment), `forearm` (the wrist-roll
housing), `wrist` and `tool`; the joints are `base_yaw`, `shoulder_pitch`,
`elbow_pitch`, `wrist_roll`, `wrist_pitch` and `tool_roll`. At zero joint
angles the upper arm stands vertically and the forearm and wrist point
forward, forming an upright L. The tool body origin is at
`(0.34, 0, 0.46)` m with identity rotation; its physical tip is at
`(0.38, 0, 0.46)` m. The shoulder-to-tip link lengths total 0.66 m.
Angles, effort limits, and masses are illustrative, not hardware specifications.

Each visual OBJ is a centered, capped 20-sided cylinder in meters, with color
supplied by the URDF material. Axial vertex rings are at most 0.04 m apart so
the point-cloud demo can sample along the links. Meshes have 80–360 triangles;
collision boxes enclose the cylinders. Centers of mass lie at the cylinder
centers. For mass `m`, radius `r`, and length `h`, the axial inertia is
`m*r*r/2` and each transverse inertia is `m*(3*r*r + h*h)/12`, with zero cross
terms. These solid-cylinder approximations ignore the small faceting of the
visual meshes. The separate ground has its top at `z = 0`.
