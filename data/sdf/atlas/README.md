# Atlas v3 sample model

The 64 meshes are unchanged copies from OSRF's
[DRCSim Atlas description](https://github.com/osrf/drcsim/tree/24db7f7b2aa5dee1240c81a73eed8e29cbde6eb3/atlas_description/meshes_v3).
Its `package.xml` declares Apache 2.0 and identifies John Hsu of the Open Source
Robotics Foundation (OSRF) as author and maintainer. Atlas is a Boston Dynamics
robot. DART lowercased the `.DAE` suffixes and renamed the arm, shoulder and
lower-leg meshes without changing their contents. The Apache-2.0 text is in
[LICENSE](LICENSE); attribution is also in the root [NOTICE](../../../NOTICE).

DART relocated mesh references in the descriptions, made no-head variants,
converted URDF to SDF, and added and later calibrated soft-foot variants.
The modified descriptions carry a DART change notice. `ground.urdf` is DART's
sample ground, rather than an upstream Atlas description.

The seven adjacent `../materials/textures/*.jpg` files were imported in
[PR #985](https://github.com/dartsim/dart/pull/985). The v3 textures come from
the pinned OSRF package above; the three `*_unplugged.jpg` files come from the
[Atlas v5 package](https://github.com/Hurisa/drcsim/tree/a2a606ae475e682df1d5214e54de2cbd4f9b016f/atlas_description),
which also declares Apache 2.0. `atlas_cage_and_camera_diffuse_flat.jpg`,
`drc_labels_1.jpg`, and `right_leg_diffuse_unplugged.jpg` match those sources.
`drc_extremities_diffuse.jpg` and `drc_torso_head_diffuse.jpg` were edited with
GIMP in November 2016. `extremities_diffuse_unplugged.jpg` and
`torso_diffuse_unplugged.jpg` also differ from the pinned upstream files.
The root NOTICE records these texture modifications.
