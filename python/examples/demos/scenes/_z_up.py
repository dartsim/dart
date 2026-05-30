"""Reorient legacy Y-up worlds to the canonical Z-up convention.

A few demo scenes load assets (``.skel`` / URDF) that were authored Y-up: their
ground slabs have a ``+Y`` normal, articulated chains hang along ``-Y``, and the
bundled skel files even embed ``<gravity>0 -9.81 0</gravity>``. The DART viewer
and every hand-built Python scene use Z-up (camera up and gravity along ``-Z``),
so those Y-up worlds render sideways — ground slabs stand as vertical walls and
chains lie flat instead of hanging.

:func:`reorient_to_z_up` converts such a world in place by rotating every
skeleton's root joint by ``RotX(+90deg)`` (mapping the legacy up axis ``+Y`` to
``+Z``) and setting gravity along ``-Z``. Because the geometry *and* gravity are
rotated by the same rigid transform, the motion is identical to the legacy Y-up
simulation expressed in a rotated frame: every joint's generalized coordinates
evolve exactly as before, so cross-language golden-set parity is preserved while
the scene displays Z-up.
"""

from __future__ import annotations

import numpy as np

# RotX(+90deg) as a homogeneous transform: maps +Y -> +Z, +Z -> -Y. Applied on
# the left of each root joint's transform-from-parent, it rotates the whole
# skeleton subtree (and its initial pose) rigidly about the world origin.
_ROT_X_90 = np.array(
    [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, -1.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]
)

_GRAVITY_Z = [0.0, 0.0, -9.81]


def reorient_to_z_up(world: "object") -> None:
    """Rotate a legacy Y-up ``world`` in place so it displays Z-up.

    Premultiplies every skeleton root joint's transform-from-parent by
    ``RotX(+90deg)`` and sets the world gravity to ``-Z``. Joint DOF values are
    unchanged, and since gravity is rotated with the geometry the simulated
    trajectory matches the legacy Y-up world in a rotated frame.
    """

    for si in range(world.get_num_skeletons()):
        skeleton = world.get_skeleton(si)
        # Iterate joints (not body nodes): a root joint is one whose parent body
        # node is the world frame (``None``). Probing joints rather than body
        # nodes also sidesteps a dartpy soft-body BodyNode accessor crash, so
        # this stays valid for the soft/mixed skel worlds too.
        for ji in range(skeleton.get_num_joints()):
            joint = skeleton.get_joint(ji)
            if joint.get_parent_body_node() is not None:
                continue  # only root joints connect to the world frame
            base = np.asarray(joint.get_transform_from_parent_body_node().matrix())
            joint.set_transform_from_parent_body_node(_ROT_X_90 @ base)

    world.set_gravity(_GRAVITY_Z)
