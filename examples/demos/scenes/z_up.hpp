/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * This file is provided under the "BSD-style" License.
 */

#ifndef DART_EXAMPLES_DEMOS_SCENES_Z_UP_HPP_
#define DART_EXAMPLES_DEMOS_SCENES_Z_UP_HPP_

#include <dart/simulation/world.hpp>

#include <dart/dynamics/joint.hpp>
#include <dart/dynamics/skeleton.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cstddef>

namespace dart::examples::demos {

/// Reorient a legacy Y-up world in place so it renders under the canonical
/// Z-up convention (camera up +Z, gravity along -Z).
///
/// Some demo assets (the bundled `.skel`/URDF models) are authored Y-up: their
/// ground slabs have a +Y normal, chains hang along -Y, and several skel files
/// even embed `<gravity>0 -9.81 0</gravity>`. The Filament viewer and the
/// hand-built scenes are Z-up, so those worlds render sideways (ground slabs
/// stand as walls, chains lie flat).
///
/// This premultiplies every skeleton root joint's transform-from-parent by
/// `RotX(+90deg)` (mapping +Y to +Z) and sets gravity along -Z. Because the
/// geometry and gravity are rotated by the same rigid transform, every joint's
/// generalized coordinates evolve exactly as in the legacy Y-up world, so the
/// motion (and cross-language golden-set parity) is preserved -- only the
/// display frame changes. It is therefore only valid for scenes whose logic
/// does not separately hardcode the up axis (e.g. balance controllers that read
/// a center-of-mass height); those must be reworked explicitly.
inline void reorientWorldToZUp(const dart::simulation::WorldPtr& world)
{
  if (world == nullptr) {
    return;
  }

  // RotX(+90deg): maps +Y -> +Z, +Z -> -Y. Applied on the left of each root
  // joint's transform-from-parent, it rotates the whole skeleton subtree (and
  // its initial pose) rigidly about the world origin.
  Eigen::Isometry3d rotation = Eigen::Isometry3d::Identity();
  rotation.linear() << 1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 1.0, 0.0;

  for (std::size_t si = 0; si < world->getNumSkeletons(); ++si) {
    const auto& skeleton = world->getSkeleton(si);
    // Iterate joints (not body nodes): a root joint is one whose parent body
    // node is the world frame (nullptr).
    for (std::size_t ji = 0; ji < skeleton->getNumJoints(); ++ji) {
      auto* joint = skeleton->getJoint(ji);
      if (joint->getParentBodyNode() != nullptr) {
        continue; // only root joints connect to the world frame
      }
      joint->setTransformFromParentBodyNode(
          rotation * joint->getTransformFromParentBodyNode());
    }
  }

  world->setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
}

} // namespace dart::examples::demos

#endif // DART_EXAMPLES_DEMOS_SCENES_Z_UP_HPP_
