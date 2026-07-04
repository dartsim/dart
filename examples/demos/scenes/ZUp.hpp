/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef DART_EXAMPLES_DEMOS_SCENES_ZUP_HPP_
#define DART_EXAMPLES_DEMOS_SCENES_ZUP_HPP_

#include <dart/dart.hpp>

namespace dart_demos {

//==============================================================================
// Many bundled .skel worlds (chain.skel, cubes.skel, shapes.skel,
// ground.skel, ...) are authored Y-up (ground normal +Y; the .skel files
// embed <gravity>0 -9.81 0</gravity>), matching the convention their original
// osg examples used. dart-demos uses one Z-up convention across every scene
// (matching DemoScene::cameraHome eye/center/up throughout examples/demos),
// so such worlds are reoriented here instead: premultiply every root joint's
// transform-from-parent by RotX(+90deg) (+Y -> +Z), then set gravity along
// -Z. Because the geometry and gravity are rotated by the same rigid
// transform, every joint's generalized coordinates evolve exactly as in the
// original Y-up world -- only the display frame changes. Ported from DART 7's
// `reorientWorldToZUp` helper (examples/demos/scenes/z_up.hpp at
// 1a5469960c2703accb2762e03fe8a6bb1156dc08).
inline void reorientWorldToZUp(const dart::simulation::WorldPtr& world)
{
  Eigen::Isometry3d rotation = Eigen::Isometry3d::Identity();
  // clang-format off
  rotation.linear() << 1.0, 0.0,  0.0,
                       0.0, 0.0, -1.0,
                       0.0, 1.0,  0.0;
  // clang-format on

  for (std::size_t si = 0; si < world->getNumSkeletons(); ++si) {
    const auto& skeleton = world->getSkeleton(si);
    for (std::size_t ji = 0; ji < skeleton->getNumJoints(); ++ji) {
      auto* joint = skeleton->getJoint(ji);
      if (joint->getParentBodyNode() != nullptr)
        continue; // only root joints connect to the world frame
      joint->setTransformFromParentBodyNode(
          rotation * joint->getTransformFromParentBodyNode());
    }
  }

  world->setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
}

} // namespace dart_demos

#endif // DART_EXAMPLES_DEMOS_SCENES_ZUP_HPP_
