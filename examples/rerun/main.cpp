/*
 * Copyright (c) 2011-2024, The DART development contributors
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

#include <dart/utils/urdf/urdf.hpp>
#include <dart/utils/utils.hpp>

#include <dart/simulation/World.hpp>

#include <dart/dart.hpp>

#include <rerun.hpp>
#include <rerun/demo_utils.hpp>

using namespace dart;
using namespace dart::common;
using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::utils;
using namespace dart::math;
using namespace rerun::demo;

int main()
{
  auto shape
      = std::make_shared<dynamics::BoxShape>(Eigen::Vector3d(0.3, 0.3, 0.3));

  // Create a box-shaped rigid body
  auto skeleton = dynamics::Skeleton::create();
  auto jointAndBody
      = skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>();
  auto body = jointAndBody.second;
  body->createShapeNodeWith<
      dynamics::VisualAspect,
      dynamics::CollisionAspect,
      dynamics::DynamicsAspect>(shape);

  // Create a world and add the rigid body
  auto world = simulation::World::create();
  world->addSkeleton(skeleton);

  // Spawn Rerun viewer
  const auto rec = rerun::RecordingStream("rerun_example_cpp");
  rec.spawn().exit_on_failure();

  for (auto i = 0; i < 1000; ++i) {
    world->step();

    for (auto j = 0u; j < world->getNumSkeletons(); ++j) {
      auto skel = world->getSkeleton(j);

      for (auto k = 0u; k < skel->getNumBodyNodes(); ++k) {
        auto body = skel->getBodyNode(k);
        body->eachShapeNodeWith<dynamics::VisualAspect>(
            [&](const dynamics::ShapeNode* shapeNode) {
              const auto& visualAspect = shapeNode->getVisualAspect();
              shapeNode->getTransform();
              const auto& shape = shapeNode->getShape();
            });
      }
    }
  }
}
