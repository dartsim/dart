/*
 * Copyright (c) 2011-2025, The DART development contributors
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

#include <dart/dynamics/BodyNode.hpp>
#include <dart/dynamics/EndEffector.hpp>
#include <dart/dynamics/FreeJoint.hpp>
#include <dart/dynamics/Skeleton.hpp>

#include <gtest/gtest.h>

#include <string>

using namespace dart::dynamics;

//==============================================================================
// Skeleton::getSupportIndices() names, for each support polygon vertex, the end
// effector whose support geometry produced it.
TEST(SupportPolygon, IndicesNameTheEndEffectorOfEachVertex)
{
  auto skeleton = Skeleton::create("biped");
  auto* body = skeleton->createJointAndBodyNodePair<FreeJoint>().second;

  const dart::math::SupportGeometry foot{
      Eigen::Vector3d(0.1, 0.1, 0.0),
      Eigen::Vector3d(-0.1, 0.1, 0.0),
      Eigen::Vector3d(-0.1, -0.1, 0.0),
      Eigen::Vector3d(0.1, -0.1, 0.0)};

  const auto addFoot = [&](const std::string& name, double x) {
    EndEffector* foot_ee = body->createEndEffector(name);
    foot_ee->setDefaultRelativeTransform(
        Eigen::Isometry3d(Eigen::Translation3d(x, 0.0, 0.0)), true);
    foot_ee->getSupport(true)->setGeometry(foot);
    foot_ee->getSupport()->setActive();
    return foot_ee;
  };
  const EndEffector* left = addFoot("left", -1.0);
  const EndEffector* right = addFoot("right", 1.0);

  const auto& polygon = skeleton->getSupportPolygon();
  const auto& indices = skeleton->getSupportIndices();
  ASSERT_GE(polygon.size(), 4u);
  ASSERT_EQ(polygon.size(), indices.size());

  // With gravity along -z, polygon coordinates are world x and y, so the left
  // foot owns every vertex with negative x.
  for (std::size_t i = 0; i < polygon.size(); ++i) {
    const EndEffector* owner = polygon[i].x() < 0.0 ? left : right;
    EXPECT_EQ(owner->getIndexInSkeleton(), indices[i]) << "vertex " << i;
  }
}
