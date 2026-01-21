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

#include <dart/dynamics/box_shape.hpp>
#include <dart/dynamics/free_joint.hpp>
#include <dart/dynamics/shape_node.hpp>
#include <dart/dynamics/skeleton.hpp>

#include <Eigen/Core>
#include <gtest/gtest.h>

using namespace dart::dynamics;

//==============================================================================
TEST(Issue896, SkeletonCloneDeepCopiesShapes)
{
  const auto skel = Skeleton::create("original");
  auto pair = skel->createJointAndBodyNodePair<FreeJoint>();
  auto* body = pair.second;

  const auto box = std::make_shared<BoxShape>(Eigen::Vector3d(1.0, 2.0, 3.0));
  body->createShapeNodeWith<VisualAspect, CollisionAspect>(box);

  const auto clone = skel->cloneSkeleton();
  ASSERT_TRUE(clone);

  auto* clonedBody = clone->getBodyNode(body->getName());
  ASSERT_NE(clonedBody, nullptr);
  // Use first shape node; cast checked below
  auto* clonedShapeNode = clonedBody->getShapeNodeWith<VisualAspect>(0);
  ASSERT_NE(clonedShapeNode, nullptr);

  const auto originalBox = std::dynamic_pointer_cast<BoxShape>(box);
  ASSERT_NE(originalBox, nullptr);
  const auto clonedBox
      = std::dynamic_pointer_cast<BoxShape>(clonedShapeNode->getShape());
  ASSERT_NE(clonedBox, nullptr);

  EXPECT_NE(originalBox.get(), clonedBox.get());

  const auto originalSize = originalBox->getSize();
  const Eigen::Vector3d clonedSize(0.25, 0.5, 0.75);

  clonedBox->setSize(clonedSize);
  EXPECT_EQ(originalBox->getSize(), originalSize);
  EXPECT_EQ(clonedBox->getSize(), clonedSize);
}
