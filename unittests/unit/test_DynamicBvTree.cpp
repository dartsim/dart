/*
 * Copyright (c) 2015, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2015, Humanoid Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016, Personal Robotics Lab, Carnegie Mellon University
 * All rights reserved.
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

#include <dart/collision/dart/dart.hpp>

#include <gtest/gtest.h>

#include "TestHelpers.hpp"

using namespace dart;

//==============================================================================
TEST(DynamicBvTree, Basic)
{
  DynamicBvTree<Aabb> tree;
  EXPECT_TRUE(tree.isEmpty());
  EXPECT_EQ(tree.getNumPopulatedNodes(), 0u);
  EXPECT_EQ(tree.getNumLeafNodes(), 0u);
  EXPECT_EQ(tree.getNumObjects(), 0u);
  EXPECT_EQ(tree.getBvMargin(), 0.0);

  // Invalid margin
  tree.setBvMargin(-1.0);
  EXPECT_EQ(tree.getBvMargin(), 0.0);

  auto box0 = BoxShape::createShared(Eigen::Vector3d::Random().cwiseAbs());
  auto box1 = BoxShape::createShared(Eigen::Vector3d::Random().cwiseAbs());

  auto aabb0 = Aabb::Random();
  auto aabb1 = Aabb::Random();

  auto index0 = tree.addObject(aabb0, &box0);
  EXPECT_EQ(tree.getNumPopulatedNodes(), 1u);
  EXPECT_EQ(tree.getNumLeafNodes(), 1u);
  EXPECT_EQ(tree.getNumObjects(), 1u);

  auto index1 = tree.addObject(aabb1, &box1);
  EXPECT_EQ(tree.getNumPopulatedNodes(), 3u);
  EXPECT_EQ(tree.getNumLeafNodes(), 2u);
  EXPECT_EQ(tree.getNumObjects(), 2u);

  tree.removeObject(index0);
  EXPECT_EQ(tree.getNumPopulatedNodes(), 3u);
  EXPECT_EQ(tree.getNumLeafNodes(), 1u);
  EXPECT_EQ(tree.getNumObjects(), 1u);

  tree.removeObject(index1);
  EXPECT_EQ(tree.getNumPopulatedNodes(), 3u);
  EXPECT_EQ(tree.getNumLeafNodes(), 0u);
  EXPECT_EQ(tree.getNumObjects(), 0u);
}

//==============================================================================
TEST(DynamicBvTree, BoundingVolumeMargin)
{
  DynamicBvTree<Aabb> tree;
  EXPECT_TRUE(tree.isEmpty());
  EXPECT_EQ(tree.getNumPopulatedNodes(), 0u);
  EXPECT_EQ(tree.getNumLeafNodes(), 0u);
  EXPECT_EQ(tree.getNumObjects(), 0u);
  EXPECT_EQ(tree.getBvMargin(), 0.0);

  // Invalid margin
  tree.setBvMargin(-1.0);
  EXPECT_EQ(tree.getBvMargin(), 0.0);

  auto box0 = BoxShape::createShared(Eigen::Vector3d::Random().cwiseAbs());
  auto box1 = BoxShape::createShared(Eigen::Vector3d::Random().cwiseAbs());

  auto aabb0 = Aabb::Random();
  auto aabb1 = Aabb::Random();

  auto index0 = tree.addObject(aabb0, &box0);

  tree.setBvMargin(1.0);
  EXPECT_EQ(tree.getBvMargin(), 1.0);
  auto index1 = tree.addObject(aabb1, &box1);

  auto aabb0FromTree = tree.getBv(index0);
  auto aabb1FromTree = tree.getBv(index1);

  EXPECT_TRUE(aabb0.almostEquals(aabb0FromTree));
  EXPECT_FALSE(aabb1.almostEquals(aabb1FromTree));
}

//==============================================================================
TEST(DynamicBvTree, Balancing)
{
  DynamicBvTree<Aabb> tree;

  EXPECT_TRUE(tree.isEmpty());
  EXPECT_EQ(tree.getNumObjects(), 0u);

  auto box0 = BoxShape::createShared(Eigen::Vector3d::Random().cwiseAbs());
  auto box1 = BoxShape::createShared(Eigen::Vector3d::Random().cwiseAbs());

  auto aabb0 = Aabb::Random();
  auto aabb1 = Aabb::Random();

  tree.addObject(aabb0, &box0);
  tree.addObject(aabb1, &box1);

  // tree.balanceBottomUp();
}

//==============================================================================
int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
