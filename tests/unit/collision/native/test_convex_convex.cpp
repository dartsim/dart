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

#include <dart/collision/native/narrow_phase/convex_convex.hpp>
#include <dart/collision/native/shapes/shape.hpp>
#include <dart/collision/native/types.hpp>

#include <gtest/gtest.h>

#include <stdexcept>
#include <vector>

using namespace dart::collision::native;

namespace {

std::vector<Eigen::Vector3d> makeOctahedronVertices(double scale = 1.0)
{
  return {
      {scale, 0.0, 0.0},
      {-scale, 0.0, 0.0},
      {0.0, scale, 0.0},
      {0.0, -scale, 0.0},
      {0.0, 0.0, scale},
      {0.0, 0.0, -scale}};
}

Eigen::Isometry3d translated(double x, double y, double z)
{
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = Eigen::Vector3d(x, y, z);
  return tf;
}

} // namespace

TEST(ConvexConvex, SeparatedReturnsFalse)
{
  const ConvexShape convex1(makeOctahedronVertices());
  const ConvexShape convex2(makeOctahedronVertices());

  CollisionResult result;
  EXPECT_FALSE(collideConvexConvex(
      convex1,
      Eigen::Isometry3d::Identity(),
      convex2,
      translated(3.0, 0.0, 0.0),
      result,
      CollisionOption()));
  EXPECT_EQ(0u, result.numContacts());
}

TEST(ConvexConvex, OverlapAddsContact)
{
  const ConvexShape convex1(makeOctahedronVertices());
  const ConvexShape convex2(makeOctahedronVertices());

  CollisionResult result;
  EXPECT_TRUE(collideConvexConvex(
      convex1,
      Eigen::Isometry3d::Identity(),
      convex2,
      translated(1.5, 0.0, 0.0),
      result,
      CollisionOption()));
  ASSERT_EQ(1u, result.numContacts());
  EXPECT_GT(result.getContact(0).depth, 0.0);
}

TEST(ConvexConvex, ConvexSphereOverlap)
{
  const ConvexShape convex(makeOctahedronVertices());
  const SphereShape sphere(0.75);

  CollisionResult result;
  EXPECT_TRUE(collideConvexConvex(
      convex,
      Eigen::Isometry3d::Identity(),
      sphere,
      translated(1.25, 0.0, 0.0),
      result,
      CollisionOption()));
  EXPECT_EQ(1u, result.numContacts());
}

TEST(ConvexConvex, BinaryCheckDoesNotAddContacts)
{
  const ConvexShape convex1(makeOctahedronVertices());
  const ConvexShape convex2(makeOctahedronVertices());

  CollisionResult result;
  EXPECT_TRUE(collideConvexConvex(
      convex1,
      Eigen::Isometry3d::Identity(),
      convex2,
      translated(1.5, 0.0, 0.0),
      result,
      CollisionOption::binaryCheck()));
  EXPECT_EQ(0u, result.numContacts());
}

TEST(ConvexConvex, ZeroContactLimitReturnsFalse)
{
  const ConvexShape convex1(makeOctahedronVertices());
  const ConvexShape convex2(makeOctahedronVertices());

  CollisionOption option;
  option.enableContact = false;
  option.maxNumContacts = 0;

  CollisionResult result;
  EXPECT_FALSE(collideConvexConvex(
      convex1,
      Eigen::Isometry3d::Identity(),
      convex2,
      translated(1.5, 0.0, 0.0),
      result,
      option));
  EXPECT_EQ(0u, result.numContacts());
}

TEST(ConvexConvexBatch, RejectsMalformedInputs)
{
  const ConvexShape convex1(makeOctahedronVertices());
  const ConvexShape convex2(makeOctahedronVertices());
  const std::vector<ConvexPair> validPairs{
      {&convex1,
       &convex2,
       Eigen::Isometry3d::Identity(),
       translated(1.5, 0.0, 0.0)}};

  std::vector<CollisionResult> emptyResults;
  EXPECT_THROW(
      collideConvexConvexBatch(validPairs, emptyResults),
      std::invalid_argument);

  std::vector<CollisionResult> results(1);
  const std::vector<ConvexPair> nullShapePairs{
      {nullptr,
       &convex2,
       Eigen::Isometry3d::Identity(),
       translated(1.5, 0.0, 0.0)}};
  EXPECT_THROW(
      collideConvexConvexBatch(nullShapePairs, results), std::invalid_argument);
}
