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

#include <dart/collision/dart/Types.hpp>
#include <dart/collision/dart/narrow_phase/CapsuleCapsule.hpp>
#include <dart/collision/dart/shapes/Shape.hpp>

#include <gtest/gtest.h>

#include <stdexcept>
#include <vector>

using namespace dart::collision::native;

namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;

Eigen::Isometry3d translated(double x, double y, double z)
{
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = Eigen::Vector3d(x, y, z);
  return tf;
}

Eigen::Isometry3d rotatedTranslated(double x, double y, double z)
{
  Eigen::Isometry3d tf = translated(x, y, z);
  tf.linear() = Eigen::AngleAxisd(0.5 * kPi, Eigen::Vector3d::UnitY())
                    .toRotationMatrix();
  return tf;
}

} // namespace

TEST(CapsuleCapsule, SeparatedReturnsFalse)
{
  const CapsuleShape capsule1(0.5, 2.0);
  const CapsuleShape capsule2(0.5, 2.0);

  CollisionResult result;
  EXPECT_FALSE(collideCapsules(
      capsule1,
      Eigen::Isometry3d::Identity(),
      capsule2,
      translated(3.0, 0.0, 0.0),
      result));
  EXPECT_EQ(0u, result.numContacts());
}

TEST(CapsuleCapsule, ParallelOverlapAddsContact)
{
  const CapsuleShape capsule1(0.5, 2.0);
  const CapsuleShape capsule2(0.5, 2.0);

  CollisionResult result;
  ASSERT_TRUE(collideCapsules(
      capsule1,
      Eigen::Isometry3d::Identity(),
      capsule2,
      translated(0.75, 0.0, 0.0),
      result));

  ASSERT_EQ(1u, result.numContacts());
  EXPECT_TRUE(
      result.getContact(0).normal.isApprox(-Eigen::Vector3d::UnitX(), 1e-12));
  EXPECT_NEAR(0.25, result.getContact(0).depth, 1e-12);
}

TEST(CapsuleCapsule, ParallelYOffsetUsesYAxisNormal)
{
  const CapsuleShape capsule1(0.5, 2.0);
  const CapsuleShape capsule2(0.5, 2.0);

  CollisionResult result;
  ASSERT_TRUE(collideCapsules(
      capsule1,
      Eigen::Isometry3d::Identity(),
      capsule2,
      translated(0.0, 0.75, 0.0),
      result));

  ASSERT_EQ(1u, result.numContacts());
  EXPECT_TRUE(
      result.getContact(0).normal.isApprox(-Eigen::Vector3d::UnitY(), 1e-12));
  EXPECT_NEAR(0.25, result.getContact(0).depth, 1e-12);
}

TEST(CapsuleCapsule, AxialEndcapOverlapAddsContact)
{
  const CapsuleShape capsule1(0.5, 2.0);
  const CapsuleShape capsule2(0.5, 2.0);

  CollisionResult result;
  ASSERT_TRUE(collideCapsules(
      capsule1,
      Eigen::Isometry3d::Identity(),
      capsule2,
      translated(0.0, 0.0, 2.25),
      result));

  ASSERT_EQ(1u, result.numContacts());
  EXPECT_TRUE(
      result.getContact(0).normal.isApprox(-Eigen::Vector3d::UnitZ(), 1e-12));
  EXPECT_NEAR(0.75, result.getContact(0).depth, 1e-12);
}

TEST(CapsuleCapsule, CoincidentAxesUseDeterministicFallbackNormal)
{
  const CapsuleShape capsule1(0.5, 2.0);
  const CapsuleShape capsule2(0.5, 2.0);

  CollisionResult result;
  ASSERT_TRUE(collideCapsules(
      capsule1,
      Eigen::Isometry3d::Identity(),
      capsule2,
      Eigen::Isometry3d::Identity(),
      result));

  ASSERT_EQ(1u, result.numContacts());
  EXPECT_TRUE(
      result.getContact(0).normal.isApprox(Eigen::Vector3d::UnitZ(), 1e-12));
  EXPECT_NEAR(1.0, result.getContact(0).depth, 1e-12);
}

TEST(CapsuleCapsule, RotatedOverlapAddsContact)
{
  const CapsuleShape capsule1(0.5, 2.0);
  const CapsuleShape capsule2(0.5, 2.0);

  CollisionResult result;
  ASSERT_TRUE(collideCapsules(
      capsule1,
      Eigen::Isometry3d::Identity(),
      capsule2,
      rotatedTranslated(0.75, 0.0, 0.0),
      result));

  ASSERT_EQ(1u, result.numContacts());
  EXPECT_GT(result.getContact(0).depth, 0.0);
}

TEST(CapsuleCapsule, RotatedSeparatedReturnsFalse)
{
  const CapsuleShape capsule1(0.5, 2.0);
  const CapsuleShape capsule2(0.5, 2.0);

  CollisionResult result;
  EXPECT_FALSE(collideCapsules(
      capsule1,
      Eigen::Isometry3d::Identity(),
      capsule2,
      rotatedTranslated(3.0, 0.0, 0.0),
      result));
  EXPECT_EQ(0u, result.numContacts());
}

TEST(CapsuleCapsule, BinaryCheckDoesNotAddContacts)
{
  const CapsuleShape capsule1(0.5, 2.0);
  const CapsuleShape capsule2(0.5, 2.0);

  CollisionResult result;
  EXPECT_TRUE(collideCapsules(
      capsule1,
      Eigen::Isometry3d::Identity(),
      capsule2,
      translated(0.75, 0.0, 0.0),
      result,
      CollisionOption::binaryCheck()));
  EXPECT_EQ(0u, result.numContacts());
}

TEST(CapsuleCapsule, NonContactOptionReportsOverlapOnly)
{
  const CapsuleShape capsule1(0.5, 2.0);
  const CapsuleShape capsule2(0.5, 2.0);

  CollisionOption option;
  option.enableContact = false;
  option.maxNumContacts = 8;

  CollisionResult result;
  EXPECT_TRUE(collideCapsules(
      capsule1,
      Eigen::Isometry3d::Identity(),
      capsule2,
      translated(0.75, 0.0, 0.0),
      result,
      option));
  EXPECT_EQ(0u, result.numContacts());

  EXPECT_FALSE(collideCapsules(
      capsule1,
      Eigen::Isometry3d::Identity(),
      capsule2,
      translated(3.0, 0.0, 0.0),
      result,
      option));
  EXPECT_EQ(0u, result.numContacts());
}

TEST(CapsuleCapsule, ZeroContactLimitReturnsFalse)
{
  const CapsuleShape capsule1(0.5, 2.0);
  const CapsuleShape capsule2(0.5, 2.0);

  CollisionOption option;
  option.enableContact = false;
  option.maxNumContacts = 0;

  CollisionResult result;
  EXPECT_FALSE(collideCapsules(
      capsule1,
      Eigen::Isometry3d::Identity(),
      capsule2,
      translated(0.75, 0.0, 0.0),
      result,
      option));
  EXPECT_EQ(0u, result.numContacts());
}

TEST(CapsuleCapsuleBatch, CollidesEachPair)
{
  const CapsuleShape capsule1(0.5, 2.0);
  const CapsuleShape capsule2(0.5, 2.0);
  const std::vector<CapsulePair> pairs{
      {&capsule1,
       &capsule2,
       Eigen::Isometry3d::Identity(),
       translated(0.75, 0.0, 0.0)},
      {&capsule1,
       &capsule2,
       Eigen::Isometry3d::Identity(),
       translated(3.0, 0.0, 0.0)}};
  std::vector<CollisionResult> results(2);

  collideCapsulesBatch(pairs, results);

  EXPECT_EQ(1u, results[0].numContacts());
  EXPECT_EQ(0u, results[1].numContacts());
}

TEST(CapsuleCapsuleBatch, RejectsMalformedInputs)
{
  const CapsuleShape capsule1(0.5, 2.0);
  const CapsuleShape capsule2(0.5, 2.0);
  const std::vector<CapsulePair> validPairs{
      {&capsule1,
       &capsule2,
       Eigen::Isometry3d::Identity(),
       translated(0.75, 0.0, 0.0)}};

  std::vector<CollisionResult> emptyResults;
  EXPECT_THROW(
      collideCapsulesBatch(validPairs, emptyResults), std::invalid_argument);

  std::vector<CollisionResult> results(1);
  const std::vector<CapsulePair> nullShapePairs{
      {nullptr,
       &capsule2,
       Eigen::Isometry3d::Identity(),
       translated(0.75, 0.0, 0.0)}};
  EXPECT_THROW(
      collideCapsulesBatch(nullShapePairs, results), std::invalid_argument);
}
