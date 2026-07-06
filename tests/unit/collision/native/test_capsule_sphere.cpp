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

#include <dart/collision/native/narrow_phase/capsule_sphere.hpp>
#include <dart/collision/native/shapes/shape.hpp>
#include <dart/collision/native/types.hpp>

#include <gtest/gtest.h>

using namespace dart::collision::native;

namespace {

Eigen::Isometry3d translated(double x, double y, double z)
{
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = Eigen::Vector3d(x, y, z);
  return tf;
}

Eigen::Isometry3d rotatedCapsule()
{
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.linear()
      = Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitY()).toRotationMatrix();
  return tf;
}

} // namespace

TEST(CapsuleSphere, SeparatedReturnsFalse)
{
  const CapsuleShape capsule(0.5, 2.0);
  const SphereShape sphere(0.5);

  CollisionResult result;
  EXPECT_FALSE(collideCapsuleSphere(
      capsule,
      Eigen::Isometry3d::Identity(),
      sphere,
      translated(2.5, 0.0, 0.0),
      result));
  EXPECT_EQ(0u, result.numContacts());
}

TEST(CapsuleSphere, OverlappingVerticalCapsuleAddsContact)
{
  const CapsuleShape capsule(0.5, 2.0);
  const SphereShape sphere(0.5);

  CollisionResult result;
  ASSERT_TRUE(collideCapsuleSphere(
      capsule,
      Eigen::Isometry3d::Identity(),
      sphere,
      translated(0.75, 0.0, 0.0),
      result));

  ASSERT_EQ(1u, result.numContacts());
  const auto& contact = result.getContact(0);
  EXPECT_TRUE(contact.normal.isApprox(-Eigen::Vector3d::UnitX(), 1e-12));
  EXPECT_NEAR(0.25, contact.depth, 1e-12);
}

TEST(CapsuleSphere, BinaryCheckDoesNotAddContacts)
{
  const CapsuleShape capsule(0.5, 2.0);
  const SphereShape sphere(0.5);

  CollisionResult result;
  EXPECT_TRUE(collideCapsuleSphere(
      capsule,
      Eigen::Isometry3d::Identity(),
      sphere,
      translated(0.75, 0.0, 0.0),
      result,
      CollisionOption::binaryCheck()));
  EXPECT_EQ(0u, result.numContacts());
}

TEST(CapsuleSphere, RotatedBinaryCheckDoesNotAddContacts)
{
  const CapsuleShape capsule(0.5, 2.0);
  const SphereShape sphere(0.5);

  CollisionResult result;
  EXPECT_TRUE(collideCapsuleSphere(
      capsule,
      rotatedCapsule(),
      sphere,
      translated(0.0, 0.75, 0.0),
      result,
      CollisionOption::binaryCheck()));
  EXPECT_EQ(0u, result.numContacts());
}

TEST(CapsuleSphere, ZeroContactLimitReturnsFalse)
{
  const CapsuleShape capsule(0.5, 2.0);
  const SphereShape sphere(0.5);

  CollisionOption option;
  option.enableContact = false;
  option.maxNumContacts = 0;

  CollisionResult result;
  EXPECT_FALSE(collideCapsuleSphere(
      capsule,
      Eigen::Isometry3d::Identity(),
      sphere,
      translated(0.75, 0.0, 0.0),
      result,
      option));
  EXPECT_EQ(0u, result.numContacts());
}
