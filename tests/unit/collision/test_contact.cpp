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

#include <dart/collision/contact.hpp>

#include <dart/dynamics/body_node.hpp>

#include <gtest/gtest.h>

#include <limits>

using dart::collision::Contact;

//==============================================================================
TEST(Contact, ZeroNormalRejectsNonFinite)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();

  EXPECT_TRUE(Contact::isZeroNormal(Eigen::Vector3d(nan, 0.0, 0.0)));
  EXPECT_TRUE(Contact::isZeroNormal(Eigen::Vector3d(0.0, inf, 0.0)));
  EXPECT_TRUE(Contact::isZeroNormal(Eigen::Vector3d(0.0, 0.0, -inf)));
}

//==============================================================================
TEST(Contact, ZeroNormalHonorsEpsilon)
{
  const double epsilon = Contact::getNormalEpsilon();

  EXPECT_TRUE(Contact::isZeroNormal(Eigen::Vector3d(0.0, 0.0, epsilon / 10.0)));
  EXPECT_FALSE(Contact::isZeroNormal(Eigen::Vector3d(0.0, 0.0, epsilon * 2.0)));
}

//==============================================================================
TEST(Contact, DefaultConstructorInitializesMembers)
{
  Contact contact;

  EXPECT_TRUE(contact.point.isZero());
  EXPECT_TRUE(contact.normal.isZero());
  EXPECT_TRUE(contact.force.isZero());
  EXPECT_EQ(contact.collisionObject1, nullptr);
  EXPECT_EQ(contact.collisionObject2, nullptr);
  EXPECT_EQ(contact.penetrationDepth, 0.0);
  EXPECT_EQ(contact.triID1, 0);
  EXPECT_EQ(contact.triID2, 0);
  EXPECT_EQ(contact.userData, nullptr);
}

//==============================================================================
TEST(Contact, PointNormalPenetrationAccessors)
{
  Contact contact;

  contact.point = Eigen::Vector3d(1.0, 2.0, 3.0);
  contact.normal = Eigen::Vector3d(0.0, 0.0, 1.0);
  contact.penetrationDepth = 0.05;

  EXPECT_EQ(contact.point, Eigen::Vector3d(1.0, 2.0, 3.0));
  EXPECT_EQ(contact.normal, Eigen::Vector3d(0.0, 0.0, 1.0));
  EXPECT_DOUBLE_EQ(contact.penetrationDepth, 0.05);
}

//==============================================================================
TEST(Contact, ForceAccessor)
{
  Contact contact;

  contact.force = Eigen::Vector3d(10.0, 20.0, 30.0);

  EXPECT_EQ(contact.force, Eigen::Vector3d(10.0, 20.0, 30.0));
}

//==============================================================================
TEST(Contact, IsNonZeroNormalInverseOfIsZeroNormal)
{
  const Eigen::Vector3d zeroNormal(0.0, 0.0, 0.0);
  const Eigen::Vector3d nonZeroNormal(0.0, 0.0, 1.0);

  EXPECT_TRUE(Contact::isZeroNormal(zeroNormal));
  EXPECT_FALSE(Contact::isNonZeroNormal(zeroNormal));

  EXPECT_FALSE(Contact::isZeroNormal(nonZeroNormal));
  EXPECT_TRUE(Contact::isNonZeroNormal(nonZeroNormal));
}

//==============================================================================
TEST(Contact, NormalEpsilonSquaredConsistency)
{
  const double epsilon = Contact::getNormalEpsilon();
  const double epsilonSquared = Contact::getNormalEpsilonSquared();

  EXPECT_DOUBLE_EQ(epsilonSquared, epsilon * epsilon);
}

//==============================================================================
TEST(Contact, ShapeFrameAccessorsReturnNullWithoutCollisionObjects)
{
  Contact contact;

  EXPECT_EQ(contact.getShapeFrame1(), nullptr);
  EXPECT_EQ(contact.getShapeFrame2(), nullptr);
  EXPECT_EQ(contact.getShapeNode1(), nullptr);
  EXPECT_EQ(contact.getShapeNode2(), nullptr);
  EXPECT_FALSE(contact.getBodyNodePtr1());
  EXPECT_FALSE(contact.getBodyNodePtr2());
}
