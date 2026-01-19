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

#include <dart/collision/experimental/aabb.hpp>

#include <gtest/gtest.h>

using namespace dart::collision::experimental;

TEST(Aabb, DefaultConstruction)
{
  Aabb aabb;

  EXPECT_EQ(aabb.min, Eigen::Vector3d::Zero());
  EXPECT_EQ(aabb.max, Eigen::Vector3d::Zero());
}

TEST(Aabb, Construction)
{
  Aabb aabb(Eigen::Vector3d(-1, -2, -3), Eigen::Vector3d(1, 2, 3));

  EXPECT_EQ(aabb.min, Eigen::Vector3d(-1, -2, -3));
  EXPECT_EQ(aabb.max, Eigen::Vector3d(1, 2, 3));
}

TEST(Aabb, Overlaps_True)
{
  Aabb a(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(2, 2, 2));
  Aabb b(Eigen::Vector3d(1, 1, 1), Eigen::Vector3d(3, 3, 3));

  EXPECT_TRUE(a.overlaps(b));
  EXPECT_TRUE(b.overlaps(a));
}

TEST(Aabb, Overlaps_Touching)
{
  Aabb a(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(1, 1, 1));
  Aabb b(Eigen::Vector3d(1, 0, 0), Eigen::Vector3d(2, 1, 1));

  EXPECT_TRUE(a.overlaps(b));
  EXPECT_TRUE(b.overlaps(a));
}

TEST(Aabb, Overlaps_Separated)
{
  Aabb a(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(1, 1, 1));
  Aabb b(Eigen::Vector3d(2, 0, 0), Eigen::Vector3d(3, 1, 1));

  EXPECT_FALSE(a.overlaps(b));
  EXPECT_FALSE(b.overlaps(a));
}

TEST(Aabb, Overlaps_SeparatedY)
{
  Aabb a(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(1, 1, 1));
  Aabb b(Eigen::Vector3d(0, 2, 0), Eigen::Vector3d(1, 3, 1));

  EXPECT_FALSE(a.overlaps(b));
}

TEST(Aabb, Overlaps_SeparatedZ)
{
  Aabb a(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(1, 1, 1));
  Aabb b(Eigen::Vector3d(0, 0, 2), Eigen::Vector3d(1, 1, 3));

  EXPECT_FALSE(a.overlaps(b));
}

TEST(Aabb, Overlaps_Contained)
{
  Aabb a(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(4, 4, 4));
  Aabb b(Eigen::Vector3d(1, 1, 1), Eigen::Vector3d(2, 2, 2));

  EXPECT_TRUE(a.overlaps(b));
  EXPECT_TRUE(b.overlaps(a));
}

TEST(Aabb, ContainsPoint_Inside)
{
  Aabb aabb(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(2, 2, 2));

  EXPECT_TRUE(aabb.contains(Eigen::Vector3d(1, 1, 1)));
}

TEST(Aabb, ContainsPoint_OnBoundary)
{
  Aabb aabb(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(2, 2, 2));

  EXPECT_TRUE(aabb.contains(Eigen::Vector3d(0, 0, 0)));
  EXPECT_TRUE(aabb.contains(Eigen::Vector3d(2, 2, 2)));
  EXPECT_TRUE(aabb.contains(Eigen::Vector3d(1, 0, 0)));
}

TEST(Aabb, ContainsPoint_Outside)
{
  Aabb aabb(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(2, 2, 2));

  EXPECT_FALSE(aabb.contains(Eigen::Vector3d(-1, 1, 1)));
  EXPECT_FALSE(aabb.contains(Eigen::Vector3d(3, 1, 1)));
  EXPECT_FALSE(aabb.contains(Eigen::Vector3d(1, -1, 1)));
  EXPECT_FALSE(aabb.contains(Eigen::Vector3d(1, 1, 3)));
}

TEST(Aabb, ContainsAabb_True)
{
  Aabb outer(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(4, 4, 4));
  Aabb inner(Eigen::Vector3d(1, 1, 1), Eigen::Vector3d(3, 3, 3));

  EXPECT_TRUE(outer.contains(inner));
}

TEST(Aabb, ContainsAabb_False)
{
  Aabb a(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(2, 2, 2));
  Aabb b(Eigen::Vector3d(1, 1, 1), Eigen::Vector3d(3, 3, 3));

  EXPECT_FALSE(a.contains(b));
  EXPECT_FALSE(b.contains(a));
}

TEST(Aabb, Center)
{
  Aabb aabb(Eigen::Vector3d(-1, -2, -3), Eigen::Vector3d(3, 4, 5));

  EXPECT_EQ(aabb.center(), Eigen::Vector3d(1, 1, 1));
}

TEST(Aabb, HalfExtents)
{
  Aabb aabb(Eigen::Vector3d(-1, -2, -3), Eigen::Vector3d(1, 2, 3));

  EXPECT_EQ(aabb.halfExtents(), Eigen::Vector3d(1, 2, 3));
}

TEST(Aabb, Extents)
{
  Aabb aabb(Eigen::Vector3d(-1, -2, -3), Eigen::Vector3d(1, 2, 3));

  EXPECT_EQ(aabb.extents(), Eigen::Vector3d(2, 4, 6));
}

TEST(Aabb, Volume)
{
  Aabb aabb(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(2, 3, 4));

  EXPECT_DOUBLE_EQ(aabb.volume(), 24.0);
}

TEST(Aabb, Merge)
{
  Aabb a(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(1, 1, 1));
  Aabb b(Eigen::Vector3d(2, 2, 2), Eigen::Vector3d(3, 3, 3));

  a.merge(b);

  EXPECT_EQ(a.min, Eigen::Vector3d(0, 0, 0));
  EXPECT_EQ(a.max, Eigen::Vector3d(3, 3, 3));
}

TEST(Aabb, Merge_Overlapping)
{
  Aabb a(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(2, 2, 2));
  Aabb b(Eigen::Vector3d(1, 1, 1), Eigen::Vector3d(3, 3, 3));

  a.merge(b);

  EXPECT_EQ(a.min, Eigen::Vector3d(0, 0, 0));
  EXPECT_EQ(a.max, Eigen::Vector3d(3, 3, 3));
}

TEST(Aabb, Expand)
{
  Aabb aabb(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(2, 2, 2));

  aabb.expand(0.5);

  EXPECT_EQ(aabb.min, Eigen::Vector3d(-0.5, -0.5, -0.5));
  EXPECT_EQ(aabb.max, Eigen::Vector3d(2.5, 2.5, 2.5));
}

TEST(Aabb, ForSphere)
{
  auto aabb = Aabb::forSphere(2.0);

  EXPECT_EQ(aabb.min, Eigen::Vector3d(-2, -2, -2));
  EXPECT_EQ(aabb.max, Eigen::Vector3d(2, 2, 2));
}

TEST(Aabb, ForBox)
{
  auto aabb = Aabb::forBox(Eigen::Vector3d(1, 2, 3));

  EXPECT_EQ(aabb.min, Eigen::Vector3d(-1, -2, -3));
  EXPECT_EQ(aabb.max, Eigen::Vector3d(1, 2, 3));
}

TEST(Aabb, Transformed_Identity)
{
  Aabb local(Eigen::Vector3d(-1, -1, -1), Eigen::Vector3d(1, 1, 1));
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();

  auto world = Aabb::transformed(local, transform);

  EXPECT_EQ(world.min, local.min);
  EXPECT_EQ(world.max, local.max);
}

TEST(Aabb, Transformed_Translation)
{
  Aabb local(Eigen::Vector3d(-1, -1, -1), Eigen::Vector3d(1, 1, 1));
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translate(Eigen::Vector3d(5, 0, 0));

  auto world = Aabb::transformed(local, transform);

  EXPECT_EQ(world.min, Eigen::Vector3d(4, -1, -1));
  EXPECT_EQ(world.max, Eigen::Vector3d(6, 1, 1));
}

TEST(Aabb, Transformed_Rotation90)
{
  Aabb local(Eigen::Vector3d(-1, -2, -3), Eigen::Vector3d(1, 2, 3));
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.rotate(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()));

  auto world = Aabb::transformed(local, transform);

  EXPECT_NEAR(world.min.x(), -2, 1e-10);
  EXPECT_NEAR(world.min.y(), -1, 1e-10);
  EXPECT_NEAR(world.min.z(), -3, 1e-10);
  EXPECT_NEAR(world.max.x(), 2, 1e-10);
  EXPECT_NEAR(world.max.y(), 1, 1e-10);
  EXPECT_NEAR(world.max.z(), 3, 1e-10);
}

TEST(Aabb, Transformed_Rotation45_Grows)
{
  Aabb local(Eigen::Vector3d(-1, -1, 0), Eigen::Vector3d(1, 1, 0));
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.rotate(Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitZ()));

  auto world = Aabb::transformed(local, transform);

  const double sqrt2 = std::sqrt(2.0);
  EXPECT_NEAR(world.min.x(), -sqrt2, 1e-10);
  EXPECT_NEAR(world.min.y(), -sqrt2, 1e-10);
  EXPECT_NEAR(world.max.x(), sqrt2, 1e-10);
  EXPECT_NEAR(world.max.y(), sqrt2, 1e-10);
}
