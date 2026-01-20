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

#include <dart/collision/experimental/narrow_phase/gjk.hpp>
#include <dart/collision/experimental/shapes/shape.hpp>

#include <gtest/gtest.h>

using namespace dart::collision::experimental;

SupportFunction makeSphereSupport(
    const Eigen::Vector3d& center, double radius)
{
  return [center, radius](const Eigen::Vector3d& dir) -> Eigen::Vector3d {
    double len = dir.norm();
    if (len < 1e-10) {
      return center + Eigen::Vector3d(radius, 0, 0);
    }
    return Eigen::Vector3d(center + radius * dir / len);
  };
}

SupportFunction makeBoxSupport(
    const Eigen::Vector3d& center, const Eigen::Vector3d& halfExtents)
{
  return [center, halfExtents](const Eigen::Vector3d& dir) {
    Eigen::Vector3d result = center;
    result.x() += (dir.x() >= 0) ? halfExtents.x() : -halfExtents.x();
    result.y() += (dir.y() >= 0) ? halfExtents.y() : -halfExtents.y();
    result.z() += (dir.z() >= 0) ? halfExtents.z() : -halfExtents.z();
    return result;
  };
}

SupportFunction makeConvexSupport(const ConvexShape& convex)
{
  return [&convex](const Eigen::Vector3d& dir) { return convex.support(dir); };
}

TEST(Gjk, SpheresIntersecting)
{
  auto supportA = makeSphereSupport(Eigen::Vector3d::Zero(), 1.0);
  auto supportB = makeSphereSupport(Eigen::Vector3d(1.5, 0, 0), 1.0);

  auto result = Gjk::query(supportA, supportB);
  EXPECT_TRUE(result.intersecting);
}

TEST(Gjk, SpheresSeparated)
{
  auto supportA = makeSphereSupport(Eigen::Vector3d::Zero(), 1.0);
  auto supportB = makeSphereSupport(Eigen::Vector3d(3.0, 0, 0), 1.0);

  auto result = Gjk::query(supportA, supportB);
  EXPECT_FALSE(result.intersecting);
}

TEST(Gjk, SpheresJustTouching)
{
  auto supportA = makeSphereSupport(Eigen::Vector3d::Zero(), 1.0);
  auto supportB = makeSphereSupport(Eigen::Vector3d(2.0, 0, 0), 1.0);

  auto result = Gjk::query(supportA, supportB);
  EXPECT_TRUE(result.intersecting);
}

TEST(Gjk, BoxesIntersecting)
{
  auto supportA = makeBoxSupport(Eigen::Vector3d::Zero(), Eigen::Vector3d::Ones());
  auto supportB =
      makeBoxSupport(Eigen::Vector3d(1.5, 0, 0), Eigen::Vector3d::Ones());

  EXPECT_TRUE(Gjk::intersect(supportA, supportB));
}

TEST(Gjk, BoxesSeparated)
{
  auto supportA = makeBoxSupport(Eigen::Vector3d::Zero(), Eigen::Vector3d::Ones());
  auto supportB =
      makeBoxSupport(Eigen::Vector3d(3.0, 0, 0), Eigen::Vector3d::Ones());

  EXPECT_FALSE(Gjk::intersect(supportA, supportB));
}

TEST(Gjk, SphereBoxIntersecting)
{
  auto supportA = makeSphereSupport(Eigen::Vector3d::Zero(), 1.0);
  auto supportB =
      makeBoxSupport(Eigen::Vector3d(1.4, 0, 0), Eigen::Vector3d(0.5, 0.5, 0.5));

  EXPECT_TRUE(Gjk::intersect(supportA, supportB));
}

TEST(Gjk, SphereBoxTouching)
{
  auto supportA = makeSphereSupport(Eigen::Vector3d::Zero(), 1.0);
  auto supportB =
      makeBoxSupport(Eigen::Vector3d(1.5, 0, 0), Eigen::Vector3d(0.5, 0.5, 0.5));

  EXPECT_FALSE(Gjk::intersect(supportA, supportB));
}

TEST(Gjk, SphereBoxSeparated)
{
  auto supportA = makeSphereSupport(Eigen::Vector3d::Zero(), 1.0);
  auto supportB =
      makeBoxSupport(Eigen::Vector3d(3.0, 0, 0), Eigen::Vector3d(0.5, 0.5, 0.5));

  EXPECT_FALSE(Gjk::intersect(supportA, supportB));
}

TEST(Gjk, ConvexConvexIntersecting)
{
  std::vector<Eigen::Vector3d> verticesA = {
      {1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};
  ConvexShape convexA(verticesA);

  std::vector<Eigen::Vector3d> verticesB = {
      {1.5, 0, 0}, {0.5, 0, 0}, {1, 0.5, 0}, {1, -0.5, 0}, {1, 0, 0.5}, {1, 0, -0.5}};
  ConvexShape convexB(verticesB);

  auto supportA = makeConvexSupport(convexA);
  auto supportB = makeConvexSupport(convexB);

  EXPECT_TRUE(Gjk::intersect(supportA, supportB));
}

TEST(Gjk, ConvexConvexSeparated)
{
  std::vector<Eigen::Vector3d> verticesA = {
      {1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};
  ConvexShape convexA(verticesA);

  std::vector<Eigen::Vector3d> verticesB = {
      {3.5, 0, 0}, {2.5, 0, 0}, {3, 0.5, 0}, {3, -0.5, 0}, {3, 0, 0.5}, {3, 0, -0.5}};
  ConvexShape convexB(verticesB);

  auto supportA = makeConvexSupport(convexA);
  auto supportB = makeConvexSupport(convexB);

  EXPECT_FALSE(Gjk::intersect(supportA, supportB));
}

TEST(Gjk, IdenticalShapes)
{
  auto supportA = makeSphereSupport(Eigen::Vector3d::Zero(), 1.0);
  auto supportB = makeSphereSupport(Eigen::Vector3d::Zero(), 1.0);

  EXPECT_TRUE(Gjk::intersect(supportA, supportB));
}

TEST(Gjk, OneContainsOther)
{
  auto supportA = makeSphereSupport(Eigen::Vector3d::Zero(), 2.0);
  auto supportB = makeSphereSupport(Eigen::Vector3d::Zero(), 0.5);

  EXPECT_TRUE(Gjk::intersect(supportA, supportB));
}

TEST(Gjk, DiagonalSeparation)
{
  auto supportA = makeBoxSupport(Eigen::Vector3d::Zero(), Eigen::Vector3d::Ones());
  auto supportB = makeBoxSupport(
      Eigen::Vector3d(3.0, 3.0, 3.0), Eigen::Vector3d::Ones());

  EXPECT_FALSE(Gjk::intersect(supportA, supportB));
}

TEST(Gjk, DiagonalIntersection)
{
  auto supportA = makeBoxSupport(Eigen::Vector3d::Zero(), Eigen::Vector3d::Ones());
  auto supportB = makeBoxSupport(
      Eigen::Vector3d(1.5, 1.5, 1.5), Eigen::Vector3d::Ones());

  EXPECT_TRUE(Gjk::intersect(supportA, supportB));
}

TEST(Gjk, SmallGap)
{
  auto supportA = makeSphereSupport(Eigen::Vector3d::Zero(), 1.0);
  auto supportB = makeSphereSupport(Eigen::Vector3d(2.001, 0, 0), 1.0);

  EXPECT_FALSE(Gjk::intersect(supportA, supportB));
}

TEST(Gjk, SmallOverlap)
{
  auto supportA = makeSphereSupport(Eigen::Vector3d::Zero(), 1.0);
  auto supportB = makeSphereSupport(Eigen::Vector3d(1.999, 0, 0), 1.0);

  EXPECT_TRUE(Gjk::intersect(supportA, supportB));
}

TEST(Epa, SphereSphereIntersecting)
{
  auto supportA = makeSphereSupport(Eigen::Vector3d::Zero(), 1.0);
  auto supportB = makeSphereSupport(Eigen::Vector3d(1.5, 0, 0), 1.0);

  GjkResult gjk = Gjk::query(supportA, supportB, Eigen::Vector3d(1.5, 0, 0));
  ASSERT_TRUE(gjk.intersecting);

  EpaResult epa = Epa::penetration(supportA, supportB, gjk.simplex);
  if (epa.success) {
    EXPECT_NEAR(epa.depth, 0.5, 1e-4);
  } else {
    EXPECT_LT(gjk.simplex.size, 4);
  }
}
