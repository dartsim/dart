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

#include <dart/collision/native/detail/span.hpp>
#include <dart/collision/native/narrow_phase/gjk.hpp>
#include <dart/collision/native/narrow_phase/mpr.hpp>
#include <dart/collision/native/shapes/shape.hpp>

#include <gtest/gtest.h>

#include <array>
#include <limits>
#include <utility>
#include <vector>

#include <cmath>

using namespace dart::collision::native;

namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;

} // namespace

SupportFunction makeSphereSupport(const Eigen::Vector3d& center, double radius)
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

SupportFunction makeOrientedBoxSupport(
    const Eigen::Isometry3d& transform, const Eigen::Vector3d& halfExtents)
{
  return [transform, halfExtents](const Eigen::Vector3d& dir) {
    const Eigen::Vector3d localDir = transform.linear().transpose() * dir;
    Eigen::Vector3d localSupport;
    localSupport.x()
        = (localDir.x() >= 0.0) ? halfExtents.x() : -halfExtents.x();
    localSupport.y()
        = (localDir.y() >= 0.0) ? halfExtents.y() : -halfExtents.y();
    localSupport.z()
        = (localDir.z() >= 0.0) ? halfExtents.z() : -halfExtents.z();
    return transform * localSupport;
  };
}

Eigen::Isometry3d makeBoxSignedDistanceFrame(
    const Eigen::Vector3d& translation = Eigen::Vector3d::Zero(),
    double angle = 0.0,
    const Eigen::Vector3d& axis = Eigen::Vector3d::UnitZ())
{
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = translation;
  if (angle != 0.0) {
    tf.linear()
        = Eigen::AngleAxisd(angle, axis.normalized()).toRotationMatrix();
  }
  return tf;
}

void expectRotatedBoxEdgeFaceSignedDistance(
    const char* name,
    const Eigen::Isometry3d& worldFromFrame,
    const Eigen::Vector3d& secondBoxTranslation,
    double expectedSignedDistance)
{
  SCOPED_TRACE(name);
  const Eigen::Vector3d halfExtents1(0.5, 0.5, 0.5);
  const Eigen::Vector3d halfExtents2(0.3, 0.4, 0.5);

  Eigen::Isometry3d frameFromBox1 = Eigen::Isometry3d::Identity();
  frameFromBox1.translation() = Eigen::Vector3d(0.0, 0.0, 0.5);

  Eigen::Isometry3d frameFromBox2 = Eigen::Isometry3d::Identity();
  frameFromBox2.linear() << 0.6, -0.8, 0.0, 0.8, 0.6, 0.0, 0.0, 0.0, 1.0;
  frameFromBox2.translation() = secondBoxTranslation;

  const Eigen::Isometry3d worldFromBox1 = worldFromFrame * frameFromBox1;
  const Eigen::Isometry3d worldFromBox2 = worldFromFrame * frameFromBox2;

  auto supportA = makeOrientedBoxSupport(worldFromBox1, halfExtents1);
  auto supportB = makeOrientedBoxSupport(worldFromBox2, halfExtents2);
  const Eigen::Vector3d initialDirection
      = worldFromBox2.translation() - worldFromBox1.translation();

  const GjkResult gjk = Gjk::query(supportA, supportB, initialDirection);
  if (expectedSignedDistance > 0.0) {
    ASSERT_FALSE(gjk.intersecting);
    EXPECT_NEAR(gjk.distance, expectedSignedDistance, 1e-6);
    EXPECT_TRUE(gjk.closestPointA.allFinite());
    EXPECT_TRUE(gjk.closestPointB.allFinite());
    return;
  }

  if (expectedSignedDistance == 0.0) {
    if (!gjk.intersecting) {
      EXPECT_NEAR(gjk.distance, 0.0, 1e-6);
      EXPECT_TRUE(gjk.closestPointA.allFinite());
      EXPECT_TRUE(gjk.closestPointB.allFinite());
    }
    return;
  }

  ASSERT_TRUE(gjk.intersecting);
  const MprResult mpr = Mpr::penetration(
      supportA,
      supportB,
      worldFromBox1.translation(),
      worldFromBox2.translation());
  ASSERT_TRUE(mpr.success);
  EXPECT_NEAR(mpr.depth, -expectedSignedDistance, 1e-5);
  EXPECT_TRUE(mpr.pointOnA.allFinite());
  EXPECT_TRUE(mpr.pointOnB.allFinite());
  EXPECT_TRUE(mpr.position.allFinite());
  EXPECT_GT(
      std::abs(mpr.normal.normalized().dot(
          worldFromFrame.linear() * Eigen::Vector3d::UnitX())),
      0.99);
}

SupportPoint makeSupportPoint(
    const SupportFunction& supportA,
    const SupportFunction& supportB,
    const Eigen::Vector3d& direction)
{
  SupportPoint point;
  point.direction = direction.normalized();
  point.v1 = supportA(point.direction);
  point.v2 = supportB(-point.direction);
  point.v = point.v1 - point.v2;
  return point;
}

SupportFunction makeConvexSupport(const ConvexShape& convex)
{
  return [&convex](const Eigen::Vector3d& dir) {
    return convex.support(dir);
  };
}

SupportFunction makeTransformedConvexSupport(
    const ConvexShape& convex, const Eigen::Isometry3d& transform)
{
  return [&convex, transform](const Eigen::Vector3d& dir) {
    const Eigen::Vector3d localDir = transform.linear().transpose() * dir;
    return transform * convex.support(localDir);
  };
}

struct ScriptedSupportEntry
{
  Eigen::Vector3d direction;
  Eigen::Vector3d point;
};

SupportFunction makeScriptedSupport(
    std::vector<ScriptedSupportEntry> entries, double fallbackScale = 1000.0)
{
  return [entries = std::move(entries),
          fallbackScale](const Eigen::Vector3d& dir) -> Eigen::Vector3d {
    Eigen::Vector3d direction = dir;
    if (direction.squaredNorm() < 1e-20) {
      direction = Eigen::Vector3d::UnitX();
    } else {
      direction.normalize();
    }

    for (const auto& entry : entries) {
      Eigen::Vector3d key = entry.direction;
      if (key.squaredNorm() < 1e-20) {
        key = Eigen::Vector3d::UnitX();
      } else {
        key.normalize();
      }
      if (direction.isApprox(key, 1e-9)) {
        return entry.point;
      }
    }

    return -fallbackScale * direction;
  };
}

SupportFunction makeZeroSupport()
{
  return [](const Eigen::Vector3d&) {
    return Eigen::Vector3d::Zero();
  };
}

GjkSimplex makeWarmStartDirections(span<const Eigen::Vector3d> directions)
{
  GjkSimplex simplex;
  for (const auto& direction : directions) {
    SupportPoint point;
    point.direction = direction;
    point.v = direction;
    simplex.push(point);
  }
  return simplex;
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
  auto supportA
      = makeBoxSupport(Eigen::Vector3d::Zero(), Eigen::Vector3d::Ones());
  auto supportB
      = makeBoxSupport(Eigen::Vector3d(1.5, 0, 0), Eigen::Vector3d::Ones());

  EXPECT_TRUE(Gjk::intersect(supportA, supportB));
}

TEST(Gjk, BoxesSeparated)
{
  auto supportA
      = makeBoxSupport(Eigen::Vector3d::Zero(), Eigen::Vector3d::Ones());
  auto supportB
      = makeBoxSupport(Eigen::Vector3d(3.0, 0, 0), Eigen::Vector3d::Ones());

  EXPECT_FALSE(Gjk::intersect(supportA, supportB));
}

TEST(Gjk, SphereBoxIntersecting)
{
  auto supportA = makeSphereSupport(Eigen::Vector3d::Zero(), 1.0);
  auto supportB = makeBoxSupport(
      Eigen::Vector3d(1.4, 0, 0), Eigen::Vector3d(0.5, 0.5, 0.5));

  EXPECT_TRUE(Gjk::intersect(supportA, supportB));
}

TEST(Gjk, SphereBoxTouching)
{
  auto supportA = makeSphereSupport(Eigen::Vector3d::Zero(), 1.0);
  auto supportB = makeBoxSupport(
      Eigen::Vector3d(1.5, 0, 0), Eigen::Vector3d(0.5, 0.5, 0.5));

  EXPECT_FALSE(Gjk::intersect(supportA, supportB));
}

TEST(Gjk, SphereBoxSeparated)
{
  auto supportA = makeSphereSupport(Eigen::Vector3d::Zero(), 1.0);
  auto supportB = makeBoxSupport(
      Eigen::Vector3d(3.0, 0, 0), Eigen::Vector3d(0.5, 0.5, 0.5));

  EXPECT_FALSE(Gjk::intersect(supportA, supportB));
}

TEST(Gjk, ConvexConvexIntersecting)
{
  std::vector<Eigen::Vector3d> verticesA
      = {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};
  ConvexShape convexA(verticesA);

  std::vector<Eigen::Vector3d> verticesB
      = {{1.5, 0, 0},
         {0.5, 0, 0},
         {1, 0.5, 0},
         {1, -0.5, 0},
         {1, 0, 0.5},
         {1, 0, -0.5}};
  ConvexShape convexB(verticesB);

  auto supportA = makeConvexSupport(convexA);
  auto supportB = makeConvexSupport(convexB);

  EXPECT_TRUE(Gjk::intersect(supportA, supportB));
}

TEST(Gjk, ConvexConvexSeparated)
{
  std::vector<Eigen::Vector3d> verticesA
      = {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};
  ConvexShape convexA(verticesA);

  std::vector<Eigen::Vector3d> verticesB
      = {{3.5, 0, 0},
         {2.5, 0, 0},
         {3, 0.5, 0},
         {3, -0.5, 0},
         {3, 0, 0.5},
         {3, 0, -0.5}};
  ConvexShape convexB(verticesB);

  auto supportA = makeConvexSupport(convexA);
  auto supportB = makeConvexSupport(convexB);

  EXPECT_FALSE(Gjk::intersect(supportA, supportB));
}

TEST(Gjk, TransformedConvexSupportMatchesVertexSearch)
{
  const std::vector<Eigen::Vector3d> vertices{
      {-1, -1, -1},
      {1, -1, -1},
      {1, 1, -1},
      {-1, 1, -1},
      {-1, -1, 1},
      {1, -1, 1},
      {1, 1, 1},
      {-1, 1, 1}};
  const ConvexShape convex(vertices);

  std::array<Eigen::Isometry3d, 3> transforms;
  transforms[0] = Eigen::Isometry3d::Identity();
  transforms[1] = Eigen::Isometry3d::Identity();
  transforms[1].translation() = Eigen::Vector3d(-1.0, 2.0, -3.0);
  transforms[2] = Eigen::Isometry3d::Identity();
  transforms[2].linear()
      = Eigen::AngleAxisd(0.5, Eigen::Vector3d(1.0, -2.0, 3.0).normalized())
            .toRotationMatrix();

  const std::array<Eigen::Vector3d, 3> directions{{
      {-2.0, -2.0, -2.0},
      {-2.0, 3.0, -2.0},
      {7.0, 1.0, -1.0},
  }};

  for (const auto& transform : transforms) {
    const auto support = makeTransformedConvexSupport(convex, transform);
    for (const auto& direction : directions) {
      Eigen::Vector3d expected = Eigen::Vector3d::Zero();
      double expectedDot = -std::numeric_limits<double>::infinity();
      for (const auto& vertex : vertices) {
        const Eigen::Vector3d transformedVertex = transform * vertex;
        const double dot = transformedVertex.dot(direction);
        if (dot > expectedDot) {
          expectedDot = dot;
          expected = transformedVertex;
        }
      }

      EXPECT_TRUE(support(direction).isApprox(expected, 1e-12));
    }
  }
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
  auto supportA
      = makeBoxSupport(Eigen::Vector3d::Zero(), Eigen::Vector3d::Ones());
  auto supportB
      = makeBoxSupport(Eigen::Vector3d(3.0, 3.0, 3.0), Eigen::Vector3d::Ones());

  EXPECT_FALSE(Gjk::intersect(supportA, supportB));
}

TEST(Gjk, DiagonalIntersection)
{
  auto supportA
      = makeBoxSupport(Eigen::Vector3d::Zero(), Eigen::Vector3d::Ones());
  auto supportB
      = makeBoxSupport(Eigen::Vector3d(1.5, 1.5, 1.5), Eigen::Vector3d::Ones());

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

TEST(Gjk, WarmStartSimplexReuseMatchesColdQuery)
{
  auto supportA
      = makeBoxSupport(Eigen::Vector3d::Zero(), Eigen::Vector3d::Ones());
  auto supportB = makeBoxSupport(
      Eigen::Vector3d(3.0, 2.5, 1.5), Eigen::Vector3d(0.5, 0.75, 0.4));

  const GjkResult cold
      = Gjk::query(supportA, supportB, Eigen::Vector3d(1.0, 1.0, 1.0));
  ASSERT_FALSE(cold.intersecting);
  ASSERT_GT(cold.simplex.size, 0);

  auto movedSupportB = makeBoxSupport(
      Eigen::Vector3d(3.05, 2.48, 1.52), Eigen::Vector3d(0.5, 0.75, 0.4));

  const GjkResult movedCold
      = Gjk::query(supportA, movedSupportB, Eigen::Vector3d(1.0, 1.0, 1.0));
  const GjkResult movedWarm = Gjk::query(
      supportA, movedSupportB, cold.simplex, Eigen::Vector3d(1.0, 1.0, 1.0));

  EXPECT_FALSE(movedWarm.intersecting);
  EXPECT_EQ(movedCold.intersecting, movedWarm.intersecting);
  EXPECT_NEAR(movedCold.distance, movedWarm.distance, 1e-9);
  EXPECT_TRUE(movedWarm.closestPointA.isApprox(movedCold.closestPointA, 1e-9));
  EXPECT_TRUE(movedWarm.closestPointB.isApprox(movedCold.closestPointB, 1e-9));
  EXPECT_GT(movedWarm.simplex.size, 0);
  for (int i = 0; i < movedWarm.simplex.size; ++i) {
    EXPECT_TRUE(movedWarm.simplex.points[i].direction.allFinite());
    EXPECT_GT(movedWarm.simplex.points[i].direction.squaredNorm(), 0.0);
  }
}

TEST(Gjk, WarmStartEmptySimplexFallsBack)
{
  auto supportA = makeSphereSupport(Eigen::Vector3d::Zero(), 1.0);
  auto supportB = makeSphereSupport(Eigen::Vector3d(3.0, 0.0, 0.0), 1.0);

  GjkSimplex emptySimplex;
  const GjkResult result
      = Gjk::query(supportA, supportB, emptySimplex, Eigen::Vector3d::Zero());

  EXPECT_FALSE(result.intersecting);
  EXPECT_NEAR(result.distance, 1.0, 1e-6);
}

TEST(Gjk, WarmStartSimplexReductionCasesRemainFinite)
{
  auto supportA
      = makeBoxSupport(Eigen::Vector3d::Zero(), Eigen::Vector3d::Ones());
  auto supportB = makeBoxSupport(
      Eigen::Vector3d(3.0, 2.0, 1.0), Eigen::Vector3d(0.5, 0.4, 0.3));

  const std::array<Eigen::Vector3d, 4> directions{{
      Eigen::Vector3d::UnitX(),
      Eigen::Vector3d::UnitY(),
      Eigen::Vector3d::UnitZ(),
      Eigen::Vector3d(-1.0, -1.0, -1.0),
  }};

  for (int simplexSize = 1; simplexSize <= 4; ++simplexSize) {
    GjkSimplex simplex;
    for (int i = 0; i < simplexSize; ++i) {
      simplex.push(makeSupportPoint(supportA, supportB, directions[i]));
    }
    simplex.points[0].direction = Eigen::Vector3d::Zero();

    const GjkResult result
        = Gjk::query(supportA, supportB, simplex, Eigen::Vector3d::Zero());
    EXPECT_FALSE(result.intersecting);
    EXPECT_TRUE(std::isfinite(result.distance));
    EXPECT_TRUE(result.closestPointA.allFinite());
    EXPECT_TRUE(result.closestPointB.allFinite());
    EXPECT_TRUE(result.separationAxis.allFinite());
  }
}

TEST(Gjk, ScriptedLineSimplexReductionsRemainFinite)
{
  const auto supportB = makeZeroSupport();
  const std::array<Eigen::Vector3d, 2> directions{
      Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitY()};

  const std::array<std::array<Eigen::Vector3d, 2>, 3> lineCases{{
      {Eigen::Vector3d(2.0, 0.0, 0.0), Eigen::Vector3d(2.0, 0.0, 0.0)},
      {Eigen::Vector3d(1.0, 0.0, 0.0), Eigen::Vector3d(2.0, 0.0, 0.0)},
      {Eigen::Vector3d(1.0, -1.0, 0.0), Eigen::Vector3d(1.0, 1.0, 0.0)},
  }};

  for (const auto& lineCase : lineCases) {
    auto supportA = makeScriptedSupport({
        {directions[0], lineCase[0]},
        {directions[1], lineCase[1]},
    });
    const GjkResult result = Gjk::query(
        supportA,
        supportB,
        makeWarmStartDirections(directions),
        Eigen::Vector3d::Zero());

    EXPECT_FALSE(result.intersecting);
    EXPECT_TRUE(std::isfinite(result.distance));
    EXPECT_TRUE(result.closestPointA.allFinite());
    EXPECT_TRUE(result.closestPointB.allFinite());
    EXPECT_TRUE(result.separationAxis.allFinite());
  }
}

TEST(Gjk, ScriptedTriangleSimplexReductionsRemainFinite)
{
  const auto supportB = makeZeroSupport();
  const std::array<Eigen::Vector3d, 3> directions{
      Eigen::Vector3d::UnitX(),
      Eigen::Vector3d::UnitY(),
      Eigen::Vector3d::UnitZ()};

  const std::array<std::array<Eigen::Vector3d, 3>, 3> triangleCases{{
      {Eigen::Vector3d(1.0, 0.0, 0.0),
       Eigen::Vector3d(2.0, -1.0, 0.0),
       Eigen::Vector3d(2.0, 1.0, 0.0)},
      {Eigen::Vector3d(1.0, -1.0, 0.0),
       Eigen::Vector3d(1.0, 1.0, 0.0),
       Eigen::Vector3d(3.0, 0.0, 0.0)},
      {Eigen::Vector3d(
           1.339155159085458, -1.8612700724644577, 3.0873592851850828),
       Eigen::Vector3d(
           0.96698692094859917, -0.24585782282486157, 2.7090516856147455),
       Eigen::Vector3d(
           -0.15173670634136016, 4.6100102129124449, 1.5718733703952394)},
  }};

  for (const auto& triangleCase : triangleCases) {
    auto supportA = makeScriptedSupport({
        {directions[0], triangleCase[0]},
        {directions[1], triangleCase[1]},
        {directions[2], triangleCase[2]},
    });
    const GjkResult result = Gjk::query(
        supportA,
        supportB,
        makeWarmStartDirections(directions),
        Eigen::Vector3d::Zero());

    EXPECT_FALSE(result.intersecting);
    EXPECT_TRUE(std::isfinite(result.distance));
    EXPECT_TRUE(result.closestPointA.allFinite());
    EXPECT_TRUE(result.closestPointB.allFinite());
    EXPECT_TRUE(result.separationAxis.allFinite());
  }
}

TEST(Gjk, ScriptedTetrahedronContainingOriginReportsIntersection)
{
  const auto supportA = makeScriptedSupport({
      {Eigen::Vector3d::UnitX(), Eigen::Vector3d(1.0, 0.0, 0.0)},
      {Eigen::Vector3d::UnitY(), Eigen::Vector3d(0.0, 1.0, 0.0)},
      {Eigen::Vector3d::UnitZ(), Eigen::Vector3d(0.0, 0.0, 1.0)},
      {Eigen::Vector3d(-1.0, -1.0, -1.0), Eigen::Vector3d(-1.0, -1.0, -1.0)},
  });
  const auto supportB = makeZeroSupport();
  const std::array<Eigen::Vector3d, 4> directions{
      Eigen::Vector3d::UnitX(),
      Eigen::Vector3d::UnitY(),
      Eigen::Vector3d::UnitZ(),
      Eigen::Vector3d(-1.0, -1.0, -1.0)};

  const GjkResult result = Gjk::query(
      supportA,
      supportB,
      makeWarmStartDirections(directions),
      Eigen::Vector3d::Zero());

  EXPECT_TRUE(result.intersecting);
  EXPECT_EQ(result.simplex.size, 4);
}

TEST(GjkSignedDistance, RotatedBoxEdgeFaceCases)
{
  const std::array<Eigen::Isometry3d, 4> frames{{
      Eigen::Isometry3d::Identity(),
      makeBoxSignedDistanceFrame(Eigen::Vector3d(0.0, 0.0, 1.0)),
      makeBoxSignedDistanceFrame(Eigen::Vector3d(0.0, 1.0, 0.0)),
      makeBoxSignedDistanceFrame(
          Eigen::Vector3d(0.1, 0.2, 0.3), kPi * 0.1, Eigen::Vector3d::UnitZ()),
  }};

  for (const auto& frame : frames) {
    expectRotatedBoxEdgeFaceSignedDistance(
        "rotated edge separated from face",
        frame,
        Eigen::Vector3d(-1.1, 0.0, 0.5),
        0.1);
    expectRotatedBoxEdgeFaceSignedDistance(
        "rotated edge touching face",
        frame,
        Eigen::Vector3d(-1.0, 0.1, 0.5),
        0.0);
    expectRotatedBoxEdgeFaceSignedDistance(
        "rotated edge penetrating face",
        frame,
        Eigen::Vector3d(-0.9, -0.05, 0.5),
        -0.1);
  }
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

TEST(Epa, RejectsIncompleteSimplex)
{
  auto supportA = makeSphereSupport(Eigen::Vector3d::Zero(), 1.0);
  auto supportB = makeSphereSupport(Eigen::Vector3d(0.5, 0.0, 0.0), 1.0);

  GjkSimplex simplex;
  simplex.push(makeSupportPoint(supportA, supportB, Eigen::Vector3d::UnitX()));
  simplex.push(makeSupportPoint(supportA, supportB, Eigen::Vector3d::UnitY()));

  const EpaResult epa = Epa::penetration(supportA, supportB, simplex);
  EXPECT_FALSE(epa.success);
  EXPECT_EQ(epa.depth, 0.0);
}

TEST(Epa, DegenerateInitialTetrahedronReturnsNoPenetration)
{
  const auto supportA = makeZeroSupport();
  const auto supportB = makeZeroSupport();

  GjkSimplex simplex;
  for (const Eigen::Vector3d& point :
       {Eigen::Vector3d(1.0, 0.0, 0.0),
        Eigen::Vector3d(2.0, 0.0, 0.0),
        Eigen::Vector3d(3.0, 0.0, 0.0),
        Eigen::Vector3d(4.0, 0.0, 0.0)}) {
    SupportPoint supportPoint;
    supportPoint.v = point;
    supportPoint.v1 = point;
    simplex.push(supportPoint);
  }

  const EpaResult epa = Epa::penetration(supportA, supportB, simplex);
  EXPECT_FALSE(epa.success);
  EXPECT_EQ(epa.depth, 0.0);
}

TEST(Epa, BoxBoxPenetrationDepthAnalytic)
{
  auto supportA
      = makeBoxSupport(Eigen::Vector3d::Zero(), Eigen::Vector3d::Ones());
  auto supportB
      = makeBoxSupport(Eigen::Vector3d(1.5, 0.0, 0.0), Eigen::Vector3d::Ones());

  GjkSimplex simplex;
  const std::array<Eigen::Vector3d, 4> directions
      = {Eigen::Vector3d::UnitX(),
         Eigen::Vector3d::UnitY(),
         Eigen::Vector3d::UnitZ(),
         Eigen::Vector3d(-1.0, -1.0, -1.0)};

  for (const auto& direction : directions) {
    simplex.push(makeSupportPoint(supportA, supportB, direction));
  }

  const EpaResult epa = Epa::penetration(supportA, supportB, simplex);
  ASSERT_TRUE(epa.success);

  EXPECT_NEAR(epa.depth, 0.5, 1e-4);
  EXPECT_GT(
      std::abs(epa.normal.normalized().dot(Eigen::Vector3d::UnitX())), 0.99);
  EXPECT_TRUE(epa.pointOnA.allFinite());
  EXPECT_TRUE(epa.pointOnB.allFinite());
}

TEST(Epa, BoxBoxSignedDistanceAnalytic)
{
  auto supportA
      = makeBoxSupport(Eigen::Vector3d::Zero(), Eigen::Vector3d::Ones());
  auto supportB
      = makeBoxSupport(Eigen::Vector3d(1.5, 0.0, 0.0), Eigen::Vector3d::Ones());

  GjkSimplex simplex;
  const std::array<Eigen::Vector3d, 4> directions
      = {Eigen::Vector3d::UnitX(),
         Eigen::Vector3d::UnitY(),
         Eigen::Vector3d::UnitZ(),
         Eigen::Vector3d(-1.0, -1.0, -1.0)};

  for (const auto& direction : directions) {
    simplex.push(makeSupportPoint(supportA, supportB, direction));
  }

  const EpaResult epa = Epa::penetration(supportA, supportB, simplex);
  ASSERT_TRUE(epa.success);

  EXPECT_NEAR(epa.signedDistance(), -0.5, 1e-4);
  EXPECT_EQ(epa.signedDistance(), -epa.depth);
}

TEST(Mpr, SphereSpherePenetrationDepthAnalytic)
{
  const Eigen::Vector3d centerA = Eigen::Vector3d::Zero();
  const Eigen::Vector3d centerB(1.5, 0.0, 0.0);
  constexpr double radius = 1.0;

  auto supportA = makeSphereSupport(centerA, radius);
  auto supportB = makeSphereSupport(centerB, radius);

  const MprResult mpr = Mpr::penetration(supportA, supportB, centerA, centerB);
  ASSERT_TRUE(mpr.success);

  const double expectedDepth = 2.0 * radius - (centerB - centerA).norm();
  EXPECT_NEAR(mpr.depth, expectedDepth, 1e-4);
  EXPECT_GT(mpr.normal.normalized().dot(Eigen::Vector3d::UnitX()), 0.99);
  EXPECT_TRUE(mpr.pointOnA.allFinite());
  EXPECT_TRUE(mpr.pointOnB.allFinite());
  EXPECT_TRUE(mpr.position.allFinite());
}

TEST(Mpr, ScriptedTouchAndSegmentPortalCases)
{
  const SupportFunction zero = makeZeroSupport();

  {
    const auto supportA = makeScriptedSupport(
        {{-Eigen::Vector3d::UnitX(), -1e-7 * Eigen::Vector3d::UnitX()}});

    const MprResult mpr = Mpr::penetration(
        supportA, zero, Eigen::Vector3d::UnitX(), Eigen::Vector3d::Zero());

    ASSERT_TRUE(mpr.success);
    EXPECT_NEAR(mpr.depth, 0.0, 1e-12);
    EXPECT_TRUE(mpr.pointOnA.allFinite());
    EXPECT_TRUE(mpr.pointOnB.allFinite());
    EXPECT_TRUE(mpr.position.allFinite());
  }

  {
    const auto supportA = makeScriptedSupport(
        {{-Eigen::Vector3d::UnitX(), -Eigen::Vector3d::UnitX()}});

    const MprResult mpr = Mpr::penetration(
        supportA, zero, Eigen::Vector3d::UnitX(), Eigen::Vector3d::Zero());

    ASSERT_TRUE(mpr.success);
    EXPECT_NEAR(mpr.depth, 1.0, 1e-12);
    EXPECT_GT(mpr.normal.dot(-Eigen::Vector3d::UnitX()), 0.99);
    EXPECT_TRUE(mpr.pointOnA.allFinite());
    EXPECT_TRUE(mpr.pointOnB.allFinite());
    EXPECT_TRUE(mpr.position.allFinite());
  }
}

TEST(Mpr, ReportsSeparatedAndConcentricCases)
{
  auto separatedA = makeSphereSupport(Eigen::Vector3d::Zero(), 1.0);
  auto separatedB = makeSphereSupport(Eigen::Vector3d(3.0, 0.0, 0.0), 1.0);
  EXPECT_FALSE(Mpr::intersect(
      separatedA,
      separatedB,
      Eigen::Vector3d::Zero(),
      Eigen::Vector3d(3.0, 0.0, 0.0)));
  EXPECT_FALSE(Mpr::penetration(
                   separatedA,
                   separatedB,
                   Eigen::Vector3d::Zero(),
                   Eigen::Vector3d(3.0, 0.0, 0.0))
                   .success);

  auto concentricA = makeSphereSupport(Eigen::Vector3d::Zero(), 1.0);
  auto concentricB = makeSphereSupport(Eigen::Vector3d::Zero(), 0.25);
  const MprResult contained = Mpr::penetration(
      concentricA,
      concentricB,
      Eigen::Vector3d::Zero(),
      Eigen::Vector3d::Zero());
  EXPECT_TRUE(contained.success);
  EXPECT_TRUE(contained.pointOnA.allFinite());
  EXPECT_TRUE(contained.pointOnB.allFinite());
  EXPECT_TRUE(contained.position.allFinite());
}
