/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * This file is provided under the following "BSD-style" License
 */

#include <dart/collision/experimental/narrow_phase/gjk.hpp>

#include <gtest/gtest.h>

using namespace dart::collision::experimental;

namespace {

constexpr double kTol = 1e-6;

SupportFunction makeSegmentSupport(
    const Eigen::Vector3d& a, const Eigen::Vector3d& b)
{
  return [a, b](const Eigen::Vector3d& dir) -> Eigen::Vector3d {
    const Eigen::Vector3d ab = b - a;
    if (dir.squaredNorm() < 1e-12) {
      return a;
    }
    return (dir.dot(ab) >= 0.0) ? b : a;
  };
}

}  // namespace

TEST(GjkDegenerate, LineSegmentsSeparated)
{
  const Eigen::Vector3d a0(-1.0, 0.0, 0.0);
  const Eigen::Vector3d a1(1.0, 0.0, 0.0);
  const Eigen::Vector3d b0(-1.0, 2.0, 0.0);
  const Eigen::Vector3d b1(1.0, 2.0, 0.0);

  auto supportA = makeSegmentSupport(a0, a1);
  auto supportB = makeSegmentSupport(b0, b1);

  const GjkResult result =
      Gjk::query(supportA, supportB, Eigen::Vector3d(0.0, 1.0, 0.0));

  EXPECT_FALSE(result.intersecting);
  EXPECT_NEAR(result.distance, 2.0, kTol);
  EXPECT_NEAR(result.closestPointA.y(), 0.0, kTol);
  EXPECT_NEAR(result.closestPointB.y(), 2.0, kTol);
  EXPECT_NEAR((result.closestPointB - result.closestPointA).norm(), 2.0, kTol);
  EXPECT_GT(result.separationAxis.dot(Eigen::Vector3d::UnitY()), 0.99);
}

TEST(GjkDegenerate, LineSegmentsIntersecting)
{
  const Eigen::Vector3d a0(-1.0, 0.0, 0.0);
  const Eigen::Vector3d a1(1.0, 0.0, 0.0);
  const Eigen::Vector3d b0(0.0, -1.0, 0.0);
  const Eigen::Vector3d b1(0.0, 1.0, 0.0);

  auto supportA = makeSegmentSupport(a0, a1);
  auto supportB = makeSegmentSupport(b0, b1);

  const GjkResult result =
      Gjk::query(supportA, supportB, Eigen::Vector3d(1.0, 0.0, 0.0));

  EXPECT_TRUE(result.intersecting);
  EXPECT_TRUE(Gjk::intersect(supportA, supportB, Eigen::Vector3d(1.0, 0.0, 0.0)));
}
