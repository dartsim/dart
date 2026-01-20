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
#include <dart/collision/experimental/narrow_phase/mpr.hpp>

#include <ccd/ccd.h>

#include <gtest/gtest.h>

#include <cmath>

using namespace dart::collision::experimental;

namespace {

constexpr double kTol = 1e-6;

struct LibccdSphere
{
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  double radius = 1.0;
};

struct LibccdBox
{
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  Eigen::Vector3d halfExtents = Eigen::Vector3d::Ones();
};

void toCcdVec3(ccd_vec3_t* out, const Eigen::Vector3d& v)
{
  ccdVec3Set(out, v.x(), v.y(), v.z());
}

Eigen::Vector3d toEigen(const ccd_vec3_t& v)
{
  return Eigen::Vector3d(v.v[0], v.v[1], v.v[2]);
}

void supportSphere(const void* obj, const ccd_vec3_t* dir, ccd_vec3_t* vec)
{
  const auto* sphere = static_cast<const LibccdSphere*>(obj);
  Eigen::Vector3d d(dir->v[0], dir->v[1], dir->v[2]);
  if (d.squaredNorm() < 1e-12) {
    d = Eigen::Vector3d::UnitX();
  } else {
    d.normalize();
  }
  const Eigen::Vector3d point = sphere->center + sphere->radius * d;
  toCcdVec3(vec, point);
}

void supportBox(const void* obj, const ccd_vec3_t* dir, ccd_vec3_t* vec)
{
  const auto* box = static_cast<const LibccdBox*>(obj);
  Eigen::Vector3d point = box->center;
  point.x() += (dir->v[0] >= 0.0) ? box->halfExtents.x() : -box->halfExtents.x();
  point.y() += (dir->v[1] >= 0.0) ? box->halfExtents.y() : -box->halfExtents.y();
  point.z() += (dir->v[2] >= 0.0) ? box->halfExtents.z() : -box->halfExtents.z();
  toCcdVec3(vec, point);
}

void centerSphere(const void* obj, ccd_vec3_t* center)
{
  const auto* sphere = static_cast<const LibccdSphere*>(obj);
  toCcdVec3(center, sphere->center);
}

void centerBox(const void* obj, ccd_vec3_t* center)
{
  const auto* box = static_cast<const LibccdBox*>(obj);
  toCcdVec3(center, box->center);
}

SupportFunction makeSphereSupport(const Eigen::Vector3d& center, double radius)
{
  return [center, radius](const Eigen::Vector3d& dir) -> Eigen::Vector3d {
    if (dir.squaredNorm() < 1e-12) {
      return center + Eigen::Vector3d(radius, 0, 0);
    }
    return center + radius * dir.normalized();
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

void configureCcd(ccd_t& ccd, ccd_support_fn support1, ccd_support_fn support2,
    ccd_center_fn center1, ccd_center_fn center2)
{
  CCD_INIT(&ccd);
  ccd.support1 = support1;
  ccd.support2 = support2;
  ccd.center1 = center1;
  ccd.center2 = center2;
  ccd.max_iterations = 64;
  ccd.epa_tolerance = 1e-6;
  ccd.mpr_tolerance = 1e-6;
}

void expectVectorNear(
    const Eigen::Vector3d& actual,
    const Eigen::Vector3d& expected,
    double tol)
{
  EXPECT_NEAR(actual.x(), expected.x(), tol);
  EXPECT_NEAR(actual.y(), expected.y(), tol);
  EXPECT_NEAR(actual.z(), expected.z(), tol);
}

}  // namespace

TEST(GjkLibccd, SphereSphereSeparationDistance)
{
  const Eigen::Vector3d centerA(0.0, 0.0, 0.0);
  const Eigen::Vector3d centerB(3.0, 0.0, 0.0);
  const double radius = 1.0;

  auto supportA = makeSphereSupport(centerA, radius);
  auto supportB = makeSphereSupport(centerB, radius);

  const GjkResult result = Gjk::query(supportA, supportB, centerB - centerA);
  EXPECT_FALSE(result.intersecting);
  EXPECT_NEAR(result.distance, 1.0, kTol);

  expectVectorNear(result.closestPointA, Eigen::Vector3d(1.0, 0.0, 0.0), kTol);
  expectVectorNear(result.closestPointB, Eigen::Vector3d(2.0, 0.0, 0.0), kTol);

  const Eigen::Vector3d expectedAxis = (centerB - centerA).normalized();
  EXPECT_GT(result.separationAxis.dot(expectedAxis), 0.99);
}

TEST(GjkLibccd, SphereSphereEpaMatchesLibccd)
{
  LibccdSphere sphereA{Eigen::Vector3d(0.0, 0.0, 0.0), 1.0};
  LibccdSphere sphereB{Eigen::Vector3d(1.5, 0.0, 0.0), 1.0};

  auto supportA = makeSphereSupport(sphereA.center, sphereA.radius);
  auto supportB = makeSphereSupport(sphereB.center, sphereB.radius);

  GjkResult gjk = Gjk::query(supportA, supportB, sphereB.center - sphereA.center);
  ASSERT_TRUE(gjk.intersecting);

  EpaResult epa = Epa::penetration(supportA, supportB, gjk.simplex);
  ASSERT_TRUE(epa.success);
  const double expectedDepth = sphereA.radius + sphereB.radius
                               - (sphereB.center - sphereA.center).norm();
  EXPECT_NEAR(epa.depth, expectedDepth, 1e-4);
  EXPECT_GT(epa.normal.normalized().dot(Eigen::Vector3d::UnitX()), 0.99);

  ccd_t ccd;
  configureCcd(ccd, supportSphere, supportSphere, centerSphere, centerSphere);

  double depth = 0.0;
  ccd_vec3_t dir;
  ccd_vec3_t pos;
  const int ret = ccdGJKPenetration(&sphereA, &sphereB, &ccd, &depth, &dir, &pos);
  ASSERT_EQ(ret, 0);

  EXPECT_NEAR(epa.depth, depth, 1e-4);

  const Eigen::Vector3d libccdNormal = toEigen(dir).normalized();
  EXPECT_GT(epa.normal.normalized().dot(libccdNormal), 0.99);

  const Eigen::Vector3d epaPos = 0.5 * (epa.pointOnA + epa.pointOnB);
  expectVectorNear(epaPos, toEigen(pos), 1e-3);
}

TEST(GjkLibccd, SphereSphereMprMatchesLibccd)
{
  LibccdSphere sphereA{Eigen::Vector3d(0.0, 0.0, 0.0), 1.0};
  LibccdSphere sphereB{Eigen::Vector3d(1.5, 0.0, 0.0), 1.0};

  auto supportA = makeSphereSupport(sphereA.center, sphereA.radius);
  auto supportB = makeSphereSupport(sphereB.center, sphereB.radius);

  MprResult mpr = Mpr::penetration(
      supportA, supportB, sphereA.center, sphereB.center);
  ASSERT_TRUE(mpr.success);
  const double expectedDepth = sphereA.radius + sphereB.radius
                               - (sphereB.center - sphereA.center).norm();
  EXPECT_NEAR(mpr.depth, expectedDepth, 1e-4);
  EXPECT_GT(mpr.normal.normalized().dot(Eigen::Vector3d::UnitX()), 0.99);

  ccd_t ccd;
  configureCcd(ccd, supportSphere, supportSphere, centerSphere, centerSphere);

  double depth = 0.0;
  ccd_vec3_t dir;
  ccd_vec3_t pos;
  const int ret = ccdMPRPenetration(&sphereA, &sphereB, &ccd, &depth, &dir, &pos);
  ASSERT_EQ(ret, 0);

  EXPECT_NEAR(mpr.depth, depth, 1e-4);

  const Eigen::Vector3d libccdNormal = toEigen(dir).normalized();
  EXPECT_GT(mpr.normal.normalized().dot(libccdNormal), 0.99);

  expectVectorNear(mpr.position, toEigen(pos), 1e-3);
}

TEST(GjkLibccd, BoxBoxIntersectMatchesLibccd)
{
  LibccdBox boxA{Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(1.0, 1.0, 1.0)};
  LibccdBox boxB{Eigen::Vector3d(1.2, 0.5, 0.0), Eigen::Vector3d(1.0, 1.0, 1.0)};

  auto supportA = makeBoxSupport(boxA.center, boxA.halfExtents);
  auto supportB = makeBoxSupport(boxB.center, boxB.halfExtents);

  EXPECT_TRUE(Gjk::intersect(supportA, supportB, boxB.center - boxA.center));

  ccd_t ccd;
  configureCcd(ccd, supportBox, supportBox, centerBox, centerBox);
  EXPECT_EQ(ccdGJKIntersect(&boxA, &boxB, &ccd), 1);
}
