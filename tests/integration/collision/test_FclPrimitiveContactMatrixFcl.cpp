/*
 * Copyright (c) 2011-2025, The DART development contributors
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

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <fcl/fcl.h>
#include <gtest/gtest.h>

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include <cmath>

namespace {

enum class ShapeKind
{
  Box,
  Sphere,
  Cylinder,
  Ellipsoid,
  Cone,
  Plane
};

struct ShapeSpec
{
  ShapeKind kind;
  const char* name;
};

struct CaseSpec
{
  const char* name;
  double separationOffset;
  bool expectCollision;
  double yawRadians;
  Eigen::Vector3d direction;
  bool allowCone;
};

const Eigen::Vector3d kBoxSize(1.0, 0.8, 0.6);
constexpr double kSphereRadius = 0.5;
const Eigen::Vector3d kEllipsoidRadii(0.6, 0.4, 0.3);
constexpr double kCylinderRadius = 0.4;
constexpr double kCylinderHeight = 0.8;
constexpr double kConeRadius = 0.5;
constexpr double kConeHeight = 1.0;
const Eigen::Vector3d kPlaneNormal = Eigen::Vector3d::UnitZ();
constexpr double kPlaneOffset = 0.0;
constexpr double kYawQuarterTurn = 0.25 * 3.141592653589793;

const std::vector<ShapeSpec> kShapeSpecs = {
    {ShapeKind::Box, "box"},
    {ShapeKind::Sphere, "sphere"},
    {ShapeKind::Cylinder, "cylinder"},
    {ShapeKind::Ellipsoid, "ellipsoid"},
    {ShapeKind::Cone, "cone"},
    {ShapeKind::Plane, "plane"},
};

const std::vector<CaseSpec> kBaseCases = {
    {"separated", 0.2, false, 0.0, Eigen::Vector3d::UnitZ(), true},
    {"near_touch", -1e-4, true, 0.0, Eigen::Vector3d::UnitZ(), true},
    {"shallow_overlap", -0.02, true, 0.0, Eigen::Vector3d::UnitZ(), true},
    {"deep_overlap", -0.2, true, 0.0, Eigen::Vector3d::UnitZ(), true},
    {"rotated_shallow",
     -0.02,
     true,
     kYawQuarterTurn,
     Eigen::Vector3d::UnitZ(),
     true},
    {"x_shallow", -0.02, true, 0.0, Eigen::Vector3d::UnitX(), false},
};

const std::vector<CaseSpec> kPlaneCases = {
    {"above_plane", 0.2, false, 0.0, kPlaneNormal, true},
    {"near_touch", -1e-4, true, 0.0, kPlaneNormal, true},
    {"shallow_overlap", -0.02, true, 0.0, kPlaneNormal, true},
    {"deep_overlap", -0.2, true, 0.0, kPlaneNormal, true},
};

bool isPlane(ShapeKind kind)
{
  return kind == ShapeKind::Plane;
}

std::shared_ptr<fcl::CollisionGeometry<double>> makeGeometry(ShapeKind kind)
{
  switch (kind) {
    case ShapeKind::Box:
      return std::make_shared<fcl::Box<double>>(
          kBoxSize.x(), kBoxSize.y(), kBoxSize.z());
    case ShapeKind::Sphere:
      return std::make_shared<fcl::Sphere<double>>(kSphereRadius);
    case ShapeKind::Cylinder:
      return std::make_shared<fcl::Cylinder<double>>(
          kCylinderRadius, kCylinderHeight);
    case ShapeKind::Ellipsoid:
      return std::make_shared<fcl::Ellipsoid<double>>(
          kEllipsoidRadii.x(), kEllipsoidRadii.y(), kEllipsoidRadii.z());
    case ShapeKind::Cone:
      return std::make_shared<fcl::Cone<double>>(kConeRadius, kConeHeight);
    case ShapeKind::Plane:
      return std::make_shared<fcl::Halfspace<double>>(
          kPlaneNormal, kPlaneOffset);
  }

  return nullptr;
}

bool getExtentAlongDirection(
    ShapeKind kind, const Eigen::Vector3d& direction, double& extent)
{
  const double norm = direction.norm();
  if (norm <= 0.0)
    return false;

  const Eigen::Vector3d dir = direction / norm;
  const Eigen::Vector3d dirAbs = dir.cwiseAbs();

  switch (kind) {
    case ShapeKind::Box:
      extent = 0.5
               * (dirAbs.x() * kBoxSize.x() + dirAbs.y() * kBoxSize.y()
                  + dirAbs.z() * kBoxSize.z());
      return true;
    case ShapeKind::Sphere:
      extent = kSphereRadius;
      return true;
    case ShapeKind::Cylinder: {
      const double xy = std::sqrt(dir.x() * dir.x() + dir.y() * dir.y());
      extent = kCylinderRadius * xy + 0.5 * kCylinderHeight * std::abs(dir.z());
      return true;
    }
    case ShapeKind::Ellipsoid:
      extent = std::sqrt(
          std::pow(kEllipsoidRadii.x() * dir.x(), 2)
          + std::pow(kEllipsoidRadii.y() * dir.y(), 2)
          + std::pow(kEllipsoidRadii.z() * dir.z(), 2));
      return true;
    case ShapeKind::Cone: {
      const double xy = std::sqrt(dir.x() * dir.x() + dir.y() * dir.y());
      const double apex = 0.5 * kConeHeight * dir.z();
      const double base = -0.5 * kConeHeight * dir.z() + kConeRadius * xy;
      extent = std::max(apex, base);
      return true;
    }
    case ShapeKind::Plane:
      return false;
  }

  return false;
}

double planeSignedDistance(
    const Eigen::Vector3d& normal, double offset, const Eigen::Vector3d& point)
{
  return normal.dot(point) - offset;
}

} // namespace

TEST(FclPrimitiveContactsFcl, PairMatrixRespectsFclConventions)
{
  // Issue #19: https://github.com/dartsim/dart/issues/19
  const double kNormalAlignment = 0.95;
  const double kNormalNormTol = 1e-6;
  const double kPointTol = 1e-3;
  const double kPlaneTol = 1e-3;

  for (const auto& shapeA : kShapeSpecs) {
    for (const auto& shapeB : kShapeSpecs) {
      if (isPlane(shapeA.kind) && isPlane(shapeB.kind))
        continue;

      const bool planeA = isPlane(shapeA.kind);
      const bool planeB = isPlane(shapeB.kind);
      const auto& cases = (planeA || planeB) ? kPlaneCases : kBaseCases;

      for (const auto& collisionCase : cases) {
        SCOPED_TRACE(
            std::string("Pair: ") + shapeA.name + " vs " + shapeB.name
            + ", case: " + collisionCase.name);
        if (!collisionCase.allowCone
            && (shapeA.kind == ShapeKind::Cone
                || shapeB.kind == ShapeKind::Cone)) {
          continue;
        }

        auto geom1 = makeGeometry(shapeA.kind);
        auto geom2 = makeGeometry(shapeB.kind);
        if (!geom1 || !geom2) {
          ADD_FAILURE() << "Failed to build FCL geometries.";
          continue;
        }

        Eigen::Vector3d p1 = Eigen::Vector3d::Zero();
        Eigen::Vector3d p2 = Eigen::Vector3d::Zero();

        const Eigen::Vector3d direction = collisionCase.direction.normalized();

        if (planeA || planeB) {
          const ShapeKind otherKind = planeA ? shapeB.kind : shapeA.kind;
          double extent = 0.0;
          if (!getExtentAlongDirection(otherKind, direction, extent)) {
            ADD_FAILURE() << "Unsupported shape for plane contact.";
            continue;
          }

          const double offset = extent + collisionCase.separationOffset;
          const Eigen::Vector3d otherPos = direction * offset;

          if (planeA) {
            p1 = Eigen::Vector3d::Zero();
            p2 = otherPos;
          } else {
            p1 = otherPos;
            p2 = Eigen::Vector3d::Zero();
          }
        } else {
          double extent1 = 0.0;
          double extent2 = 0.0;
          if (!getExtentAlongDirection(shapeA.kind, direction, extent1)
              || !getExtentAlongDirection(shapeB.kind, direction, extent2)) {
            ADD_FAILURE() << "Unsupported shape in contact matrix.";
            continue;
          }

          const double separation
              = extent1 + extent2 + collisionCase.separationOffset;
          p1 = 0.5 * separation * direction;
          p2 = -0.5 * separation * direction;
        }

        fcl::Transform3<double> tf1 = fcl::Transform3<double>::Identity();
        fcl::Transform3<double> tf2 = fcl::Transform3<double>::Identity();
        tf1.translation() = p1;
        tf2.translation() = p2;
        if (collisionCase.yawRadians != 0.0 && !(planeA || planeB)) {
          tf1.linear() = Eigen::AngleAxisd(
                             collisionCase.yawRadians, Eigen::Vector3d::UnitZ())
                             .toRotationMatrix();
        }

        fcl::CollisionObject<double> obj1(geom1, tf1);
        fcl::CollisionObject<double> obj2(geom2, tf2);
        obj1.computeAABB();
        obj2.computeAABB();

        fcl::CollisionRequest<double> request;
        request.enable_contact = true;
        request.num_max_contacts = 20u;

        fcl::CollisionResult<double> result;
        fcl::collide(&obj1, &obj2, request, result);

        if (!collisionCase.expectCollision) {
          EXPECT_FALSE(result.isCollision());
          continue;
        }

        EXPECT_TRUE(result.isCollision());

        std::vector<fcl::Contact<double>> contacts;
        result.getContacts(contacts);
        if (contacts.empty()) {
          ADD_FAILURE() << "No contacts reported.";
          continue;
        }

        Eigen::Vector3d expectedDir = p2 - p1;
        const double expectedNorm = expectedDir.norm();
        if (expectedNorm <= 0.0) {
          ADD_FAILURE() << "Degenerate frame separation.";
          continue;
        }
        expectedDir /= expectedNorm;

        for (const auto& contact : contacts) {
          const Eigen::Vector3d contactNormal = contact.normal;
          const Eigen::Vector3d contactPoint = contact.pos;
          const double alignment = contactNormal.dot(expectedDir);

          EXPECT_NEAR(contactNormal.norm(), 1.0, kNormalNormTol);
          EXPECT_GT(alignment, kNormalAlignment);
          EXPECT_GT(contact.penetration_depth, 0.0);

          if (planeA || planeB) {
            const double dist
                = planeSignedDistance(kPlaneNormal, kPlaneOffset, contactPoint);
            EXPECT_LE(std::abs(dist), contact.penetration_depth + kPlaneTol);
            continue;
          }

          double extent1 = 0.0;
          double extent2 = 0.0;
          if (!getExtentAlongDirection(shapeA.kind, expectedDir, extent1)
              || !getExtentAlongDirection(shapeB.kind, expectedDir, extent2)) {
            ADD_FAILURE() << "Unsupported shape in contact check.";
            continue;
          }

          const double p1Proj = expectedDir.dot(p1);
          const double p2Proj = expectedDir.dot(p2);
          const double surface1 = p1Proj + extent1;
          const double surface2 = p2Proj - extent2;
          const double minSurface = std::min(surface1, surface2);
          const double maxSurface = std::max(surface1, surface2);
          const double contactProj = expectedDir.dot(contactPoint);

          EXPECT_GE(contactProj, minSurface - kPointTol);
          EXPECT_LE(contactProj, maxSurface + kPointTol);
        }
      }
    }
  }
}
