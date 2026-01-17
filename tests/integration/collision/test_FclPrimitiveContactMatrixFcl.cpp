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
const Eigen::Vector3d kContainerBoxSize(4.0, 4.0, 4.0);
constexpr double kContainerSphereRadius = 2.0;
const Eigen::Vector3d kContainerEllipsoidRadii(2.0, 1.5, 1.0);
constexpr double kContainerCylinderRadius = 2.0;
constexpr double kContainerCylinderHeight = 4.0;
constexpr double kContainmentOffset = 0.2;
constexpr double kYawQuarterTurn = 0.25 * 3.141592653589793;

const std::vector<ShapeSpec> kShapeSpecs = {
    {ShapeKind::Box, "box"},
    {ShapeKind::Sphere, "sphere"},
    {ShapeKind::Cylinder, "cylinder"},
    {ShapeKind::Ellipsoid, "ellipsoid"},
    {ShapeKind::Cone, "cone"},
    {ShapeKind::Plane, "plane"},
};

const std::vector<ShapeSpec> kContainmentContainers = {
    {ShapeKind::Box, "box"},
    {ShapeKind::Sphere, "sphere"},
    {ShapeKind::Cylinder, "cylinder"},
    {ShapeKind::Ellipsoid, "ellipsoid"},
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

const std::vector<CaseSpec> kEdgeVertexCases = {
    {"edge_shallow", -0.02, true, 0.0, Eigen::Vector3d(1.0, 1.0, 0.0), true},
    {"vertex_shallow", -0.02, true, 0.0, Eigen::Vector3d(1.0, 1.0, 1.0), true},
};

bool isPlane(ShapeKind kind)
{
  return kind == ShapeKind::Plane;
}

bool isEdgeVertexKind(ShapeKind kind)
{
  return kind == ShapeKind::Box || kind == ShapeKind::Cylinder
         || kind == ShapeKind::Cone;
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

std::shared_ptr<fcl::CollisionGeometry<double>> makeContainerGeometry(
    ShapeKind kind)
{
  switch (kind) {
    case ShapeKind::Box:
      return std::make_shared<fcl::Box<double>>(
          kContainerBoxSize.x(), kContainerBoxSize.y(), kContainerBoxSize.z());
    case ShapeKind::Sphere:
      return std::make_shared<fcl::Sphere<double>>(kContainerSphereRadius);
    case ShapeKind::Cylinder:
      return std::make_shared<fcl::Cylinder<double>>(
          kContainerCylinderRadius, kContainerCylinderHeight);
    case ShapeKind::Ellipsoid:
      return std::make_shared<fcl::Ellipsoid<double>>(
          kContainerEllipsoidRadii.x(),
          kContainerEllipsoidRadii.y(),
          kContainerEllipsoidRadii.z());
    case ShapeKind::Cone:
    case ShapeKind::Plane:
      return nullptr;
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

bool getConeSideExtentAlongDirection(
    ShapeKind kind, const Eigen::Vector3d& direction, double& extent)
{
  if (kind == ShapeKind::Cone) {
    if (std::abs(direction.z()) > 1e-6)
      return false;
    extent = 0.5 * kConeRadius;
    return true;
  }

  return getExtentAlongDirection(kind, direction, extent);
}

bool getContainerExtentAlongDirection(
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
               * (dirAbs.x() * kContainerBoxSize.x()
                  + dirAbs.y() * kContainerBoxSize.y()
                  + dirAbs.z() * kContainerBoxSize.z());
      return true;
    case ShapeKind::Sphere:
      extent = kContainerSphereRadius;
      return true;
    case ShapeKind::Cylinder: {
      const double xy = std::sqrt(dir.x() * dir.x() + dir.y() * dir.y());
      extent = kContainerCylinderRadius * xy
               + 0.5 * kContainerCylinderHeight * std::abs(dir.z());
      return true;
    }
    case ShapeKind::Ellipsoid:
      extent = std::sqrt(
          std::pow(kContainerEllipsoidRadii.x() * dir.x(), 2)
          + std::pow(kContainerEllipsoidRadii.y() * dir.y(), 2)
          + std::pow(kContainerEllipsoidRadii.z() * dir.z(), 2));
      return true;
    case ShapeKind::Cone:
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
  const double kNormalAlignment = 0.95;
  const double kEdgeVertexAlignment = 0.7;
  const double kNormalNormTol = 1e-6;
  const double kPointTol = 1e-3;
  const double kPlaneTol = 1e-3;

  for (const auto& shapeA : kShapeSpecs) {
    for (const auto& shapeB : kShapeSpecs) {
      if (isPlane(shapeA.kind) && isPlane(shapeB.kind))
        continue;

      const bool planeA = isPlane(shapeA.kind);
      const bool planeB = isPlane(shapeB.kind);
      std::vector<CaseSpec> cases
          = (planeA || planeB) ? kPlaneCases : kBaseCases;
      if (!planeA && !planeB && shapeA.kind == shapeB.kind
          && isEdgeVertexKind(shapeA.kind)) {
        cases.insert(
            cases.end(), kEdgeVertexCases.begin(), kEdgeVertexCases.end());
      }

      for (const auto& collisionCase : cases) {
        SCOPED_TRACE(
            std::string("Pair: ") + shapeA.name + " vs " + shapeB.name
            + ", case: " + collisionCase.name);
        const bool edgeCase
            = (std::string(collisionCase.name) == "edge_shallow");
        const bool vertexCase
            = (std::string(collisionCase.name) == "vertex_shallow");
        const bool edgeVertexCase = edgeCase || vertexCase;
        const bool useAxisPlacement = edgeVertexCase
                                      && shapeA.kind == ShapeKind::Box
                                      && shapeB.kind == ShapeKind::Box;
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
        } else if (useAxisPlacement) {
          double extent1x = 0.0;
          double extent1y = 0.0;
          double extent1z = 0.0;
          double extent2x = 0.0;
          double extent2y = 0.0;
          double extent2z = 0.0;
          if (!getExtentAlongDirection(
                  shapeA.kind, Eigen::Vector3d::UnitX(), extent1x)
              || !getExtentAlongDirection(
                  shapeA.kind, Eigen::Vector3d::UnitY(), extent1y)
              || !getExtentAlongDirection(
                  shapeB.kind, Eigen::Vector3d::UnitX(), extent2x)
              || !getExtentAlongDirection(
                  shapeB.kind, Eigen::Vector3d::UnitY(), extent2y)) {
            ADD_FAILURE() << "Unsupported shape in edge/vertex placement.";
            continue;
          }
          if (vertexCase) {
            if (!getExtentAlongDirection(
                    shapeA.kind, Eigen::Vector3d::UnitZ(), extent1z)
                || !getExtentAlongDirection(
                    shapeB.kind, Eigen::Vector3d::UnitZ(), extent2z)) {
              ADD_FAILURE() << "Unsupported shape in vertex placement.";
              continue;
            }
          }

          const double offset = collisionCase.separationOffset;
          const Eigen::Vector3d separation(
              extent1x + extent2x + offset,
              extent1y + extent2y + offset,
              vertexCase ? (extent1z + extent2z + offset) : 0.0);
          p1 = 0.5 * separation;
          p2 = -0.5 * separation;
        } else {
          double extent1 = 0.0;
          double extent2 = 0.0;
          if (!getExtentAlongDirection(shapeA.kind, direction, extent1)
              || !getExtentAlongDirection(shapeB.kind, direction, extent2)) {
            ADD_FAILURE() << "Unsupported shape in contact matrix.";
            continue;
          }

          double separation
              = extent1 + extent2 + collisionCase.separationOffset;
          if (edgeVertexCase && !useAxisPlacement) {
            double extent1x = 0.0;
            double extent2x = 0.0;
            double extent1z = 0.0;
            double extent2z = 0.0;
            if (!getExtentAlongDirection(
                    shapeA.kind, Eigen::Vector3d::UnitX(), extent1x)
                || !getExtentAlongDirection(
                    shapeB.kind, Eigen::Vector3d::UnitX(), extent2x)
                || !getExtentAlongDirection(
                    shapeA.kind, Eigen::Vector3d::UnitZ(), extent1z)
                || !getExtentAlongDirection(
                    shapeB.kind, Eigen::Vector3d::UnitZ(), extent2z)) {
              ADD_FAILURE() << "Unsupported shape in edge/vertex placement.";
              continue;
            }

            const double radialFactor = std::sqrt(
                direction.x() * direction.x() + direction.y() * direction.y());
            const double axialFactor = std::abs(direction.z());
            if (radialFactor > 0.0) {
              const double radialSep
                  = (extent1x + extent2x + collisionCase.separationOffset)
                    / radialFactor;
              separation = std::min(separation, radialSep);
            }
            if (axialFactor > 0.0) {
              const double axialSep
                  = (extent1z + extent2z + collisionCase.separationOffset)
                    / axialFactor;
              separation = std::min(separation, axialSep);
            }
          }
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

          const double alignmentThreshold
              = edgeVertexCase ? kEdgeVertexAlignment : kNormalAlignment;

          EXPECT_NEAR(contactNormal.norm(), 1.0, kNormalNormTol);
          EXPECT_GT(alignment, alignmentThreshold);
          EXPECT_GT(contact.penetration_depth, 0.0);

          if (planeA || planeB) {
            const double dist
                = planeSignedDistance(kPlaneNormal, kPlaneOffset, contactPoint);
            EXPECT_LE(std::abs(dist), contact.penetration_depth + kPlaneTol);
            continue;
          }

          const bool coneInvolved
              = (shapeA.kind == ShapeKind::Cone
                 || shapeB.kind == ShapeKind::Cone);
          if (coneInvolved && edgeVertexCase) {
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

TEST(FclPrimitiveContactsFcl, RotatedPlaneNormal)
{
  const double kNormalAlignment = 0.95;
  const double kNormalNormTol = 1e-6;
  const double kPlaneTol = 1e-3;

  struct TiltCase
  {
    const char* name;
    double separationOffset;
    bool expectCollision;
  };

  const std::vector<TiltCase> kTiltCases = {
      {"tilted_above", 0.2, false},
      {"tilted_shallow", -0.02, true},
  };

  const Eigen::Vector3d tiltedNormal
      = Eigen::AngleAxisd(kYawQuarterTurn, Eigen::Vector3d::UnitY())
            .toRotationMatrix()
        * kPlaneNormal;

  for (const auto& shapeSpec : kShapeSpecs) {
    if (isPlane(shapeSpec.kind))
      continue;

    SCOPED_TRACE(std::string("Rotated plane vs ") + shapeSpec.name);

    auto planeGeom
        = std::make_shared<fcl::Halfspace<double>>(tiltedNormal, kPlaneOffset);
    auto otherGeom = makeGeometry(shapeSpec.kind);
    if (!planeGeom || !otherGeom) {
      ADD_FAILURE() << "Failed to build geometries for rotated plane contact.";
      continue;
    }

    fcl::Transform3<double> tfPlane = fcl::Transform3<double>::Identity();
    tfPlane.translation() = Eigen::Vector3d::Zero();

    for (const auto& tiltCase : kTiltCases) {
      SCOPED_TRACE(std::string("Case: ") + tiltCase.name);

      double extent = 0.0;
      if (!getExtentAlongDirection(shapeSpec.kind, -tiltedNormal, extent)) {
        ADD_FAILURE() << "Unsupported shape for rotated plane contact.";
        continue;
      }

      const double offset = extent + tiltCase.separationOffset;
      const Eigen::Vector3d p1 = Eigen::Vector3d::Zero();
      const Eigen::Vector3d p2 = tiltedNormal * offset;

      fcl::Transform3<double> tfOther = fcl::Transform3<double>::Identity();
      tfOther.translation() = p2;

      fcl::CollisionObject<double> obj1(planeGeom, tfPlane);
      fcl::CollisionObject<double> obj2(otherGeom, tfOther);
      obj1.computeAABB();
      obj2.computeAABB();

      fcl::CollisionRequest<double> request;
      request.enable_contact = true;
      request.num_max_contacts = 20u;

      fcl::CollisionResult<double> result;
      fcl::collide(&obj1, &obj2, request, result);

      if (!tiltCase.expectCollision) {
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

      const Eigen::Vector3d worldNormal = tiltedNormal;
      const double worldOffset = kPlaneOffset;

      for (const auto& contact : contacts) {
        const Eigen::Vector3d contactNormal = contact.normal;
        const Eigen::Vector3d contactPoint = contact.pos;
        const double alignment = contactNormal.dot(expectedDir);

        EXPECT_NEAR(contactNormal.norm(), 1.0, kNormalNormTol);
        EXPECT_GT(alignment, kNormalAlignment);
        EXPECT_GT(contact.penetration_depth, 0.0);

        const double dist
            = planeSignedDistance(worldNormal, worldOffset, contactPoint);
        EXPECT_LE(std::abs(dist), contact.penetration_depth + kPlaneTol);
      }
    }
  }
}

TEST(FclPrimitiveContactsFcl, ContainmentCases)
{
  const double kNormalAlignment = 0.95;
  const double kConeNormalAlignment = 0.9;
  const double kNormalNormTol = 1e-6;
  const double kPointTol = 1e-3;

  for (const auto& containerSpec : kContainmentContainers) {
    for (const auto& containedSpec : kShapeSpecs) {
      if (isPlane(containedSpec.kind))
        continue;

      SCOPED_TRACE(
          std::string("Container: ") + containerSpec.name
          + ", contained: " + containedSpec.name);

      auto geomContained = makeGeometry(containedSpec.kind);
      auto geomContainer = makeContainerGeometry(containerSpec.kind);
      if (!geomContained || !geomContainer) {
        ADD_FAILURE() << "Failed to build containment geometries.";
        continue;
      }

      const Eigen::Vector3d p1(kContainmentOffset, 0.0, 0.0);
      const Eigen::Vector3d p2 = Eigen::Vector3d::Zero();

      fcl::Transform3<double> tf1 = fcl::Transform3<double>::Identity();
      fcl::Transform3<double> tf2 = fcl::Transform3<double>::Identity();
      tf1.translation() = p1;
      tf2.translation() = p2;

      fcl::CollisionObject<double> obj1(geomContained, tf1);
      fcl::CollisionObject<double> obj2(geomContainer, tf2);
      obj1.computeAABB();
      obj2.computeAABB();

      fcl::CollisionRequest<double> request;
      request.enable_contact = true;
      request.num_max_contacts = 20u;

      fcl::CollisionResult<double> result;
      fcl::collide(&obj1, &obj2, request, result);

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
        const bool coneInvolved
            = (containedSpec.kind == ShapeKind::Cone
               || containerSpec.kind == ShapeKind::Cone);
        const double alignmentThreshold
            = coneInvolved ? kConeNormalAlignment : kNormalAlignment;

        EXPECT_NEAR(contactNormal.norm(), 1.0, kNormalNormTol);
        EXPECT_GT(alignment, alignmentThreshold);
        EXPECT_GT(contact.penetration_depth, 0.0);

        double extent1 = 0.0;
        double extent2 = 0.0;
        if (!getExtentAlongDirection(containedSpec.kind, expectedDir, extent1)
            || !getContainerExtentAlongDirection(
                containerSpec.kind, expectedDir, extent2)) {
          ADD_FAILURE() << "Unsupported shape in containment check.";
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

TEST(FclPrimitiveContactsFcl, ConeSideContacts)
{
  const double kNormalAlignment = 0.8;
  const double kNormalNormTol = 1e-6;
  const double kPointTol = 1e-3;
  const Eigen::Vector3d direction = Eigen::Vector3d::UnitX();
  const double kSeparationOffset = -0.02;

  for (const auto& shapeA : kShapeSpecs) {
    for (const auto& shapeB : kShapeSpecs) {
      if (isPlane(shapeA.kind) || isPlane(shapeB.kind))
        continue;

      if (shapeA.kind != ShapeKind::Cone && shapeB.kind != ShapeKind::Cone)
        continue;

      SCOPED_TRACE(
          std::string("Pair: ") + shapeA.name + " vs " + shapeB.name
          + ", case: cone_side");

      auto geom1 = makeGeometry(shapeA.kind);
      auto geom2 = makeGeometry(shapeB.kind);
      if (!geom1 || !geom2) {
        ADD_FAILURE() << "Failed to build geometries.";
        continue;
      }

      double extent1 = 0.0;
      double extent2 = 0.0;
      if (!getConeSideExtentAlongDirection(shapeA.kind, direction, extent1)
          || !getConeSideExtentAlongDirection(
              shapeB.kind, direction, extent2)) {
        ADD_FAILURE() << "Unsupported shape in cone side placement.";
        continue;
      }

      const double separation = extent1 + extent2 + kSeparationOffset;
      const Eigen::Vector3d p1 = 0.5 * separation * direction;
      const Eigen::Vector3d p2 = -0.5 * separation * direction;

      fcl::Transform3<double> tf1 = fcl::Transform3<double>::Identity();
      fcl::Transform3<double> tf2 = fcl::Transform3<double>::Identity();
      tf1.translation() = p1;
      tf2.translation() = p2;

      fcl::CollisionObject<double> obj1(geom1, tf1);
      fcl::CollisionObject<double> obj2(geom2, tf2);
      obj1.computeAABB();
      obj2.computeAABB();

      fcl::CollisionRequest<double> request;
      request.enable_contact = true;
      request.num_max_contacts = 20u;

      fcl::CollisionResult<double> result;
      fcl::collide(&obj1, &obj2, request, result);

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

        double checkExtent1 = 0.0;
        double checkExtent2 = 0.0;
        if (!getExtentAlongDirection(shapeA.kind, expectedDir, checkExtent1)
            || !getExtentAlongDirection(
                shapeB.kind, expectedDir, checkExtent2)) {
          ADD_FAILURE() << "Unsupported shape in cone side check.";
          continue;
        }

        const double p1Proj = expectedDir.dot(p1);
        const double p2Proj = expectedDir.dot(p2);
        const double surface1 = p1Proj + checkExtent1;
        const double surface2 = p2Proj - checkExtent2;
        const double minSurface = std::min(surface1, surface2);
        const double maxSurface = std::max(surface1, surface2);
        const double contactProj = expectedDir.dot(contactPoint);

        EXPECT_GE(contactProj, minSurface - kPointTol);
        EXPECT_LE(contactProj, maxSurface + kPointTol);
      }
    }
  }
}
