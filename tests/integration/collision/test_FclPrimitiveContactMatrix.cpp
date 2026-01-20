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

#include <dart/collision/collision_result.hpp>
#include <dart/collision/fcl/fcl_collision_detector.hpp>

#include <dart/dynamics/box_shape.hpp>
#include <dart/dynamics/cone_shape.hpp>
#include <dart/dynamics/cylinder_shape.hpp>
#include <dart/dynamics/ellipsoid_shape.hpp>
#include <dart/dynamics/frame.hpp>
#include <dart/dynamics/plane_shape.hpp>
#include <dart/dynamics/simple_frame.hpp>
#include <dart/dynamics/sphere_shape.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
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
const double kSphereRadius = 0.5;
const Eigen::Vector3d kEllipsoidRadii(0.6, 0.4, 0.3);
const double kCylinderRadius = 0.4;
const double kCylinderHeight = 0.8;
const double kConeRadius = 0.5;
const double kConeHeight = 1.0;
const Eigen::Vector3d kPlaneNormal = Eigen::Vector3d::UnitZ();
const double kPlaneOffset = 0.0;
const Eigen::Vector3d kContainerBoxSize(4.0, 4.0, 4.0);
const double kContainerSphereRadius = 2.0;
const Eigen::Vector3d kContainerEllipsoidRadii(2.0, 1.5, 1.0);
const double kContainerCylinderRadius = 2.0;
const double kContainerCylinderHeight = 4.0;
const double kContainmentOffset = 0.2;
const double kYawQuarterTurn = 0.25 * 3.141592653589793;

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

dart::dynamics::ShapePtr makeShape(ShapeKind kind)
{
  switch (kind) {
    case ShapeKind::Box:
      return std::make_shared<dart::dynamics::BoxShape>(kBoxSize);
    case ShapeKind::Sphere:
      return std::make_shared<dart::dynamics::SphereShape>(kSphereRadius);
    case ShapeKind::Cylinder:
      return std::make_shared<dart::dynamics::CylinderShape>(
          kCylinderRadius, kCylinderHeight);
    case ShapeKind::Ellipsoid:
      return std::make_shared<dart::dynamics::EllipsoidShape>(
          kEllipsoidRadii * 2.0);
    case ShapeKind::Cone:
      return std::make_shared<dart::dynamics::ConeShape>(
          kConeRadius, kConeHeight);
    case ShapeKind::Plane:
      return std::make_shared<dart::dynamics::PlaneShape>(
          kPlaneNormal, kPlaneOffset);
  }

  return nullptr;
}

dart::dynamics::ShapePtr makeContainerShape(ShapeKind kind)
{
  switch (kind) {
    case ShapeKind::Box:
      return std::make_shared<dart::dynamics::BoxShape>(kContainerBoxSize);
    case ShapeKind::Sphere:
      return std::make_shared<dart::dynamics::SphereShape>(
          kContainerSphereRadius);
    case ShapeKind::Cylinder:
      return std::make_shared<dart::dynamics::CylinderShape>(
          kContainerCylinderRadius, kContainerCylinderHeight);
    case ShapeKind::Ellipsoid:
      return std::make_shared<dart::dynamics::EllipsoidShape>(
          kContainerEllipsoidRadii * 2.0);
    case ShapeKind::Cone:
    case ShapeKind::Plane:
      return nullptr;
  }

  return nullptr;
}

bool getExtentAlongDirection(
    const dart::dynamics::Shape& shape,
    const Eigen::Vector3d& direction,
    double& extent)
{
  const double norm = direction.norm();
  if (norm <= 0.0)
    return false;

  const Eigen::Vector3d dir = direction / norm;
  const Eigen::Vector3d dirAbs = dir.cwiseAbs();

  if (const auto* box = dynamic_cast<const dart::dynamics::BoxShape*>(&shape)) {
    const Eigen::Vector3d size = box->getSize();
    extent = 0.5
             * (dirAbs.x() * size.x() + dirAbs.y() * size.y()
                + dirAbs.z() * size.z());
    return true;
  }
  if (const auto* sphere
      = dynamic_cast<const dart::dynamics::SphereShape*>(&shape)) {
    extent = sphere->getRadius();
    return true;
  }
  if (const auto* ellipsoid
      = dynamic_cast<const dart::dynamics::EllipsoidShape*>(&shape)) {
    const Eigen::Vector3d radii = ellipsoid->getRadii();
    extent = std::sqrt(
        std::pow(radii.x() * dir.x(), 2) + std::pow(radii.y() * dir.y(), 2)
        + std::pow(radii.z() * dir.z(), 2));
    return true;
  }
  if (const auto* cylinder
      = dynamic_cast<const dart::dynamics::CylinderShape*>(&shape)) {
    const double xy = std::sqrt(dir.x() * dir.x() + dir.y() * dir.y());
    extent = cylinder->getRadius() * xy
             + 0.5 * cylinder->getHeight() * std::abs(dir.z());
    return true;
  }
  if (const auto* cone
      = dynamic_cast<const dart::dynamics::ConeShape*>(&shape)) {
    const double xy = std::sqrt(dir.x() * dir.x() + dir.y() * dir.y());
    const double apex = 0.5 * cone->getHeight() * dir.z();
    const double base
        = -0.5 * cone->getHeight() * dir.z() + cone->getRadius() * xy;
    extent = std::max(apex, base);
    return true;
  }

  return false;
}

bool getConeSideExtentAlongDirection(
    const dart::dynamics::Shape& shape,
    const Eigen::Vector3d& direction,
    double& extent)
{
  if (const auto* cone
      = dynamic_cast<const dart::dynamics::ConeShape*>(&shape)) {
    if (std::abs(direction.z()) > 1e-6)
      return false;
    extent = 0.5 * cone->getRadius();
    return true;
  }

  return getExtentAlongDirection(shape, direction, extent);
}

double planeSignedDistance(
    const dart::dynamics::PlaneShape& plane,
    const dart::dynamics::ShapeFrame& frame,
    const Eigen::Vector3d& point)
{
  const Eigen::Isometry3d& tf = frame.getWorldTransform();
  const Eigen::Vector3d normal = tf.linear() * plane.getNormal();
  const double offset = plane.getOffset() + normal.dot(tf.translation());
  return normal.dot(point) - offset;
}

} // namespace

TEST(FclPrimitiveContacts, PairMatrixRespectsDartConventions)
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

        auto detector = dart::collision::FCLCollisionDetector::create();
        detector->setPrimitiveShapeType(
            dart::collision::FCLCollisionDetector::PRIMITIVE);

        auto shape1 = makeShape(shapeA.kind);
        auto shape2 = makeShape(shapeB.kind);
        if (!shape1 || !shape2) {
          ADD_FAILURE() << "Failed to build shapes for pair.";
          continue;
        }

        auto frame1 = dart::dynamics::SimpleFrame::createShared(
            dart::dynamics::Frame::World(),
            std::string("frame1_") + shapeA.name + "_" + shapeB.name);
        auto frame2 = dart::dynamics::SimpleFrame::createShared(
            dart::dynamics::Frame::World(),
            std::string("frame2_") + shapeA.name + "_" + shapeB.name);

        frame1->setShape(shape1);
        frame2->setShape(shape2);

        const Eigen::Vector3d direction = collisionCase.direction.normalized();

        if (planeA || planeB) {
          auto* planeFrame = planeA ? frame1.get() : frame2.get();
          auto* otherFrame = planeA ? frame2.get() : frame1.get();

          const auto* otherShape = otherFrame->getShape().get();
          double extent = 0.0;
          if (!getExtentAlongDirection(*otherShape, direction, extent)) {
            ADD_FAILURE() << "Unsupported shape for plane contact.";
            continue;
          }

          const double offset = extent + collisionCase.separationOffset;
          planeFrame->setTranslation(Eigen::Vector3d::Zero());
          otherFrame->setTranslation(direction * offset);
        } else if (useAxisPlacement) {
          double extent1x = 0.0;
          double extent1y = 0.0;
          double extent1z = 0.0;
          double extent2x = 0.0;
          double extent2y = 0.0;
          double extent2z = 0.0;
          if (!getExtentAlongDirection(
                  *shape1, Eigen::Vector3d::UnitX(), extent1x)
              || !getExtentAlongDirection(
                  *shape1, Eigen::Vector3d::UnitY(), extent1y)
              || !getExtentAlongDirection(
                  *shape2, Eigen::Vector3d::UnitX(), extent2x)
              || !getExtentAlongDirection(
                  *shape2, Eigen::Vector3d::UnitY(), extent2y)) {
            ADD_FAILURE() << "Unsupported shape in edge/vertex placement.";
            continue;
          }
          if (vertexCase) {
            if (!getExtentAlongDirection(
                    *shape1, Eigen::Vector3d::UnitZ(), extent1z)
                || !getExtentAlongDirection(
                    *shape2, Eigen::Vector3d::UnitZ(), extent2z)) {
              ADD_FAILURE() << "Unsupported shape in vertex placement.";
              continue;
            }
          }

          const double offset = collisionCase.separationOffset;
          const Eigen::Vector3d separation(
              extent1x + extent2x + offset,
              extent1y + extent2y + offset,
              vertexCase ? (extent1z + extent2z + offset) : 0.0);
          frame1->setTranslation(0.5 * separation);
          frame2->setTranslation(-0.5 * separation);
        } else {
          double extent1 = 0.0;
          double extent2 = 0.0;
          if (!getExtentAlongDirection(*shape1, direction, extent1)
              || !getExtentAlongDirection(*shape2, direction, extent2)) {
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
                    *shape1, Eigen::Vector3d::UnitX(), extent1x)
                || !getExtentAlongDirection(
                    *shape2, Eigen::Vector3d::UnitX(), extent2x)
                || !getExtentAlongDirection(
                    *shape1, Eigen::Vector3d::UnitZ(), extent1z)
                || !getExtentAlongDirection(
                    *shape2, Eigen::Vector3d::UnitZ(), extent2z)) {
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
          frame1->setTranslation(0.5 * separation * direction);
          frame2->setTranslation(-0.5 * separation * direction);
        }

        if (collisionCase.yawRadians != 0.0 && !(planeA || planeB)) {
          frame1->setRotation(
              Eigen::AngleAxisd(
                  collisionCase.yawRadians, Eigen::Vector3d::UnitZ())
                  .toRotationMatrix());
        }

        auto group = detector->createCollisionGroup(frame1.get(), frame2.get());

        dart::collision::CollisionOption option;
        option.enableContact = true;
        option.maxNumContacts = 20u;

        dart::collision::CollisionResult result;
        const bool collided = group->collide(option, &result);

        if (!collisionCase.expectCollision) {
          EXPECT_FALSE(collided);
          continue;
        }

        EXPECT_TRUE(collided);

        if (!collided || result.getNumContacts() == 0u) {
          ADD_FAILURE() << "No contacts reported.";
          continue;
        }

        for (std::size_t i = 0; i < result.getNumContacts(); ++i) {
          const auto& contact = result.getContact(i);

          const auto* frameA = contact.getShapeFrame1();
          const auto* frameB = contact.getShapeFrame2();
          if (!frameA || !frameB) {
            ADD_FAILURE() << "Missing shape frames in contact.";
            continue;
          }

          const Eigen::Vector3d p1 = frameA->getWorldTransform().translation();
          const Eigen::Vector3d p2 = frameB->getWorldTransform().translation();
          const Eigen::Vector3d delta = p1 - p2;
          const double deltaNorm = delta.norm();
          if (deltaNorm <= 0.0) {
            ADD_FAILURE() << "Degenerate frame separation.";
            continue;
          }

          const Eigen::Vector3d expectedDir = delta / deltaNorm;
          const double alignment = contact.normal.dot(expectedDir);

          const double alignmentThreshold
              = edgeVertexCase ? kEdgeVertexAlignment : kNormalAlignment;

          EXPECT_NEAR(contact.normal.norm(), 1.0, kNormalNormTol);
          EXPECT_GT(alignment, alignmentThreshold);
          EXPECT_GT(contact.penetrationDepth, 0.0);

          const auto* planeShapeA
              = dynamic_cast<const dart::dynamics::PlaneShape*>(
                  frameA->getShape().get());
          const auto* planeShapeB
              = dynamic_cast<const dart::dynamics::PlaneShape*>(
                  frameB->getShape().get());

          if (planeShapeA || planeShapeB) {
            const auto* planeShape = planeShapeA ? planeShapeA : planeShapeB;
            const auto* planeFrame = planeShapeA ? frameA : frameB;
            const double dist
                = planeSignedDistance(*planeShape, *planeFrame, contact.point);
            EXPECT_LE(std::abs(dist), contact.penetrationDepth + kPlaneTol);
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
          if (!getExtentAlongDirection(
                  *frameA->getShape(), expectedDir, extent1)
              || !getExtentAlongDirection(
                  *frameB->getShape(), expectedDir, extent2)) {
            ADD_FAILURE() << "Unsupported shape in contact check.";
            continue;
          }

          const double p1Proj = expectedDir.dot(p1);
          const double p2Proj = expectedDir.dot(p2);
          const double surface1 = p1Proj - extent1;
          const double surface2 = p2Proj + extent2;
          const double minSurface = std::min(surface1, surface2);
          const double maxSurface = std::max(surface1, surface2);
          const double contactProj = expectedDir.dot(contact.point);

          EXPECT_GE(contactProj, minSurface - kPointTol);
          EXPECT_LE(contactProj, maxSurface + kPointTol);
        }
      }
    }
  }
}

TEST(FclPrimitiveContacts, RotatedPlaneNormal)
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

    auto detector = dart::collision::FCLCollisionDetector::create();
    detector->setPrimitiveShapeType(
        dart::collision::FCLCollisionDetector::PRIMITIVE);

    auto planeShape = std::make_shared<dart::dynamics::PlaneShape>(
        tiltedNormal, kPlaneOffset);
    auto otherShape = makeShape(shapeSpec.kind);
    if (!planeShape || !otherShape) {
      ADD_FAILURE() << "Failed to build shapes for rotated plane contact.";
      continue;
    }

    auto planeFrame = dart::dynamics::SimpleFrame::createShared(
        dart::dynamics::Frame::World(),
        std::string("plane_tilt_") + shapeSpec.name);
    auto otherFrame = dart::dynamics::SimpleFrame::createShared(
        dart::dynamics::Frame::World(),
        std::string("other_tilt_") + shapeSpec.name);

    planeFrame->setShape(planeShape);
    otherFrame->setShape(otherShape);

    planeFrame->setRotation(Eigen::Matrix3d::Identity());
    planeFrame->setTranslation(Eigen::Vector3d::Zero());

    for (const auto& tiltCase : kTiltCases) {
      SCOPED_TRACE(std::string("Case: ") + tiltCase.name);

      double extent = 0.0;
      if (!getExtentAlongDirection(*otherShape, -tiltedNormal, extent)) {
        ADD_FAILURE() << "Unsupported shape for rotated plane contact.";
        continue;
      }

      const double offset = extent + tiltCase.separationOffset;
      otherFrame->setTranslation(tiltedNormal * offset);

      auto group
          = detector->createCollisionGroup(planeFrame.get(), otherFrame.get());

      dart::collision::CollisionOption option;
      option.enableContact = true;
      option.maxNumContacts = 20u;

      dart::collision::CollisionResult result;
      const bool collided = group->collide(option, &result);

      if (!tiltCase.expectCollision) {
        EXPECT_FALSE(collided);
        continue;
      }

      EXPECT_TRUE(collided);

      if (!collided || result.getNumContacts() == 0u) {
        ADD_FAILURE() << "No contacts reported.";
        continue;
      }

      for (std::size_t i = 0; i < result.getNumContacts(); ++i) {
        const auto& contact = result.getContact(i);

        const auto* frameA = contact.getShapeFrame1();
        const auto* frameB = contact.getShapeFrame2();
        if (!frameA || !frameB) {
          ADD_FAILURE() << "Missing shape frames in contact.";
          continue;
        }

        const Eigen::Vector3d p1 = frameA->getWorldTransform().translation();
        const Eigen::Vector3d p2 = frameB->getWorldTransform().translation();
        const Eigen::Vector3d delta = p1 - p2;
        const double deltaNorm = delta.norm();
        if (deltaNorm <= 0.0) {
          ADD_FAILURE() << "Degenerate frame separation.";
          continue;
        }

        const Eigen::Vector3d expectedDir = delta / deltaNorm;
        const double alignment = contact.normal.dot(expectedDir);

        EXPECT_NEAR(contact.normal.norm(), 1.0, kNormalNormTol);
        EXPECT_GT(alignment, kNormalAlignment);
        EXPECT_GT(contact.penetrationDepth, 0.0);

        const auto* planeShapeLocal
            = dynamic_cast<const dart::dynamics::PlaneShape*>(
                planeFrame->getShape().get());
        if (!planeShapeLocal) {
          ADD_FAILURE() << "Missing plane shape.";
          continue;
        }

        const double dist
            = planeSignedDistance(*planeShapeLocal, *planeFrame, contact.point);
        EXPECT_LE(std::abs(dist), contact.penetrationDepth + kPlaneTol);
      }
    }
  }
}

TEST(FclPrimitiveContacts, ContainmentCases)
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

      auto detector = dart::collision::FCLCollisionDetector::create();
      detector->setPrimitiveShapeType(
          dart::collision::FCLCollisionDetector::PRIMITIVE);

      auto containerShape = makeContainerShape(containerSpec.kind);
      auto containedShape = makeShape(containedSpec.kind);
      if (!containerShape || !containedShape) {
        ADD_FAILURE() << "Failed to build containment shapes.";
        continue;
      }

      auto containedFrame = dart::dynamics::SimpleFrame::createShared(
          dart::dynamics::Frame::World(),
          std::string("contained_") + containedSpec.name + "_in_"
              + containerSpec.name);
      auto containerFrame = dart::dynamics::SimpleFrame::createShared(
          dart::dynamics::Frame::World(),
          std::string("container_") + containerSpec.name + "_with_"
              + containedSpec.name);

      containedFrame->setShape(containedShape);
      containerFrame->setShape(containerShape);

      containerFrame->setTranslation(Eigen::Vector3d::Zero());
      containedFrame->setTranslation(
          Eigen::Vector3d(kContainmentOffset, 0.0, 0.0));

      auto group = detector->createCollisionGroup(
          containedFrame.get(), containerFrame.get());

      dart::collision::CollisionOption option;
      option.enableContact = true;
      option.maxNumContacts = 20u;

      dart::collision::CollisionResult result;
      const bool collided = group->collide(option, &result);

      EXPECT_TRUE(collided);

      if (!collided || result.getNumContacts() == 0u) {
        ADD_FAILURE() << "No contacts reported.";
        continue;
      }

      for (std::size_t i = 0; i < result.getNumContacts(); ++i) {
        const auto& contact = result.getContact(i);

        const auto* frameA = contact.getShapeFrame1();
        const auto* frameB = contact.getShapeFrame2();
        if (!frameA || !frameB) {
          ADD_FAILURE() << "Missing shape frames in contact.";
          continue;
        }

        const Eigen::Vector3d p1 = frameA->getWorldTransform().translation();
        const Eigen::Vector3d p2 = frameB->getWorldTransform().translation();
        const Eigen::Vector3d delta = p1 - p2;
        const double deltaNorm = delta.norm();
        if (deltaNorm <= 0.0) {
          ADD_FAILURE() << "Degenerate frame separation.";
          continue;
        }

        const Eigen::Vector3d expectedDir = delta / deltaNorm;
        const double alignment = contact.normal.dot(expectedDir);
        const bool coneInvolved
            = (containedSpec.kind == ShapeKind::Cone
               || containerSpec.kind == ShapeKind::Cone);
        const double alignmentThreshold
            = coneInvolved ? kConeNormalAlignment : kNormalAlignment;

        EXPECT_NEAR(contact.normal.norm(), 1.0, kNormalNormTol);
        EXPECT_GT(alignment, alignmentThreshold);
        EXPECT_GT(contact.penetrationDepth, 0.0);

        double extent1 = 0.0;
        double extent2 = 0.0;
        if (!getExtentAlongDirection(*frameA->getShape(), expectedDir, extent1)
            || !getExtentAlongDirection(
                *frameB->getShape(), expectedDir, extent2)) {
          ADD_FAILURE() << "Unsupported shape in containment check.";
          continue;
        }

        const double p1Proj = expectedDir.dot(p1);
        const double p2Proj = expectedDir.dot(p2);
        const double surface1 = p1Proj - extent1;
        const double surface2 = p2Proj + extent2;
        const double minSurface = std::min(surface1, surface2);
        const double maxSurface = std::max(surface1, surface2);
        const double contactProj = expectedDir.dot(contact.point);

        EXPECT_GE(contactProj, minSurface - kPointTol);
        EXPECT_LE(contactProj, maxSurface + kPointTol);
      }
    }
  }
}

TEST(FclPrimitiveContacts, ConeSideContacts)
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

      auto detector = dart::collision::FCLCollisionDetector::create();
      detector->setPrimitiveShapeType(
          dart::collision::FCLCollisionDetector::PRIMITIVE);

      auto shape1 = makeShape(shapeA.kind);
      auto shape2 = makeShape(shapeB.kind);
      if (!shape1 || !shape2) {
        ADD_FAILURE() << "Failed to build shapes for cone side contact.";
        continue;
      }

      auto frame1 = dart::dynamics::SimpleFrame::createShared(
          dart::dynamics::Frame::World(),
          std::string("cone_side_frame1_") + shapeA.name + "_" + shapeB.name);
      auto frame2 = dart::dynamics::SimpleFrame::createShared(
          dart::dynamics::Frame::World(),
          std::string("cone_side_frame2_") + shapeA.name + "_" + shapeB.name);

      frame1->setShape(shape1);
      frame2->setShape(shape2);

      double extent1 = 0.0;
      double extent2 = 0.0;
      if (!getConeSideExtentAlongDirection(*shape1, direction, extent1)
          || !getConeSideExtentAlongDirection(*shape2, direction, extent2)) {
        ADD_FAILURE() << "Unsupported shape in cone side placement.";
        continue;
      }

      const double separation = extent1 + extent2 + kSeparationOffset;
      frame1->setTranslation(0.5 * separation * direction);
      frame2->setTranslation(-0.5 * separation * direction);

      auto group = detector->createCollisionGroup(frame1.get(), frame2.get());

      dart::collision::CollisionOption option;
      option.enableContact = true;
      option.maxNumContacts = 20u;

      dart::collision::CollisionResult result;
      const bool collided = group->collide(option, &result);

      EXPECT_TRUE(collided);

      if (!collided || result.getNumContacts() == 0u) {
        ADD_FAILURE() << "No contacts reported.";
        continue;
      }

      for (std::size_t i = 0; i < result.getNumContacts(); ++i) {
        const auto& contact = result.getContact(i);

        const auto* frameA = contact.getShapeFrame1();
        const auto* frameB = contact.getShapeFrame2();
        if (!frameA || !frameB) {
          ADD_FAILURE() << "Missing shape frames in contact.";
          continue;
        }

        const Eigen::Vector3d p1 = frameA->getWorldTransform().translation();
        const Eigen::Vector3d p2 = frameB->getWorldTransform().translation();
        const Eigen::Vector3d delta = p1 - p2;
        const double deltaNorm = delta.norm();
        if (deltaNorm <= 0.0) {
          ADD_FAILURE() << "Degenerate frame separation.";
          continue;
        }

        const Eigen::Vector3d expectedDir = delta / deltaNorm;
        const double alignment = contact.normal.dot(expectedDir);

        EXPECT_NEAR(contact.normal.norm(), 1.0, kNormalNormTol);
        EXPECT_GT(alignment, kNormalAlignment);
        EXPECT_GT(contact.penetrationDepth, 0.0);

        double checkExtent1 = 0.0;
        double checkExtent2 = 0.0;
        if (!getExtentAlongDirection(
                *frameA->getShape(), expectedDir, checkExtent1)
            || !getExtentAlongDirection(
                *frameB->getShape(), expectedDir, checkExtent2)) {
          ADD_FAILURE() << "Unsupported shape in cone side check.";
          continue;
        }

        const double p1Proj = expectedDir.dot(p1);
        const double p2Proj = expectedDir.dot(p2);
        const double surface1 = p1Proj - checkExtent1;
        const double surface2 = p2Proj + checkExtent2;
        const double minSurface = std::min(surface1, surface2);
        const double maxSurface = std::max(surface1, surface2);
        const double contactProj = expectedDir.dot(contact.point);

        EXPECT_GE(contactProj, minSurface - kPointTol);
        EXPECT_LE(contactProj, maxSurface + kPointTol);
      }
    }
  }
}
