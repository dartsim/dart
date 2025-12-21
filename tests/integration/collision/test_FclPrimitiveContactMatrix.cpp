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

#include <dart/collision/CollisionResult.hpp>
#include <dart/collision/fcl/FCLCollisionDetector.hpp>

#include <dart/dynamics/BoxShape.hpp>
#include <dart/dynamics/ConeShape.hpp>
#include <dart/dynamics/CylinderShape.hpp>
#include <dart/dynamics/EllipsoidShape.hpp>
#include <dart/dynamics/Frame.hpp>
#include <dart/dynamics/PlaneShape.hpp>
#include <dart/dynamics/SimpleFrame.hpp>
#include <dart/dynamics/SphereShape.hpp>

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
const double kYawQuarterTurn = 0.25 * 3.141592653589793;

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
        } else {
          double extent1 = 0.0;
          double extent2 = 0.0;
          if (!getExtentAlongDirection(*shape1, direction, extent1)
              || !getExtentAlongDirection(*shape2, direction, extent2)) {
            ADD_FAILURE() << "Unsupported shape in contact matrix.";
            continue;
          }

          const double separation
              = extent1 + extent2 + collisionCase.separationOffset;
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

          EXPECT_NEAR(contact.normal.norm(), 1.0, kNormalNormTol);
          EXPECT_GT(alignment, kNormalAlignment);
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
