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
#include <dart/dynamics/CylinderShape.hpp>
#include <dart/dynamics/EllipsoidShape.hpp>
#include <dart/dynamics/Frame.hpp>
#include <dart/dynamics/PlaneShape.hpp>
#include <dart/dynamics/SimpleFrame.hpp>
#include <dart/dynamics/SphereShape.hpp>

#include <Eigen/Core>
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

namespace {

enum class ShapeKind
{
  Box,
  Sphere,
  Cylinder,
  Ellipsoid,
  Plane
};

struct ShapeSpec
{
  ShapeKind kind;
  const char* name;
};

const std::vector<ShapeSpec> kShapeSpecs = {
    {ShapeKind::Box, "box"},
    {ShapeKind::Sphere, "sphere"},
    {ShapeKind::Cylinder, "cylinder"},
    {ShapeKind::Ellipsoid, "ellipsoid"},
    {ShapeKind::Plane, "plane"},
};

bool isPlane(ShapeKind kind)
{
  return kind == ShapeKind::Plane;
}

dart::dynamics::ShapePtr makeShape(ShapeKind kind)
{
  switch (kind) {
    case ShapeKind::Box:
      return std::make_shared<dart::dynamics::BoxShape>(
          Eigen::Vector3d::Ones());
    case ShapeKind::Sphere:
      return std::make_shared<dart::dynamics::SphereShape>(0.5);
    case ShapeKind::Cylinder:
      return std::make_shared<dart::dynamics::CylinderShape>(0.5, 1.0);
    case ShapeKind::Ellipsoid:
      return std::make_shared<dart::dynamics::EllipsoidShape>(
          Eigen::Vector3d::Constant(0.5));
    case ShapeKind::Plane:
      return std::make_shared<dart::dynamics::PlaneShape>(
          Eigen::Vector3d::UnitZ(), 0.0);
  }

  return nullptr;
}

int axisIndexFromDirection(const Eigen::Vector3d& direction)
{
  int axisIndex = 0;
  direction.cwiseAbs().maxCoeff(&axisIndex);
  return axisIndex;
}

bool getExtentAlongAxis(
    const dart::dynamics::Shape& shape, int axisIndex, double& extent)
{
  if (const auto* box = dynamic_cast<const dart::dynamics::BoxShape*>(&shape)) {
    extent = 0.5 * box->getSize()[axisIndex];
    return true;
  }
  if (const auto* sphere
      = dynamic_cast<const dart::dynamics::SphereShape*>(&shape)) {
    extent = sphere->getRadius();
    return true;
  }
  if (const auto* ellipsoid
      = dynamic_cast<const dart::dynamics::EllipsoidShape*>(&shape)) {
    extent = ellipsoid->getRadii()[axisIndex];
    return true;
  }
  if (const auto* cylinder
      = dynamic_cast<const dart::dynamics::CylinderShape*>(&shape)) {
    extent = (axisIndex == 2) ? 0.5 * cylinder->getHeight()
                              : cylinder->getRadius();
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
  const double kOverlap = 0.1;
  const double kNormalAlignment = 0.95;
  const double kNormalNormTol = 1e-6;
  const double kPointTol = 1e-3;
  const double kPlaneTol = 1e-3;

  for (const auto& shapeA : kShapeSpecs) {
    for (const auto& shapeB : kShapeSpecs) {
      if (isPlane(shapeA.kind) && isPlane(shapeB.kind))
        continue;

      SCOPED_TRACE(
          std::string("Pair: ") + shapeA.name + " vs " + shapeB.name);

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

      if (isPlane(shapeA.kind) || isPlane(shapeB.kind)) {
        const auto* planeFrame = isPlane(shapeA.kind) ? frame1.get()
                                                      : frame2.get();
        const auto* otherFrame = isPlane(shapeA.kind) ? frame2.get()
                                                      : frame1.get();

        const auto* otherShape = otherFrame->getShape().get();
        double extent = 0.0;
        if (!getExtentAlongAxis(*otherShape, 2, extent)) {
          ADD_FAILURE() << "Unsupported shape for plane contact.";
          continue;
        }

        planeFrame->setTranslation(Eigen::Vector3d::Zero());
        otherFrame->setTranslation(Eigen::Vector3d(0.0, 0.0, extent - kOverlap));
      } else {
        double extent1 = 0.0;
        double extent2 = 0.0;
        if (!getExtentAlongAxis(*shape1, 2, extent1)
            || !getExtentAlongAxis(*shape2, 2, extent2)) {
          ADD_FAILURE() << "Unsupported shape in contact matrix.";
          continue;
        }

        const double distance = extent1 + extent2 - kOverlap;
        frame1->setTranslation(Eigen::Vector3d(0.0, 0.0, 0.5 * distance));
        frame2->setTranslation(Eigen::Vector3d(0.0, 0.0, -0.5 * distance));
      }

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
          EXPECT_NEAR(dist, 0.0, kPlaneTol);
          continue;
        }

        const int axisIndex = axisIndexFromDirection(expectedDir);
        double extent1 = 0.0;
        double extent2 = 0.0;
        if (!getExtentAlongAxis(*frameA->getShape(), axisIndex, extent1)
            || !getExtentAlongAxis(*frameB->getShape(), axisIndex, extent2)) {
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
