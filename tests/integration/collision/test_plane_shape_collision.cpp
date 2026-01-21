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

#include <dart/utils/dart_resource_retriever.hpp>

#include <dart/simulation/world.hpp>

#include <dart/constraint/constraint_solver.hpp>

#include <dart/collision/bullet/bullet_collision_detector.hpp>
#include <dart/collision/fcl/fcl_collision_detector.hpp>
#include <dart/collision/ode/ode_collision_detector.hpp>

#include <dart/dynamics/All.hpp>
#include <dart/dynamics/box_shape.hpp>
#include <dart/dynamics/free_joint.hpp>
#include <dart/dynamics/mesh_shape.hpp>
#include <dart/dynamics/plane_shape.hpp>
#include <dart/dynamics/skeleton.hpp>
#include <dart/dynamics/weld_joint.hpp>

#include <dart/common/diagnostics.hpp>

#include <gtest/gtest.h>
#include <helpers/gtest_utils.hpp>

#include <cmath>

using namespace dart::dynamics;
using namespace dart::test;

//==============================================================================
bool runIssue1234Subtest(
    const dart::collision::CollisionDetectorPtr detector,
    const dart::dynamics::ShapePtr& against,
    const Eigen::Vector3d normal,
    const double offset,
    const double angle,
    const Eigen::Vector3d axis)
{
  auto group = detector->createCollisionGroup();

  const Eigen::Matrix3d R(Eigen::AngleAxisd(angle, axis));

  auto plane = dart::dynamics::Skeleton::create("plane");
  plane->createJointAndBodyNodePair<FreeJoint>()
      .second->createShapeNodeWith<dart::dynamics::CollisionAspect>(
          std::make_shared<dart::dynamics::PlaneShape>(R * normal, offset));
  group->subscribeTo(plane);

  auto mesh = dart::dynamics::Skeleton::create("against");
  mesh->createJointAndBodyNodePair<FreeJoint>()
      .second->createShapeNodeWith<dart::dynamics::CollisionAspect>(against)
      ->setRelativeTranslation(R * normal * offset);
  group->subscribeTo(mesh);

  const bool collision = group->collide();
  EXPECT_TRUE(collision) << "\nFailed config:"
                         << "\nNormal:  " << (R * normal).transpose()
                         << "\nOffset:  " << offset
                         << "\nTilt:    " << dart::math::toDegree(angle)
                         << " degrees"
                         << "\nAgainst: " << against->getType() << std::endl;

  group->removeAllShapeFrames();

  return collision;
}

//==============================================================================
void runIssue1234Test(
    std::function<dart::collision::CollisionDetectorPtr()> detectorFactory)
{
  const std::string meshUri = "dart://sample/obj/BoxSmall.obj";

  // Load mesh using MeshShape::loadMesh
  const auto retriever = dart::utils::DartResourceRetriever::create();
  DART_SUPPRESS_DEPRECATED_BEGIN
  const aiScene* aiMesh
      = dart::dynamics::MeshShape::loadMesh(meshUri, retriever);
  DART_SUPPRESS_DEPRECATED_END

  ASSERT_TRUE(aiMesh);
  DART_SUPPRESS_DEPRECATED_BEGIN
  const auto mesh = std::make_shared<dart::dynamics::MeshShape>(
      100.0 * Eigen::Vector3d::Ones(), aiMesh, dart::common::Uri(meshUri));
  DART_SUPPRESS_DEPRECATED_END

  const auto bb = mesh->getBoundingBox();
  for (int i = 0; i < 3; ++i) {
    EXPECT_NEAR(bb.getMin()[i], -2.0, 1e-3);
    EXPECT_NEAR(bb.getMax()[i], 2.0, 1e-3);
  }

  const auto box
      = std::make_shared<dart::dynamics::BoxShape>(bb.getMax() - bb.getMin());

  std::size_t numTests = 0;
  std::size_t numPasses = 0;

  for (const dart::dynamics::ShapePtr& shape :
       {dart::dynamics::ShapePtr{box}, dart::dynamics::ShapePtr{mesh}}) {
    for (const double offset : {-0.1, 0.0, 0.1}) {
      for (const auto& normal :
           {Eigen::Vector3d(0.0, 0.0, 1.0),
            Eigen::Vector3d(0.0, 0.0, -1.0),
            Eigen::Vector3d(0.0, 1.0, 0.0),
            Eigen::Vector3d(0.0, -1.0, 0.0),
            Eigen::Vector3d(1.0, 0.0, 0.0),
            Eigen::Vector3d(-1.0, 0.0, 0.0)}) {
        if (offset < 0.0) {
          for (const auto& angle : {0.0, dart::math::toRadian(10.0)}) {
            const Eigen::Vector3d axis = std::abs(normal[1]) > 1e-3
                                             ? Eigen::Vector3d::UnitY()
                                             : Eigen::Vector3d::UnitX();

            ++numTests;
            if (runIssue1234Subtest(
                    detectorFactory(), shape, normal, offset, angle, axis)) {
              ++numPasses;
            }
          }
        } else {
          ++numTests;
          if (runIssue1234Subtest(
                  detectorFactory(),
                  shape,
                  normal,
                  offset,
                  0.0,
                  Eigen::Vector3d::UnitZ())) {
            ++numPasses;
          }
        }
      }
    }
  }

  EXPECT_EQ(numTests, numPasses);
  std::cout << "Failures: " << numTests - numPasses << std::endl;
}

//==============================================================================
TEST(Issue1234, Bullet)
{
  runIssue1234Test(
      [] { return dart::collision::BulletCollisionDetector::create(); });
}

//==============================================================================
TEST(Issue1234, Ode)
{
  runIssue1234Test(
      [] { return dart::collision::OdeCollisionDetector::create(); });
}

//==============================================================================
TEST(Issue1234, Fcl)
{
  runIssue1234Test([] {
    auto detector = dart::collision::FCLCollisionDetector::create();
    detector->setPrimitiveShapeType(
        dart::collision::FCLCollisionDetector::PRIMITIVE);
    return detector;
  });
}

//==============================================================================
TEST(Issue426, FclThinBoxMeshModeUsesHalfspacePlane)
{
  auto detector = dart::collision::FCLCollisionDetector::create();
  detector->setPrimitiveShapeType(dart::collision::FCLCollisionDetector::MESH);

  const double thickness = 5e-5;

  auto plane = dart::dynamics::Skeleton::create("plane");
  plane->createJointAndBodyNodePair<FreeJoint>()
      .second->createShapeNodeWith<dart::dynamics::CollisionAspect>(
          std::make_shared<dart::dynamics::PlaneShape>(
              Eigen::Vector3d::UnitZ(), 0.0));

  auto thin = dart::dynamics::Skeleton::create("thin");
  auto thinBody = thin->createJointAndBodyNodePair<FreeJoint>().second;
  auto thinShape = std::make_shared<dart::dynamics::BoxShape>(
      Eigen::Vector3d(0.2, 0.2, thickness));
  thinBody->createShapeNodeWith<dart::dynamics::CollisionAspect>(thinShape)
      ->setRelativeTranslation(
          Eigen::Vector3d(
              0.0, 0.0, 0.5 * thickness)); // place the base on the plane

  auto group = detector->createCollisionGroup();
  group->subscribeTo(plane);
  group->subscribeTo(thin);

  dart::collision::CollisionResult result;
  const bool collided
      = group->collide(dart::collision::CollisionOption(), &result);

  EXPECT_TRUE(collided);
  EXPECT_GT(result.getNumContacts(), 0u);

  if (result.getNumContacts() > 0u) {
    const auto& contact = result.getContact(0u);
    EXPECT_NEAR(std::abs(contact.normal[2]), 1.0, 1e-6);
    EXPECT_LE(std::abs(contact.point[2]), thickness);
  }
}
