/*
 * Copyright (c) 2011-2023, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/master/LICENSE
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

#include <dart/test/io/TestHelpers.hpp>

#include <dart/simulation/World.hpp>

#include <dart/io/DartResourceRetriever.hpp>

#include <dart/dynamics/BoxShape.hpp>
#include <dart/dynamics/ConstraintSolver.hpp>
#include <dart/dynamics/FreeJoint.hpp>
#include <dart/dynamics/MeshShape.hpp>
#include <dart/dynamics/PlaneShape.hpp>
#include <dart/dynamics/Skeleton.hpp>
#include <dart/dynamics/WeldJoint.hpp>
#include <dart/dynamics/bullet/BulletCollisionDetector.hpp>
#include <dart/dynamics/ode/OdeCollisionDetector.hpp>

#include <gtest/gtest.h>

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

  return collision;
}

//==============================================================================
void runIssue1234Test(
    std::function<dart::collision::CollisionDetectorPtr()> detectorFactory)
{
  const std::string meshUri = "dart://sample/obj/BoxSmall.obj";
  const auto aiscene = dart::dynamics::MeshShape::loadMesh(
      meshUri, dart::io::DartResourceRetriever::create());
  ASSERT_TRUE(aiscene);
  const auto mesh = std::make_shared<dart::dynamics::MeshShape>(
      100.0 * Eigen::Vector3d::Ones(), aiscene, meshUri);

  const auto bb = mesh->getBoundingBox();
  for (int i = 0; i < 3; ++i) {
    EXPECT_NEAR(bb.getMin()[i], -2.0, 1e-3);
    EXPECT_NEAR(bb.getMax()[i], 2.0, 1e-3);
  }

  const auto box
      = std::make_shared<dart::dynamics::BoxShape>(bb.getMax() - bb.getMin());

  const auto sphere = std::make_shared<dart::dynamics::SphereShape>(
      bb.getMax()[0] - bb.getMin()[0]);

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
TEST(DISABLED_Issue1234, Bullet)
{
  runIssue1234Test(
      [] { return dart::collision::BulletCollisionDetector::create(); });
}

//==============================================================================
TEST(DISABLED_Issue1234, ODE)
{
  runIssue1234Test(
      [] { return dart::collision::OdeCollisionDetector::create(); });
}
