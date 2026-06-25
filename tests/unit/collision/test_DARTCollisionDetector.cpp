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

#include <dart/collision/CollisionObject.hpp>
#include <dart/collision/dart/DARTCollisionDetector.hpp>

#include <dart/dynamics/PlaneShape.hpp>
#include <dart/dynamics/SimpleFrame.hpp>
#include <dart/dynamics/SphereShape.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <vector>

using namespace dart;

namespace {

// Must match the DART native duplicate-contact tolerance. This is intentionally
// duplicated here so the test exercises the public collision path, not private
// grid internals.
constexpr double kDuplicateContactTolerance = 3.0e-12;
constexpr double kDuplicateContactCellSize = 4.0 * kDuplicateContactTolerance;

struct PlaneSphereGroups
{
  collision::CollisionDetectorPtr detector;
  dynamics::SimpleFramePtr planeFrame;
  dynamics::SimpleFramePtr sphereFrame1;
  dynamics::SimpleFramePtr sphereFrame2;
  std::unique_ptr<collision::CollisionGroup> planeGroup;
  std::unique_ptr<collision::CollisionGroup> sphereGroup;
};

PlaneSphereGroups makePlaneSphereGroups(double sphereX1, double sphereX2)
{
  PlaneSphereGroups groups;
  groups.detector = collision::DARTCollisionDetector::create();

  groups.planeFrame
      = dynamics::SimpleFrame::createShared(dynamics::Frame::World());
  groups.sphereFrame1
      = dynamics::SimpleFrame::createShared(dynamics::Frame::World());
  groups.sphereFrame2
      = dynamics::SimpleFrame::createShared(dynamics::Frame::World());

  groups.planeFrame->setShape(
      std::make_shared<dynamics::PlaneShape>(Eigen::Vector3d::UnitZ(), 0.0));
  groups.sphereFrame1->setShape(std::make_shared<dynamics::SphereShape>(1.0));
  groups.sphereFrame2->setShape(std::make_shared<dynamics::SphereShape>(1.0));

  groups.sphereFrame1->setTranslation(Eigen::Vector3d(sphereX1, 0.0, 0.999));
  groups.sphereFrame2->setTranslation(Eigen::Vector3d(sphereX2, 0.0, 0.999));

  groups.planeGroup
      = groups.detector->createCollisionGroup(groups.planeFrame.get());
  groups.sphereGroup = groups.detector->createCollisionGroup(
      groups.sphereFrame1.get(), groups.sphereFrame2.get());

  return groups;
}

} // namespace

//==============================================================================
TEST(DARTCollisionDetector, DeduplicatesPlaneContactsAcrossGridCellBoundary)
{
  const double x1
      = kDuplicateContactCellSize - 0.25 * kDuplicateContactTolerance;
  const double x2
      = kDuplicateContactCellSize + 0.25 * kDuplicateContactTolerance;
  auto groups = makePlaneSphereGroups(x1, x2);

  collision::CollisionOption option;
  option.enableContact = true;
  option.maxNumContacts = 10u;

  collision::CollisionResult result;
  const bool collided
      = groups.planeGroup->collide(groups.sphereGroup.get(), option, &result);

  EXPECT_TRUE(collided);
  ASSERT_EQ(1u, result.getNumContacts());
  EXPECT_NEAR(result.getContact(0).point.x(), x1, kDuplicateContactTolerance);
}

//==============================================================================
TEST(DARTCollisionDetector, KeepsDistinctPlaneContactsInNearbyGridCells)
{
  const double x1
      = kDuplicateContactCellSize - 0.25 * kDuplicateContactTolerance;
  const double x2 = x1 + 2.0 * kDuplicateContactTolerance;
  auto groups = makePlaneSphereGroups(x1, x2);

  collision::CollisionOption option;
  option.enableContact = true;
  option.maxNumContacts = 10u;

  collision::CollisionResult result;
  const bool collided
      = groups.planeGroup->collide(groups.sphereGroup.get(), option, &result);

  EXPECT_TRUE(collided);
  ASSERT_EQ(2u, result.getNumContacts());
  EXPECT_NEAR(result.getContact(0).point.x(), x1, kDuplicateContactTolerance);
  EXPECT_NEAR(result.getContact(1).point.x(), x2, kDuplicateContactTolerance);
}

//==============================================================================
TEST(DARTCollisionDetector, ParallelDisjointSinglePlaneContactsMatchSerial)
{
  constexpr std::size_t kNumSpheres = 140u;

  auto detector = collision::DARTCollisionDetector::create();

  auto planeFrame
      = dynamics::SimpleFrame::createShared(dynamics::Frame::World());
  planeFrame->setShape(
      std::make_shared<dynamics::PlaneShape>(Eigen::Vector3d::UnitZ(), 0.0));

  std::vector<dynamics::SimpleFramePtr> sphereFrames;
  sphereFrames.reserve(kNumSpheres);
  for (std::size_t i = 0u; i < kNumSpheres; ++i) {
    auto sphereFrame
        = dynamics::SimpleFrame::createShared(dynamics::Frame::World());
    sphereFrame->setShape(std::make_shared<dynamics::SphereShape>(0.5));
    sphereFrame->setTranslation(Eigen::Vector3d(1.5 * i, 0.0, 0.49));
    sphereFrames.push_back(sphereFrame);
  }

  auto group = detector->createCollisionGroup();
  group->addShapeFrame(planeFrame.get());
  for (const auto& sphereFrame : sphereFrames)
    group->addShapeFrame(sphereFrame.get());

  collision::CollisionOption option;
  option.enableContact = true;
  option.maxNumContacts = kNumSpheres;

  detector->setNumCollisionThreads(1u);
  collision::CollisionResult serialResult;
  ASSERT_TRUE(group->collide(option, &serialResult));
  ASSERT_EQ(kNumSpheres, serialResult.getNumContacts());

  detector->setNumCollisionThreads(4u);
  collision::CollisionResult parallelResult;
  ASSERT_TRUE(group->collide(option, &parallelResult));
  ASSERT_EQ(serialResult.getNumContacts(), parallelResult.getNumContacts());

  for (std::size_t i = 0u; i < serialResult.getNumContacts(); ++i) {
    SCOPED_TRACE(i);
    const auto& serialContact = serialResult.getContact(i);
    const auto& parallelContact = parallelResult.getContact(i);
    EXPECT_EQ(
        serialContact.collisionObject1->getShapeFrame(),
        parallelContact.collisionObject1->getShapeFrame());
    EXPECT_EQ(
        serialContact.collisionObject2->getShapeFrame(),
        parallelContact.collisionObject2->getShapeFrame());
    EXPECT_TRUE(serialContact.point.isApprox(parallelContact.point, 1e-12));
    EXPECT_TRUE(serialContact.normal.isApprox(parallelContact.normal, 1e-12));
    EXPECT_NEAR(
        serialContact.penetrationDepth,
        parallelContact.penetrationDepth,
        1e-12);
  }
}
