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

#include <dart/collision/CollisionFilter.hpp>
#include <dart/collision/CollisionObject.hpp>
#include <dart/collision/dart/DARTCollisionDetector.hpp>

#include <dart/dynamics/PlaneShape.hpp>
#include <dart/dynamics/SimpleFrame.hpp>
#include <dart/dynamics/SphereShape.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <utility>
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

class AnchorOnlyCollisionFilter : public collision::CollisionFilter
{
public:
  explicit AnchorOnlyCollisionFilter(const dynamics::ShapeFrame* anchor)
    : mAnchor(anchor)
  {
  }

  bool ignoresCollision(
      const collision::CollisionObject* object1,
      const collision::CollisionObject* object2) const override
  {
    return object1->getShapeFrame() != mAnchor
           && object2->getShapeFrame() != mAnchor;
  }

private:
  const dynamics::ShapeFrame* mAnchor;
};

class CountingCollisionFilter : public collision::CollisionFilter
{
public:
  bool ignoresCollision(
      const collision::CollisionObject*,
      const collision::CollisionObject*) const override
  {
    ++mNumChecks;
    return false;
  }

  std::size_t getNumChecks() const
  {
    return mNumChecks;
  }

private:
  mutable std::size_t mNumChecks = 0u;
};

} // namespace

// NOTE: the legacy detector's grid-hash contact deduplication (which merged
// contacts from distinct collision objects that landed in the same grid cell)
// was retired with the engine consolidation; per-pair manifold reduction
// supersedes it, and cross-object contacts are intentionally kept distinct.
// The former DeduplicatesPlaneContactsAcrossGridCellBoundary test asserted
// that quirk and was removed with the mechanism.

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
TEST(DARTCollisionDetector, DetectsTranslatedIdentityBounds)
{
  auto detector = collision::DARTCollisionDetector::create();

  auto sphereFrame1
      = dynamics::SimpleFrame::createShared(dynamics::Frame::World());
  auto sphereFrame2
      = dynamics::SimpleFrame::createShared(dynamics::Frame::World());
  auto sphereFrame3
      = dynamics::SimpleFrame::createShared(dynamics::Frame::World());

  sphereFrame1->setShape(std::make_shared<dynamics::SphereShape>(0.5));
  sphereFrame2->setShape(std::make_shared<dynamics::SphereShape>(0.5));
  sphereFrame3->setShape(std::make_shared<dynamics::SphereShape>(0.5));

  sphereFrame1->setTranslation(Eigen::Vector3d(10.0, 20.0, 30.0));
  sphereFrame2->setTranslation(Eigen::Vector3d(10.9, 20.0, 30.0));
  sphereFrame3->setTranslation(Eigen::Vector3d(30.0, 20.0, 30.0));

  auto group = detector->createCollisionGroup(
      sphereFrame1.get(), sphereFrame2.get(), sphereFrame3.get());

  collision::CollisionOption option;
  option.enableContact = true;
  option.maxNumContacts = 10u;

  collision::CollisionResult result;
  EXPECT_TRUE(group->collide(option, &result));
  ASSERT_EQ(1u, result.getNumContacts());

  const auto& contact = result.getContact(0);
  const auto* shapeFrame1 = contact.collisionObject1->getShapeFrame();
  const auto* shapeFrame2 = contact.collisionObject2->getShapeFrame();
  EXPECT_TRUE(
      (shapeFrame1 == sphereFrame1.get() && shapeFrame2 == sphereFrame2.get())
      || (shapeFrame1 == sphereFrame2.get()
          && shapeFrame2 == sphereFrame1.get()));
  EXPECT_GT(contact.penetrationDepth, 0.0);
}

//==============================================================================
TEST(DARTCollisionDetector, BatchedFiniteSweepPreservesCandidateOrder)
{
  auto detector = collision::DARTCollisionDetector::create();

  auto anchorFrame
      = dynamics::SimpleFrame::createShared(dynamics::Frame::World());
  anchorFrame->setShape(std::make_shared<dynamics::SphereShape>(1.0));
  anchorFrame->setTranslation(Eigen::Vector3d::Zero());

  const std::vector<Eigen::Vector3d> candidatePositions{
      {0.1, 0.0, 0.0},
      {0.2, 3.5, 0.0},
      {0.3, -0.25, 0.0},
      {0.4, 0.0, 3.5},
      {0.5, 0.3, 0.4},
      {0.6, -3.5, 0.0},
      {0.7, 0.0, -0.2},
      {0.8, 0.0, 3.5},
  };
  const std::vector<std::size_t> expectedContactCandidates{0u, 2u, 4u, 6u};

  std::vector<dynamics::SimpleFramePtr> candidateFrames;
  candidateFrames.reserve(candidatePositions.size());
  for (const auto& position : candidatePositions) {
    auto frame = dynamics::SimpleFrame::createShared(dynamics::Frame::World());
    frame->setShape(std::make_shared<dynamics::SphereShape>(1.0));
    frame->setTranslation(position);
    candidateFrames.push_back(frame);
  }

  auto anchorGroup = detector->createCollisionGroup(anchorFrame.get());
  auto candidateGroup = detector->createCollisionGroup();
  for (const auto& candidateFrame : candidateFrames)
    candidateGroup->addShapeFrame(candidateFrame.get());

  collision::CollisionOption option;
  option.enableContact = true;
  option.maxNumContacts = 10u;

  collision::CollisionResult result;
  ASSERT_TRUE(anchorGroup->collide(candidateGroup.get(), option, &result));
  ASSERT_EQ(expectedContactCandidates.size(), result.getNumContacts());

  for (std::size_t i = 0u; i < expectedContactCandidates.size(); ++i) {
    SCOPED_TRACE(i);
    const auto& contact = result.getContact(i);
    EXPECT_EQ(anchorFrame.get(), contact.collisionObject1->getShapeFrame());
    EXPECT_EQ(
        candidateFrames[expectedContactCandidates[i]].get(),
        contact.collisionObject2->getShapeFrame());
    EXPECT_GT(contact.penetrationDepth, 0.0);
  }
}

//==============================================================================
TEST(DARTCollisionDetector, BatchedSingleGroupSweepPreservesCandidateOrder)
{
  auto detector = collision::DARTCollisionDetector::create();

  auto anchorFrame
      = dynamics::SimpleFrame::createShared(dynamics::Frame::World());
  anchorFrame->setShape(std::make_shared<dynamics::SphereShape>(1.0));
  anchorFrame->setTranslation(Eigen::Vector3d::Zero());

  const std::vector<Eigen::Vector3d> candidatePositions{
      {0.1, 0.0, 0.0},
      {0.2, 3.5, 0.0},
      {0.3, -0.25, 0.0},
      {0.4, 0.0, 3.5},
      {0.5, 0.3, 0.4},
      {0.6, -3.5, 0.0},
      {0.7, 0.0, -0.2},
      {0.8, 0.0, 3.5},
  };
  const std::vector<std::size_t> expectedContactCandidates{0u, 2u, 4u, 6u};

  std::vector<dynamics::SimpleFramePtr> candidateFrames;
  candidateFrames.reserve(candidatePositions.size());
  for (const auto& position : candidatePositions) {
    auto frame = dynamics::SimpleFrame::createShared(dynamics::Frame::World());
    frame->setShape(std::make_shared<dynamics::SphereShape>(1.0));
    frame->setTranslation(position);
    candidateFrames.push_back(frame);
  }

  auto group = detector->createCollisionGroup();
  group->addShapeFrame(anchorFrame.get());
  for (const auto& candidateFrame : candidateFrames)
    group->addShapeFrame(candidateFrame.get());

  collision::CollisionOption option;
  option.enableContact = true;
  option.maxNumContacts = 10u;
  option.collisionFilter
      = std::make_shared<AnchorOnlyCollisionFilter>(anchorFrame.get());

  collision::CollisionResult result;
  ASSERT_TRUE(group->collide(option, &result));
  ASSERT_EQ(expectedContactCandidates.size(), result.getNumContacts());

  for (std::size_t i = 0u; i < expectedContactCandidates.size(); ++i) {
    SCOPED_TRACE(i);
    const auto& contact = result.getContact(i);
    EXPECT_EQ(anchorFrame.get(), contact.collisionObject1->getShapeFrame());
    EXPECT_EQ(
        candidateFrames[expectedContactCandidates[i]].get(),
        contact.collisionObject2->getShapeFrame());
    EXPECT_GT(contact.penetrationDepth, 0.0);
  }
}

//==============================================================================
TEST(
    DARTCollisionDetector, BinaryFiniteSweepShortCircuitsWithoutCollectingPairs)
{
  constexpr std::size_t kNumSpheres = 24u;

  auto detector = collision::DARTCollisionDetector::create();

  std::vector<dynamics::SimpleFramePtr> sphereFrames;
  sphereFrames.reserve(kNumSpheres);
  for (std::size_t i = 0u; i < kNumSpheres; ++i) {
    auto frame = dynamics::SimpleFrame::createShared(dynamics::Frame::World());
    frame->setShape(std::make_shared<dynamics::SphereShape>(1.0));
    frame->setTranslation(Eigen::Vector3d::Zero());
    sphereFrames.push_back(frame);
  }

  auto group = detector->createCollisionGroup();
  for (const auto& sphereFrame : sphereFrames)
    group->addShapeFrame(sphereFrame.get());

  auto filter = std::make_shared<CountingCollisionFilter>();
  collision::CollisionOption option(false, 1u, filter);

  EXPECT_TRUE(group->collide(option, nullptr));
  EXPECT_EQ(1u, filter->getNumChecks());
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
