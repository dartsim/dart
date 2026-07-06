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
#include <dart/collision/dart/DARTCollide.hpp>
#include <dart/collision/dart/DARTCollisionDetector.hpp>

#include <dart/dynamics/BoxShape.hpp>
#include <dart/dynamics/EllipsoidShape.hpp>
#include <dart/dynamics/FreeJoint.hpp>
#include <dart/dynamics/PlaneShape.hpp>
#include <dart/dynamics/ShapeNode.hpp>
#include <dart/dynamics/SimpleFrame.hpp>
#include <dart/dynamics/Skeleton.hpp>
#include <dart/dynamics/SoftBodyNode.hpp>
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

class TestCollisionObject final : public collision::CollisionObject
{
public:
  TestCollisionObject(
      collision::CollisionDetector* collisionDetector,
      const dynamics::ShapeFrame* shapeFrame)
    : collision::CollisionObject(collisionDetector, shapeFrame)
  {
  }

private:
  void updateEngineData() override {}
};

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

struct PlaneSoftMeshGroups
{
  collision::CollisionDetectorPtr detector;
  dynamics::SimpleFramePtr planeFrame;
  dynamics::SkeletonPtr softSkeleton;
  dynamics::SoftBodyNode* softBody{nullptr};
  dynamics::ShapeNode* softShapeNode{nullptr};
  std::unique_ptr<collision::CollisionGroup> group;
  std::unique_ptr<collision::CollisionGroup> planeGroup;
  std::unique_ptr<collision::CollisionGroup> softGroup;
};

PlaneSoftMeshGroups makePlaneSoftMeshGroups(double softCenterZ)
{
  PlaneSoftMeshGroups groups;
  groups.detector = collision::DARTCollisionDetector::create();

  groups.planeFrame
      = dynamics::SimpleFrame::createShared(dynamics::Frame::World());
  groups.planeFrame->setShape(
      std::make_shared<dynamics::PlaneShape>(Eigen::Vector3d::UnitZ(), 0.0));

  groups.softSkeleton = dynamics::Skeleton::create("soft");
  const auto softProperties = dynamics::SoftBodyNodeHelper::makeBoxProperties(
      Eigen::Vector3d::Ones(), Eigen::Isometry3d::Identity(), 1.0);
  const dynamics::BodyNode::Properties bodyProperties(
      dynamics::BodyNode::AspectProperties("soft_body"));
  const dynamics::SoftBodyNode::Properties softBodyProperties(
      bodyProperties, softProperties);

  auto pair = groups.softSkeleton->createJointAndBodyNodePair<
      dynamics::FreeJoint,
      dynamics::SoftBodyNode>(
      nullptr, dynamics::FreeJoint::Properties(), softBodyProperties);
  groups.softBody = pair.second;

  Eigen::Isometry3d softTransform = Eigen::Isometry3d::Identity();
  softTransform.translation().z() = softCenterZ;
  pair.first->setPositions(
      dynamics::FreeJoint::convertToPositions(softTransform));

  groups.softShapeNode
      = groups.softBody->getShapeNodeWith<dynamics::CollisionAspect>(0);
  groups.group = groups.detector->createCollisionGroup(
      groups.planeFrame.get(), groups.softShapeNode);
  groups.planeGroup
      = groups.detector->createCollisionGroup(groups.planeFrame.get());
  groups.softGroup
      = groups.detector->createCollisionGroup(groups.softShapeNode);

  return groups;
}

struct BoxSoftMeshGroups
{
  collision::CollisionDetectorPtr detector;
  dynamics::SimpleFramePtr boxFrame;
  dynamics::SkeletonPtr softSkeleton;
  dynamics::SoftBodyNode* softBody{nullptr};
  dynamics::ShapeNode* softShapeNode{nullptr};
  std::unique_ptr<collision::CollisionGroup> boxGroup;
  std::unique_ptr<collision::CollisionGroup> softGroup;
};

BoxSoftMeshGroups makeBoxSoftMeshGroups(double softCenterZ)
{
  BoxSoftMeshGroups groups;
  groups.detector = collision::DARTCollisionDetector::create();

  groups.boxFrame
      = dynamics::SimpleFrame::createShared(dynamics::Frame::World());
  groups.boxFrame->setShape(
      std::make_shared<dynamics::BoxShape>(Eigen::Vector3d(10.0, 10.0, 1.0)));
  groups.boxFrame->setTranslation(Eigen::Vector3d(0.0, 0.0, -0.5));

  groups.softSkeleton = dynamics::Skeleton::create("soft_box");
  const auto softProperties = dynamics::SoftBodyNodeHelper::makeBoxProperties(
      Eigen::Vector3d::Ones(), Eigen::Isometry3d::Identity(), 1.0);
  const dynamics::BodyNode::Properties bodyProperties(
      dynamics::BodyNode::AspectProperties("soft_box_body"));
  const dynamics::SoftBodyNode::Properties softBodyProperties(
      bodyProperties, softProperties);

  auto pair = groups.softSkeleton->createJointAndBodyNodePair<
      dynamics::FreeJoint,
      dynamics::SoftBodyNode>(
      nullptr, dynamics::FreeJoint::Properties(), softBodyProperties);
  groups.softBody = pair.second;

  Eigen::Isometry3d softTransform = Eigen::Isometry3d::Identity();
  softTransform.translation().z() = softCenterZ;
  pair.first->setPositions(
      dynamics::FreeJoint::convertToPositions(softTransform));

  groups.softShapeNode
      = groups.softBody->getShapeNodeWith<dynamics::CollisionAspect>(0);
  groups.boxGroup
      = groups.detector->createCollisionGroup(groups.boxFrame.get());
  groups.softGroup
      = groups.detector->createCollisionGroup(groups.softShapeNode);

  return groups;
}

struct SphereSoftMeshGroups
{
  collision::CollisionDetectorPtr detector;
  dynamics::SimpleFramePtr sphereFrame;
  dynamics::SkeletonPtr softSkeleton;
  dynamics::SoftBodyNode* softBody{nullptr};
  dynamics::ShapeNode* softShapeNode{nullptr};
  std::unique_ptr<collision::CollisionGroup> sphereGroup;
  std::unique_ptr<collision::CollisionGroup> softGroup;
};

SphereSoftMeshGroups makeSphereSoftMeshGroups(bool sphereLikeEllipsoid)
{
  SphereSoftMeshGroups groups;
  groups.detector = collision::DARTCollisionDetector::create();

  groups.sphereFrame
      = dynamics::SimpleFrame::createShared(dynamics::Frame::World());
  if (sphereLikeEllipsoid) {
    groups.sphereFrame->setShape(std::make_shared<dynamics::EllipsoidShape>(
        Eigen::Vector3d::Constant(0.4)));
  } else {
    groups.sphereFrame->setShape(std::make_shared<dynamics::SphereShape>(0.2));
  }
  groups.sphereFrame->setTranslation(Eigen::Vector3d(-0.5, -0.5, -0.15));

  groups.softSkeleton = dynamics::Skeleton::create("soft_sphere");
  const auto softProperties = dynamics::SoftBodyNodeHelper::makeBoxProperties(
      Eigen::Vector3d::Ones(), Eigen::Isometry3d::Identity(), 1.0);
  const dynamics::BodyNode::Properties bodyProperties(
      dynamics::BodyNode::AspectProperties("soft_sphere_body"));
  const dynamics::SoftBodyNode::Properties softBodyProperties(
      bodyProperties, softProperties);

  auto pair = groups.softSkeleton->createJointAndBodyNodePair<
      dynamics::FreeJoint,
      dynamics::SoftBodyNode>(
      nullptr, dynamics::FreeJoint::Properties(), softBodyProperties);
  groups.softBody = pair.second;

  Eigen::Isometry3d softTransform = Eigen::Isometry3d::Identity();
  softTransform.translation().z() = 0.45;
  pair.first->setPositions(
      dynamics::FreeJoint::convertToPositions(softTransform));

  groups.softShapeNode
      = groups.softBody->getShapeNodeWith<dynamics::CollisionAspect>(0);
  groups.sphereGroup
      = groups.detector->createCollisionGroup(groups.sphereFrame.get());
  groups.softGroup
      = groups.detector->createCollisionGroup(groups.softShapeNode);

  return groups;
}

SphereSoftMeshGroups makeEllipsoidSoftMeshGroups()
{
  SphereSoftMeshGroups groups;
  groups.detector = collision::DARTCollisionDetector::create();

  groups.sphereFrame
      = dynamics::SimpleFrame::createShared(dynamics::Frame::World());
  groups.sphereFrame->setShape(std::make_shared<dynamics::EllipsoidShape>(
      Eigen::Vector3d(0.30, 0.40, 0.60)));
  groups.sphereFrame->setTranslation(Eigen::Vector3d(-0.5, -0.5, -0.25));

  groups.softSkeleton = dynamics::Skeleton::create("soft_ellipsoid");
  const auto softProperties = dynamics::SoftBodyNodeHelper::makeBoxProperties(
      Eigen::Vector3d::Ones(), Eigen::Isometry3d::Identity(), 1.0);
  const dynamics::BodyNode::Properties bodyProperties(
      dynamics::BodyNode::AspectProperties("soft_ellipsoid_body"));
  const dynamics::SoftBodyNode::Properties softBodyProperties(
      bodyProperties, softProperties);

  auto pair = groups.softSkeleton->createJointAndBodyNodePair<
      dynamics::FreeJoint,
      dynamics::SoftBodyNode>(
      nullptr, dynamics::FreeJoint::Properties(), softBodyProperties);
  groups.softBody = pair.second;

  Eigen::Isometry3d softTransform = Eigen::Isometry3d::Identity();
  softTransform.translation().z() = 0.45;
  pair.first->setPositions(
      dynamics::FreeJoint::convertToPositions(softTransform));

  groups.softShapeNode
      = groups.softBody->getShapeNodeWith<dynamics::CollisionAspect>(0);
  groups.sphereGroup
      = groups.detector->createCollisionGroup(groups.sphereFrame.get());
  groups.softGroup
      = groups.detector->createCollisionGroup(groups.softShapeNode);

  return groups;
}

struct SoftSoftMeshGroups
{
  collision::CollisionDetectorPtr detector;
  dynamics::SkeletonPtr softSkeleton1;
  dynamics::SkeletonPtr softSkeleton2;
  dynamics::SoftBodyNode* softBody1{nullptr};
  dynamics::SoftBodyNode* softBody2{nullptr};
  dynamics::ShapeNode* softShapeNode1{nullptr};
  dynamics::ShapeNode* softShapeNode2{nullptr};
  std::unique_ptr<collision::CollisionGroup> softGroup1;
  std::unique_ptr<collision::CollisionGroup> softGroup2;
};

std::pair<dynamics::SoftBodyNode*, dynamics::ShapeNode*> addSoftBox(
    const dynamics::SkeletonPtr& skeleton,
    const std::string& name,
    double centerZ)
{
  const auto softProperties = dynamics::SoftBodyNodeHelper::makeBoxProperties(
      Eigen::Vector3d::Ones(), Eigen::Isometry3d::Identity(), 1.0);
  const dynamics::BodyNode::Properties bodyProperties{
      dynamics::BodyNode::AspectProperties(name)};
  const dynamics::SoftBodyNode::Properties softBodyProperties(
      bodyProperties, softProperties);

  auto pair = skeleton->createJointAndBodyNodePair<
      dynamics::FreeJoint,
      dynamics::SoftBodyNode>(
      nullptr, dynamics::FreeJoint::Properties(), softBodyProperties);

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation().z() = centerZ;
  pair.first->setPositions(dynamics::FreeJoint::convertToPositions(transform));

  return {
      pair.second, pair.second->getShapeNodeWith<dynamics::CollisionAspect>(0)};
}

SoftSoftMeshGroups makeSoftSoftMeshGroups()
{
  SoftSoftMeshGroups groups;
  groups.detector = collision::DARTCollisionDetector::create();

  groups.softSkeleton1 = dynamics::Skeleton::create("soft_soft_1");
  groups.softSkeleton2 = dynamics::Skeleton::create("soft_soft_2");

  auto soft1 = addSoftBox(groups.softSkeleton1, "soft_soft_body_1", 0.0);
  auto soft2 = addSoftBox(groups.softSkeleton2, "soft_soft_body_2", 0.95);
  groups.softBody1 = soft1.first;
  groups.softShapeNode1 = soft1.second;
  groups.softBody2 = soft2.first;
  groups.softShapeNode2 = soft2.second;

  groups.softGroup1
      = groups.detector->createCollisionGroup(groups.softShapeNode1);
  groups.softGroup2
      = groups.detector->createCollisionGroup(groups.softShapeNode2);

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

//==============================================================================
TEST(DARTCollisionDetector, DetectsNativeSoftMeshPlaneContact)
{
  auto groups = makePlaneSoftMeshGroups(0.45);
  ASSERT_NE(nullptr, groups.softBody);
  ASSERT_NE(nullptr, groups.softShapeNode);

  collision::CollisionOption option;
  option.enableContact = true;
  option.maxNumContacts = 10u;

  collision::CollisionResult result;
  EXPECT_TRUE(groups.group->collide(option, &result));
  ASSERT_GT(result.getNumContacts(), 0u);

  bool sawSoftContact = false;
  for (std::size_t i = 0u; i < result.getNumContacts(); ++i) {
    const auto& contact = result.getContact(i);
    if (contact.collisionObject2->getShapeFrame() != groups.softShapeNode)
      continue;

    sawSoftContact = true;
    EXPECT_EQ(
        groups.planeFrame.get(), contact.collisionObject1->getShapeFrame());
    EXPECT_TRUE(contact.normal.isApprox(-Eigen::Vector3d::UnitZ(), 1e-12));
    EXPECT_GE(contact.penetrationDepth, 0.0);
    EXPECT_GE(contact.triID2, 0);
    EXPECT_LT(
        static_cast<std::size_t>(contact.triID2),
        groups.softBody->getNumFaces());
  }

  EXPECT_TRUE(sawSoftContact);
}

//==============================================================================
TEST(DARTCollisionDetector, KeepsNativeSoftPlaneContactPointOnCollidingVertex)
{
  auto groups = makePlaneSoftMeshGroups(0.45);
  ASSERT_NE(nullptr, groups.planeGroup);
  ASSERT_NE(nullptr, groups.softGroup);

  collision::CollisionOption option;
  option.enableContact = true;
  option.maxNumContacts = 10u;

  collision::CollisionResult primitiveFirstResult;
  EXPECT_TRUE(groups.planeGroup->collide(
      groups.softGroup.get(), option, &primitiveFirstResult));
  ASSERT_GT(primitiveFirstResult.getNumContacts(), 0u);

  bool sawPrimitiveFirstContact = false;
  for (std::size_t i = 0u; i < primitiveFirstResult.getNumContacts(); ++i) {
    const auto& contact = primitiveFirstResult.getContact(i);
    if (contact.collisionObject2->getShapeFrame() != groups.softShapeNode)
      continue;

    sawPrimitiveFirstContact = true;
    EXPECT_GT(contact.penetrationDepth, 0.0);
    EXPECT_NEAR(contact.point.z(), -contact.penetrationDepth, 1e-12);
  }
  EXPECT_TRUE(sawPrimitiveFirstContact);

  collision::CollisionResult softFirstResult;
  EXPECT_TRUE(groups.softGroup->collide(
      groups.planeGroup.get(), option, &softFirstResult));
  ASSERT_GT(softFirstResult.getNumContacts(), 0u);

  bool sawSoftFirstContact = false;
  for (std::size_t i = 0u; i < softFirstResult.getNumContacts(); ++i) {
    const auto& contact = softFirstResult.getContact(i);
    if (contact.collisionObject1->getShapeFrame() != groups.softShapeNode)
      continue;

    sawSoftFirstContact = true;
    EXPECT_GT(contact.penetrationDepth, 0.0);
    EXPECT_NEAR(contact.point.z(), -contact.penetrationDepth, 1e-12);
  }
  EXPECT_TRUE(sawSoftFirstContact);
}

//==============================================================================
TEST(DARTCollisionDetector, PublicSoftMeshCollideFallsBackForNonDartObjects)
{
  auto groups = makePlaneSoftMeshGroups(0.45);
  ASSERT_NE(nullptr, groups.softShapeNode);

  TestCollisionObject planeObject(
      groups.detector.get(), groups.planeFrame.get());
  TestCollisionObject softObject(groups.detector.get(), groups.softShapeNode);

  collision::CollisionResult primitiveFirstResult;
  EXPECT_GT(
      collision::collide(&planeObject, &softObject, primitiveFirstResult), 0);
  EXPECT_GT(primitiveFirstResult.getNumContacts(), 0u);

  collision::CollisionResult softFirstResult;
  EXPECT_GT(collision::collide(&softObject, &planeObject, softFirstResult), 0);
  EXPECT_GT(softFirstResult.getNumContacts(), 0u);
}

//==============================================================================
TEST(DARTCollisionDetector, PublicSoftSoftCollideIgnoresNonDartObjects)
{
  auto groups = makeSoftSoftMeshGroups();
  ASSERT_NE(nullptr, groups.softShapeNode1);
  ASSERT_NE(nullptr, groups.softShapeNode2);

  TestCollisionObject softObject1(groups.detector.get(), groups.softShapeNode1);
  TestCollisionObject softObject2(groups.detector.get(), groups.softShapeNode2);

  collision::CollisionResult result;
  EXPECT_EQ(collision::collide(&softObject1, &softObject2, result), 0);
  EXPECT_EQ(result.getNumContacts(), 0u);
}

//==============================================================================
TEST(DARTCollisionDetector, DetectsNativeSoftMeshBoxContact)
{
  auto groups = makeBoxSoftMeshGroups(0.45);
  ASSERT_NE(nullptr, groups.softBody);
  ASSERT_NE(nullptr, groups.softShapeNode);

  collision::CollisionOption option;
  option.enableContact = true;
  option.maxNumContacts = 10u;

  collision::CollisionResult result;
  EXPECT_TRUE(
      groups.boxGroup->collide(groups.softGroup.get(), option, &result));
  ASSERT_GT(result.getNumContacts(), 0u);

  bool sawSoftContact = false;
  for (std::size_t i = 0u; i < result.getNumContacts(); ++i) {
    const auto& contact = result.getContact(i);
    if (contact.collisionObject2->getShapeFrame() != groups.softShapeNode)
      continue;

    sawSoftContact = true;
    EXPECT_EQ(groups.boxFrame.get(), contact.collisionObject1->getShapeFrame());
    EXPECT_TRUE(contact.normal.isApprox(-Eigen::Vector3d::UnitZ(), 1e-12));
    EXPECT_GT(contact.penetrationDepth, 0.0);
    EXPECT_GE(contact.triID2, 0);
    EXPECT_LT(
        static_cast<std::size_t>(contact.triID2),
        groups.softBody->getNumFaces());
  }

  EXPECT_TRUE(sawSoftContact);
}

//==============================================================================
TEST(DARTCollisionDetector, DetectsNativeSoftMeshSphereContact)
{
  auto groups = makeSphereSoftMeshGroups(false);
  ASSERT_NE(nullptr, groups.softBody);
  ASSERT_NE(nullptr, groups.softShapeNode);

  collision::CollisionOption option;
  option.enableContact = true;
  option.maxNumContacts = 10u;

  collision::CollisionResult result;
  EXPECT_TRUE(
      groups.sphereGroup->collide(groups.softGroup.get(), option, &result));
  ASSERT_GT(result.getNumContacts(), 0u);

  bool sawSoftContact = false;
  for (std::size_t i = 0u; i < result.getNumContacts(); ++i) {
    const auto& contact = result.getContact(i);
    if (contact.collisionObject2->getShapeFrame() != groups.softShapeNode)
      continue;

    sawSoftContact = true;
    EXPECT_EQ(
        groups.sphereFrame.get(), contact.collisionObject1->getShapeFrame());
    EXPECT_TRUE(contact.normal.isApprox(-Eigen::Vector3d::UnitZ(), 1e-12));
    EXPECT_NEAR(contact.penetrationDepth, 0.1, 1e-12);
    EXPECT_GE(contact.triID2, 0);
    EXPECT_LT(
        static_cast<std::size_t>(contact.triID2),
        groups.softBody->getNumFaces());
  }

  EXPECT_TRUE(sawSoftContact);
}

//==============================================================================
TEST(DARTCollisionDetector, DetectsNativeSoftMeshSphereLikeEllipsoidContact)
{
  auto groups = makeSphereSoftMeshGroups(true);
  ASSERT_NE(nullptr, groups.softBody);
  ASSERT_NE(nullptr, groups.softShapeNode);

  collision::CollisionOption option;
  option.enableContact = true;
  option.maxNumContacts = 10u;

  collision::CollisionResult primitiveFirstResult;
  EXPECT_TRUE(groups.sphereGroup->collide(
      groups.softGroup.get(), option, &primitiveFirstResult));
  ASSERT_GT(primitiveFirstResult.getNumContacts(), 0u);

  bool sawPrimitiveFirstSoftContact = false;
  for (std::size_t i = 0u; i < primitiveFirstResult.getNumContacts(); ++i) {
    const auto& contact = primitiveFirstResult.getContact(i);
    if (contact.collisionObject2->getShapeFrame() != groups.softShapeNode)
      continue;

    sawPrimitiveFirstSoftContact = true;
    EXPECT_EQ(
        groups.sphereFrame.get(), contact.collisionObject1->getShapeFrame());
    EXPECT_TRUE(contact.normal.isApprox(-Eigen::Vector3d::UnitZ(), 1e-12));
    EXPECT_NEAR(contact.penetrationDepth, 0.1, 1e-12);
    EXPECT_GE(contact.triID2, 0);
    EXPECT_LT(
        static_cast<std::size_t>(contact.triID2),
        groups.softBody->getNumFaces());
  }
  EXPECT_TRUE(sawPrimitiveFirstSoftContact);

  collision::CollisionResult softFirstResult;
  EXPECT_TRUE(groups.softGroup->collide(
      groups.sphereGroup.get(), option, &softFirstResult));
  ASSERT_GT(softFirstResult.getNumContacts(), 0u);

  bool sawSoftFirstContact = false;
  for (std::size_t i = 0u; i < softFirstResult.getNumContacts(); ++i) {
    const auto& contact = softFirstResult.getContact(i);
    if (contact.collisionObject1->getShapeFrame() != groups.softShapeNode)
      continue;

    sawSoftFirstContact = true;
    EXPECT_EQ(
        groups.sphereFrame.get(), contact.collisionObject2->getShapeFrame());
    EXPECT_TRUE(contact.normal.isApprox(Eigen::Vector3d::UnitZ(), 1e-12));
    EXPECT_NEAR(contact.penetrationDepth, 0.1, 1e-12);
    EXPECT_GE(contact.triID1, 0);
    EXPECT_LT(
        static_cast<std::size_t>(contact.triID1),
        groups.softBody->getNumFaces());
  }
  EXPECT_TRUE(sawSoftFirstContact);
}

//==============================================================================
TEST(DARTCollisionDetector, DetectsNativeSoftMeshEllipsoidContact)
{
  auto groups = makeEllipsoidSoftMeshGroups();
  ASSERT_NE(nullptr, groups.softBody);
  ASSERT_NE(nullptr, groups.softShapeNode);

  const auto* ellipsoid = static_cast<const dynamics::EllipsoidShape*>(
      groups.sphereFrame->getShape().get());
  ASSERT_NE(nullptr, ellipsoid);
  ASSERT_FALSE(ellipsoid->isSphere());

  collision::CollisionOption option;
  option.enableContact = true;
  option.maxNumContacts = 10u;

  collision::CollisionResult primitiveFirstResult;
  EXPECT_TRUE(groups.sphereGroup->collide(
      groups.softGroup.get(), option, &primitiveFirstResult));
  ASSERT_GT(primitiveFirstResult.getNumContacts(), 0u);

  bool sawPrimitiveFirstSoftContact = false;
  for (std::size_t i = 0u; i < primitiveFirstResult.getNumContacts(); ++i) {
    const auto& contact = primitiveFirstResult.getContact(i);
    if (contact.collisionObject2->getShapeFrame() != groups.softShapeNode)
      continue;

    sawPrimitiveFirstSoftContact = true;
    EXPECT_EQ(
        groups.sphereFrame.get(), contact.collisionObject1->getShapeFrame());
    EXPECT_TRUE(contact.normal.isApprox(-Eigen::Vector3d::UnitZ(), 1e-12));
    EXPECT_NEAR(contact.penetrationDepth, 0.1, 1e-12);
    EXPECT_GE(contact.triID2, 0);
    EXPECT_LT(
        static_cast<std::size_t>(contact.triID2),
        groups.softBody->getNumFaces());
  }
  EXPECT_TRUE(sawPrimitiveFirstSoftContact);

  collision::CollisionResult softFirstResult;
  EXPECT_TRUE(groups.softGroup->collide(
      groups.sphereGroup.get(), option, &softFirstResult));
  ASSERT_GT(softFirstResult.getNumContacts(), 0u);

  bool sawSoftFirstContact = false;
  for (std::size_t i = 0u; i < softFirstResult.getNumContacts(); ++i) {
    const auto& contact = softFirstResult.getContact(i);
    if (contact.collisionObject1->getShapeFrame() != groups.softShapeNode)
      continue;

    sawSoftFirstContact = true;
    EXPECT_EQ(
        groups.sphereFrame.get(), contact.collisionObject2->getShapeFrame());
    EXPECT_TRUE(contact.normal.isApprox(Eigen::Vector3d::UnitZ(), 1e-12));
    EXPECT_NEAR(contact.penetrationDepth, 0.1, 1e-12);
    EXPECT_GE(contact.triID1, 0);
    EXPECT_LT(
        static_cast<std::size_t>(contact.triID1),
        groups.softBody->getNumFaces());
  }
  EXPECT_TRUE(sawSoftFirstContact);
}

//==============================================================================
TEST(DARTCollisionDetector, DetectsNativeSoftMeshSoftMeshContact)
{
  auto groups = makeSoftSoftMeshGroups();
  ASSERT_NE(nullptr, groups.softBody1);
  ASSERT_NE(nullptr, groups.softBody2);
  ASSERT_NE(nullptr, groups.softShapeNode1);
  ASSERT_NE(nullptr, groups.softShapeNode2);

  collision::CollisionOption option;
  option.enableContact = true;
  option.maxNumContacts = 32u;

  collision::CollisionResult result;
  EXPECT_TRUE(
      groups.softGroup1->collide(groups.softGroup2.get(), option, &result));
  ASSERT_GT(result.getNumContacts(), 0u);

  bool sawSoftSoftContact = false;
  bool sawExpectedSeparatingContact = false;
  for (std::size_t i = 0u; i < result.getNumContacts(); ++i) {
    const auto& contact = result.getContact(i);
    if (contact.collisionObject1->getShapeFrame() != groups.softShapeNode1
        || contact.collisionObject2->getShapeFrame() != groups.softShapeNode2) {
      continue;
    }

    sawSoftSoftContact = true;
    EXPECT_FALSE(collision::Contact::isZeroNormal(contact.normal));
    EXPECT_GE(contact.penetrationDepth, 0.0);
    EXPECT_GE(contact.triID1, 0);
    EXPECT_LT(
        static_cast<std::size_t>(contact.triID1),
        groups.softBody1->getNumFaces());
    EXPECT_GE(contact.triID2, 0);
    EXPECT_LT(
        static_cast<std::size_t>(contact.triID2),
        groups.softBody2->getNumFaces());

    if (contact.normal.isApprox(-Eigen::Vector3d::UnitZ(), 1e-12)
        && contact.penetrationDepth > 0.0) {
      sawExpectedSeparatingContact = true;
    }
  }

  EXPECT_TRUE(sawSoftSoftContact);
  EXPECT_TRUE(sawExpectedSeparatingContact);
}
