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

#include <dart/all.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <sstream>

using namespace dart;
using namespace dart::dynamics;
using namespace dart::simulation;

namespace {

constexpr double kBoxSize = 0.12;
constexpr double kBoxMass = 8.0;
constexpr double kWeldedOffset = 1.6;
constexpr double kFreeOffset = 0.6;

struct BoxBounceWorld
{
  WorldPtr world;
  BodyNode* leftFree{};
  BodyNode* rightFree{};
};

// Creates the exact four-box setup described in the issue:
// two free boxes between two welded boxes, zero gravity, restitution 1.0.
BoxBounceWorld makeFourBoxBounceWorld(double pitch)
{
  auto world = World::create("issue870_boxes");
  world->setGravity(Eigen::Vector3d::Zero());
  // Use a small time step to keep the symmetric configuration stable.
  world->setTimeStep(0.00025);

  auto makeBoxSkeleton = [&](const std::string& name,
                             bool weld,
                             double xTranslation) -> BodyNode* {
    auto skel = Skeleton::create(name);
    std::pair<Joint*, BodyNode*> pair;

    if (weld) {
      pair = skel->createJointAndBodyNodePair<WeldJoint>();
    } else {
      pair = skel->createJointAndBodyNodePair<FreeJoint>();
    }

    auto* joint = pair.first;
    auto* body = pair.second;

    auto shape
        = std::make_shared<BoxShape>(Eigen::Vector3d::Constant(kBoxSize));
    auto* shapeNode = body->createShapeNodeWith<
        VisualAspect,
        CollisionAspect,
        DynamicsAspect>(shape);
    // Use elastic contact as in the original regression reproducer.
    shapeNode->getDynamicsAspect()->setRestitutionCoeff(1.0);
    shapeNode->getDynamicsAspect()->setFrictionCoeff(0.0);

    Inertia inertia;
    inertia.setMass(kBoxMass);
    inertia.setMoment(shape->computeInertia(kBoxMass));
    body->setInertia(inertia);

    Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
    tf.translation() = Eigen::Vector3d(xTranslation, 0.1, 0.0);
    tf.linear()
        = Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()).toRotationMatrix();

    if (auto* weldJoint = dynamic_cast<WeldJoint*>(joint)) {
      weldJoint->setTransformFromParentBodyNode(tf);
    } else {
      joint->setPositions(FreeJoint::convertToPositions(tf));
    }

    if (!weld) {
      auto* freeJoint = dynamic_cast<FreeJoint*>(joint);
      DART_ASSERT(freeJoint != nullptr);
      Eigen::Vector6d spatial = Eigen::Vector6d::Zero();
      spatial.tail<3>()
          = Eigen::Vector3d(xTranslation < 0 ? 1.0 : -1.0, 0.0, 0.0);
      freeJoint->setSpatialVelocity(spatial, Frame::World(), Frame::World());
    }

    world->addSkeleton(skel);
    return body;
  };

  BoxBounceWorld result;
  result.world = world;
  makeBoxSkeleton("box3", /*weld=*/true, -kWeldedOffset);
  makeBoxSkeleton("box4", /*weld=*/true, kWeldedOffset);
  result.leftFree = makeBoxSkeleton("box1", /*weld=*/false, -kFreeOffset);
  result.rightFree = makeBoxSkeleton("box2", /*weld=*/false, kFreeOffset);
  return result;
}

WorldPtr makeFreeFallWorld(bool spinning)
{
  auto world = World::create(spinning ? "spinning" : "baseline");
  world->setGravity(Eigen::Vector3d(0.0, -9.81, 0.0));
  world->setTimeStep(0.001);

  SkeletonPtr sphere
      = Skeleton::create(spinning ? "spinning_sphere" : "baseline_sphere");
  auto pair = sphere->createJointAndBodyNodePair<FreeJoint>();
  auto* joint = pair.first;
  auto* body = pair.second;

  auto shape = std::make_shared<EllipsoidShape>(Eigen::Vector3d::Constant(0.2));
  auto* shapeNode = body->createShapeNodeWith<
      VisualAspect,
      CollisionAspect,
      DynamicsAspect>(shape);
  shapeNode->getDynamicsAspect()->setFrictionCoeff(0.0);

  const double mass = 2.0;
  Inertia inertia;
  inertia.setMass(mass);
  inertia.setMoment(shape->computeInertia(mass));
  body->setInertia(inertia);

  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = Eigen::Vector3d(0.0, 1.0, 0.0);
  joint->setPositions(FreeJoint::convertToPositions(tf));

  if (spinning) {
    Eigen::Vector6d velocities = Eigen::Vector6d::Zero();
    velocities.head<3>() = Eigen::Vector3d(0.0, 0.0, 10.0);
    joint->setVelocities(velocities);
  }

  world->addSkeleton(sphere);
  return world;
}

} // namespace

// Regression for https://github.com/dartsim/dart/issues/870 (box bounce
// symmetry): rotating every box by pi/2 should not affect the symmetric
// translation of the free boxes or allow them to bypass the welded boxes.
TEST(Issue870, RotatedBoxesRemainSymmetricBetweenWeldedStops)
{
  BoxBounceWorld baseline = makeFourBoxBounceWorld(0.0);
  BoxBounceWorld rotated = makeFourBoxBounceWorld(dart::math::pi / 2.0);

  const double barrier = kWeldedOffset;
  const double softLimit = barrier + 1e-2; // allow tiny penetration tolerance
  // Run long enough to cover multiple bounces without letting numerical drift
  // dominate the signal.
  // Keep total simulated time roughly consistent with the original test.
  const int steps = 7200;

  double maxSymmetryError = 0.0;
  double maxAbsPosition = 0.0;

  for (int i = 0; i < steps; ++i) {
    baseline.world->step();
    rotated.world->step();

    const auto updateStats = [&](const BoxBounceWorld& w) {
      const auto lp = w.leftFree->getWorldTransform().translation();
      const auto rp = w.rightFree->getWorldTransform().translation();
      const auto lv
          = w.leftFree->getSpatialVelocity(Frame::World(), Frame::World());
      const auto rv
          = w.rightFree->getSpatialVelocity(Frame::World(), Frame::World());

      maxSymmetryError = std::max(
          maxSymmetryError,
          std::abs((lp + rp).x()) + std::abs((lv + rv).tail<3>().x()));

      maxAbsPosition
          = std::max({maxAbsPosition, std::abs(lp.x()), std::abs(rp.x())});
    };

    updateStats(baseline);
    updateStats(rotated);
  }

  std::ostringstream oss;
  oss << "Issue #870 metrics: maxSymmetryError=" << maxSymmetryError
      << ", maxAbsPosition=" << maxAbsPosition;
  SCOPED_TRACE(oss.str());

  EXPECT_LT(maxSymmetryError, 1e-4);
  EXPECT_LT(maxAbsPosition, softLimit);
}

// Regression for https://github.com/dartsim/dart/issues/870: A spinning body in
// free fall should not pick up lateral translation.
TEST(Issue870, SpinningSphereFreeFallDoesNotDriftSideways)
{
  auto baselineWorld = makeFreeFallWorld(false);
  auto spinningWorld = makeFreeFallWorld(true);

  const int steps = 4000;
  double maxHorizontalSeparation = 0.0;

  for (int i = 0; i < steps; ++i) {
    baselineWorld->step();
    spinningWorld->step();

    const auto* baselineBody = baselineWorld->getSkeleton(0)->getRootBodyNode();
    const auto* spinningBody = spinningWorld->getSkeleton(0)->getRootBodyNode();

    const Eigen::Vector3d baselinePos
        = baselineBody->getWorldTransform().translation();
    const Eigen::Vector3d spinningPos
        = spinningBody->getWorldTransform().translation();

    const Eigen::Vector2d horizDiff(
        spinningPos.x() - baselinePos.x(), spinningPos.z() - baselinePos.z());
    maxHorizontalSeparation
        = std::max(maxHorizontalSeparation, horizDiff.norm());
  }

  EXPECT_LT(maxHorizontalSeparation, 1e-7);
}
