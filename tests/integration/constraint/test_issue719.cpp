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

// Regression test for gazebosim/gz-physics#719: "Closed kinematic chains only
// work when joints are aligned with world axes". A non-axis-aligned closed loop
// produces a rank-deficient constraint diagonal. With the previous purely
// multiplicative Constraint Force Mixing, that near-zero diagonal received
// almost no regularization, leaving the boxed LCP singular: the Dantzig solver
// hit a singular pivot and emitted NaN/garbage. The pre-existing NaN fail-safe
// then scrubbed the solution to zero, so the loop-closure constraint applied no
// impulse and the loop silently drifted open (the simulation stayed finite but
// the closed chain was not enforced - exactly the gz symptom).
//
// This fix has two complementary halves:
//   1. An additive CFM floor in the holonomic loop-closure constraints
//      (BallJointConstraint / WeldJointConstraint) regularizes the
//      rank-deficient directions so the Dantzig solver itself converges instead
//      of hitting a singular pivot. It is deliberately NOT applied to the motor
//      / limit / contact constraints, where the constant floor would perturb
//      otherwise well-conditioned solves.
//   2. The Dantzig driver reports failure on a singular pivot so the constraint
//      solver can fall back to the secondary/PGS solver (or its zero fail-safe)
//      instead of propagating a partially-zeroed solution as success.
//
// The default solver (Dantzig primary + PGS secondary) already kept the loop
// closed via the PGS fallback, so PART A guards that invariant. PART B disables
// the secondary so only Dantzig runs: without this fix the loop drifts ~6 mm;
// with it the loop stays closed to ~1e-9. That makes PART B the discriminating
// regression guard.

#include "helpers/gtest_utils.hpp"

#include <dart/all.hpp>

#include <gtest/gtest.h>

#include <numbers>

using namespace dart;
using namespace dart::dynamics;
using namespace dart::simulation;

namespace {

// Maximum tolerated loop-closure error. Without the fix the Dantzig-only case
// drifts ~6e-3 m; with it the loop stays closed to ~2e-9 m, so this threshold
// sits comfortably between the broken and fixed behavior.
constexpr double kMaxLoopClosureError = 1e-3;

//==============================================================================
// Builds a closed four-bar linkage as a single skeleton: a fixed base plus
// three links connected by RevoluteJoints. The entire linkage is rigidly
// rotated ~30 degrees about an arbitrary, non-axis-aligned axis so neither the
// revolute axes nor the loop-closure anchor line up with the world X/Y/Z axes.
// The caller closes the loop with a BallJointConstraint between the last link
// and the base. Returns the skeleton and writes the loop-closure body nodes and
// world-frame anchor through the out parameters.
SkeletonPtr createNonAxisAlignedFourBar(
    BodyNode** baseBodyOut,
    BodyNode** lastLinkOut,
    Eigen::Vector3d& loopAnchorOut)
{
  // Arbitrary, deliberately non-axis-aligned orientation for the whole linkage.
  const Eigen::Vector3d arbitraryAxis
      = Eigen::Vector3d(1.0, 2.0, 3.0).normalized();
  const Eigen::AngleAxisd linkageRotation(
      std::numbers::pi_v<double> / 6.0, arbitraryAxis); // ~30 degrees

  Eigen::Isometry3d baseTf = Eigen::Isometry3d::Identity();
  baseTf.linear() = linkageRotation.toRotationMatrix();

  SkeletonPtr skel = Skeleton::create("fourBar");

  const double linkLength = 0.4;
  auto boxShape
      = std::make_shared<BoxShape>(Eigen::Vector3d(linkLength, 0.05, 0.05));

  // Base: welded to the world but rotated by the arbitrary transform so the
  // whole linkage (and all child revolute axes/anchors) is non-axis-aligned.
  WeldJoint::Properties baseProps;
  baseProps.mName = "baseJoint";
  baseProps.mT_ParentBodyToJoint = baseTf;
  BodyNode* base
      = skel->createJointAndBodyNodePair<WeldJoint>(nullptr, baseProps).second;
  base->createShapeNodeWith<VisualAspect, CollisionAspect, DynamicsAspect>(
      boxShape);

  // Three serial links connected by revolute joints. The revolute axis is the
  // local Z axis, which - because of the base rotation - is not aligned with
  // any world axis.
  BodyNode* parent = base;
  BodyNode* lastLink = nullptr;
  for (std::size_t i = 0; i < 3; ++i) {
    RevoluteJoint::Properties props;
    props.mName = "joint" + std::to_string(i);
    props.mAxis = Eigen::Vector3d::UnitZ();
    props.mT_ParentBodyToJoint.translation()
        = Eigen::Vector3d(linkLength, 0.0, 0.0);
    props.mT_ChildBodyToJoint.translation()
        = Eigen::Vector3d(-linkLength / 2.0, 0.0, 0.0);

    BodyNode* link
        = skel->createJointAndBodyNodePair<RevoluteJoint>(parent, props).second;
    link->createShapeNodeWith<VisualAspect, CollisionAspect, DynamicsAspect>(
        boxShape);

    parent = link;
    lastLink = link;
  }

  // Close the loop between the free end of the last link and the base. The
  // anchor is expressed in the world frame; using the last link's current
  // transform keeps the constraint consistent with the rotated linkage.
  const Eigen::Vector3d loopAnchor
      = lastLink->getTransform() * Eigen::Vector3d(linkLength / 2.0, 0.0, 0.0);

  *baseBodyOut = base;
  *lastLinkOut = lastLink;
  loopAnchorOut = loopAnchor;

  return skel;
}

//==============================================================================
// Steps the world and asserts (a) the linkage stays finite the whole time and
// (b) the closed loop stays closed: the loop-closure anchor on the last link
// must not drift away from its anchor on the base by more than
// kMaxLoopClosureError. Without the fix, the Dantzig-only configuration scrubs
// the singular solution to zero and the loop drifts open, violating (b).
void stepAndCheckLoopClosed(
    const WorldPtr& world,
    const SkeletonPtr& skel,
    BodyNode* base,
    BodyNode* lastLink,
    const Eigen::Vector3d& loopAnchor)
{
  const Eigen::Vector3d off1
      = lastLink->getWorldTransform().inverse() * loopAnchor;
  const Eigen::Vector3d off2 = base->getWorldTransform().inverse() * loopAnchor;
  const std::size_t numSteps = 100;
  for (std::size_t i = 0; i < numSteps; ++i) {
    // Apply a small actuation/perturbation on the first revolute joint.
    skel->getJoint(1)->setForce(0, 0.01);

    world->step();

    ASSERT_TRUE(skel->getPositions().allFinite())
        << "Non-finite positions at step " << i << ": "
        << skel->getPositions().transpose();
    ASSERT_TRUE(skel->getVelocities().allFinite())
        << "Non-finite velocities at step " << i << ": "
        << skel->getVelocities().transpose();
    const Eigen::Vector3d p1 = lastLink->getWorldTransform() * off1;
    const Eigen::Vector3d p2 = base->getWorldTransform() * off2;
    const double gap = (p1 - p2).norm();
    ASSERT_LT(gap, kMaxLoopClosureError)
        << "Loop drifted open at step " << i << ": gap=" << gap;
  }
}

} // namespace

//==============================================================================
// PART A: the default World solver uses the Dantzig primary with the PGS
// secondary solver. Stepping a non-axis-aligned closed loop must keep the loop
// closed (the PGS fallback already rescued it; this prevents regressions).
TEST(Issue719, NonAxisAlignedClosedLoopDefaultSolver)
{
  BodyNode* base = nullptr;
  BodyNode* lastLink = nullptr;
  Eigen::Vector3d loopAnchor = Eigen::Vector3d::Zero();
  SkeletonPtr skel = createNonAxisAlignedFourBar(&base, &lastLink, loopAnchor);

  WorldPtr world = World::create();
  world->setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  world->addSkeleton(skel);

  // Close the kinematic loop with a ball joint constraint between the last
  // link and the base. This is what makes the chain a closed loop.
  auto loopConstraint = std::make_shared<constraint::BallJointConstraint>(
      lastLink, base, loopAnchor);
  world->getConstraintSolver()->addConstraint(loopConstraint);

  EXPECT_NO_FATAL_FAILURE(
      stepAndCheckLoopClosed(world, skel, base, lastLink, loopAnchor));
}

//==============================================================================
// PART B: with the secondary LCP solver disabled (nullptr) only the Dantzig
// solver runs. On a singular pivot Dantzig must report failure so the zero
// fail-safe takes over, and the additive CFM floor must keep the loop closed
// rather than letting it drift open. This is the discriminating regression
// guard.
TEST(Issue719, NonAxisAlignedClosedLoopDantzigOnly)
{
  BodyNode* base = nullptr;
  BodyNode* lastLink = nullptr;
  Eigen::Vector3d loopAnchor = Eigen::Vector3d::Zero();
  SkeletonPtr skel = createNonAxisAlignedFourBar(&base, &lastLink, loopAnchor);

  WorldPtr world = World::create();
  world->setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  world->addSkeleton(skel);

  // Disable the secondary (PGS) solver so only Dantzig runs.
  world->getConstraintSolver()->setSecondaryLcpSolver(nullptr);
  EXPECT_EQ(nullptr, world->getConstraintSolver()->getSecondaryLcpSolver());

  auto loopConstraint = std::make_shared<constraint::BallJointConstraint>(
      lastLink, base, loopAnchor);
  world->getConstraintSolver()->addConstraint(loopConstraint);

  EXPECT_NO_FATAL_FAILURE(
      stepAndCheckLoopClosed(world, skel, base, lastLink, loopAnchor));
}
