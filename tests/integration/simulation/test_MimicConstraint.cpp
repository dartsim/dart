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

#include "helpers/GTestUtils.hpp"

#include "dart/utils/dart_resource_retriever.hpp"

#include <dart/config.hpp>

#include <dart/simulation/world.hpp>

#include <dart/constraint/constraint_solver.hpp>
#include <dart/constraint/mimic_motor_constraint.hpp>

#include <dart/math/lcp/pivoting/dantzig_solver.hpp>
#include <dart/math/lcp/projection/pgs_solver.hpp>

#include <dart/io/read.hpp>

#if DART_HAVE_BULLET
  #include <dart/collision/bullet/bullet_collision_detector.hpp>
#endif
#if DART_HAVE_ODE
  #include <dart/collision/ode/ode_collision_detector.hpp>
#endif

#include <dart/dynamics/body_node.hpp>
#include <dart/dynamics/joint.hpp>
#include <dart/dynamics/skeleton.hpp>

#include <dart/math/constants.hpp>

#include <dart/common/uri.hpp>

#include <Eigen/Core>
#include <gtest/gtest.h>

#include <algorithm>
#include <span>
#include <string>
#include <vector>

#include <cmath>

using dart::common::Uri;
using dart::dynamics::Joint;
using dart::dynamics::MimicConstraintType;
using dart::dynamics::SkeletonPtr;
using dart::simulation::WorldPtr;
namespace {

bool hasFiniteState(const SkeletonPtr& skeleton)
{
  bool finite = true;
  skeleton->eachBodyNode([&](dart::dynamics::BodyNode* bn) {
    const auto tf = bn->getWorldTransform();
    if (!tf.matrix().array().isFinite().all())
      finite = false;
  });
  return finite;
}

Eigen::Vector3d getTranslation(const dart::dynamics::BodyNode* bn)
{
  if (bn == nullptr)
    return Eigen::Vector3d::Zero();
  return bn->getWorldTransform().translation();
}

void setCollisionDetector(WorldPtr world, bool useOde)
{
#if DART_HAVE_ODE
  if (useOde) {
    world->getConstraintSolver()->setCollisionDetector(
        dart::collision::OdeCollisionDetector::create());
    return;
  }
#else
  (void)useOde;
#endif
#if DART_HAVE_BULLET
  world->getConstraintSolver()->setCollisionDetector(
      dart::collision::BulletCollisionDetector::create());
#else
  (void)world;
#endif
}

void setBoxedSolver(WorldPtr world, bool usePgs)
{
  auto* boxedSolver = dynamic_cast<dart::constraint::ConstraintSolver*>(
      world->getConstraintSolver());
  if (!boxedSolver)
    return;

  if (usePgs) {
    boxedSolver->setLcpSolver(std::make_shared<dart::math::PgsSolver>());
    boxedSolver->setSecondaryLcpSolver(nullptr);
  } else {
    boxedSolver->setLcpSolver(std::make_shared<dart::math::DantzigSolver>());
    boxedSolver->setSecondaryLcpSolver(
        std::make_shared<dart::math::PgsSolver>());
  }
}

void retargetMimicJoints(const WorldPtr& world, const std::string& baselineName)
{
  ASSERT_TRUE(world);

  const auto baseline = world->getSkeleton(baselineName);
  ASSERT_NE(nullptr, baseline)
      << "Missing baseline skeleton [" << baselineName << "]";

  for (std::size_t i = 0; i < world->getNumSkeletons(); ++i) {
    const auto skeleton = world->getSkeleton(i);
    if (!skeleton)
      continue;

    for (std::size_t j = 0; j < skeleton->getNumJoints(); ++j) {
      auto* joint = skeleton->getJoint(j);
      if (!joint)
        continue;

      const auto props = joint->getMimicDofProperties();
      if (props.empty())
        continue;

      if (skeleton == baseline) {
        std::vector<dart::dynamics::MimicDofProperties> clearedProps(
            joint->getNumDofs());
        joint->setMimicJointDofs(
            std::span<const dart::dynamics::MimicDofProperties>(clearedProps));
        joint->setActuatorType(dart::dynamics::Joint::FORCE);
        joint->setUseCouplerConstraint(false);
        continue;
      }

      bool updated = false;
      for (std::size_t dofIndex = 0; dofIndex < props.size(); ++dofIndex) {
        auto prop = props[dofIndex];
        if (prop.mReferenceJoint == nullptr)
          continue;

        auto* ref = baseline->getJoint(prop.mReferenceJoint->getName());
        ASSERT_NE(nullptr, ref);
        prop.mReferenceJoint = ref;
        joint->setMimicJointDof(dofIndex, prop);
        updated = true;
      }

      if (updated) {
        joint->setActuatorType(Joint::MIMIC);
        joint->setUseCouplerConstraint(false);
      }
    }
  }
}

} // namespace

//==============================================================================
TEST(MimicConstraint, PendulumMimicWorldFromSdf)
{
  const std::string worldUri
      = "dart://sample/sdf/test/mimic_fast_slow_pendulums_world.sdf";

  auto retriever = std::make_shared<dart::utils::DartResourceRetriever>();
  dart::io::ReadOptions options;
  options.resourceRetriever = retriever;

  WorldPtr world = dart::io::readWorld(Uri(worldUri), options);
  ASSERT_TRUE(world);

  retargetMimicJoints(world, "pendulum_with_base");
  setCollisionDetector(world, /*useOde=*/true);
  setBoxedSolver(world, /*usePgs=*/false);

  const auto baseline = world->getSkeleton("pendulum_with_base");
  ASSERT_TRUE(baseline);
  const auto slowFollower
      = world->getSkeleton("pendulum_with_base_mimic_slow_follows_fast");
  ASSERT_TRUE(slowFollower);
  const auto fastFollower
      = world->getSkeleton("pendulum_with_base_mimic_fast_follows_slow");
  ASSERT_TRUE(fastFollower);

  const auto slowBase = slowFollower->getBodyNode("base");
  const auto fastBase = fastFollower->getBodyNode("base");
  ASSERT_NE(nullptr, slowBase);
  ASSERT_NE(nullptr, fastBase);
  const auto baselineBase = baseline->getBodyNode("base");
  ASSERT_NE(nullptr, baselineBase);

  auto* baselineFast = baseline->getJoint("fast_joint");
  auto* baselineSlow = baseline->getJoint("slow_joint");
  ASSERT_NE(nullptr, baselineFast);
  ASSERT_NE(nullptr, baselineSlow);
  EXPECT_NE(baselineFast->getActuatorType(), Joint::MIMIC);
  EXPECT_NE(baselineSlow->getActuatorType(), Joint::MIMIC);
  auto* slowJoint = slowFollower->getJoint("slow_joint");
  auto* slowRef = baseline->getJoint("fast_joint");
  auto* fastFollowJoint = fastFollower->getJoint("fast_joint");
  auto* fastRef = baseline->getJoint("slow_joint");
  ASSERT_NE(nullptr, slowJoint);
  ASSERT_NE(nullptr, slowRef);
  ASSERT_NE(nullptr, fastFollowJoint);
  ASSERT_NE(nullptr, fastRef);

  auto expectMimic = [](Joint* follower, const Joint* reference) {
    const auto props = follower->getMimicDofProperties();
    ASSERT_FALSE(props.empty());
    EXPECT_EQ(props[0].mReferenceJoint, reference);
    EXPECT_EQ(props[0].mReferenceDofIndex, 0u);
    EXPECT_EQ(props[0].mConstraintType, MimicConstraintType::Motor);
    EXPECT_FALSE(follower->isUsingCouplerConstraint());
    EXPECT_EQ(follower->getActuatorType(), Joint::MIMIC);
  };

  expectMimic(slowJoint, slowRef);
  expectMimic(fastFollowJoint, fastRef);

  const Eigen::Vector3d slowBaseStart = getTranslation(slowBase);
  const Eigen::Vector3d fastBaseStart = getTranslation(fastBase);
  const Eigen::Vector3d baselineBaseStart = getTranslation(baselineBase);

  constexpr int steps = 32;
  for (int i = 0; i < steps; ++i) {
    world->step();
    ASSERT_TRUE(hasFiniteState(slowFollower));
    ASSERT_TRUE(hasFiniteState(fastFollower));
    ASSERT_TRUE(hasFiniteState(baseline));
  }

  const Eigen::Vector3d slowBaseNow = getTranslation(slowBase);
  const Eigen::Vector3d fastBaseNow = getTranslation(fastBase);
  const Eigen::Vector3d baselineBaseNow = getTranslation(baselineBase);

  const double baseDriftTol = 1e-3;
  EXPECT_LT((slowBaseNow - slowBaseStart).norm(), baseDriftTol);
  EXPECT_LT((fastBaseNow - fastBaseStart).norm(), baseDriftTol);
  EXPECT_LT((baselineBaseNow - baselineBaseStart).norm(), baseDriftTol);

  const double angleTol = 2e-2;
  const double velTol = 1e-1;
  EXPECT_NEAR(slowJoint->getPosition(0), slowRef->getPosition(0), angleTol);
  EXPECT_NEAR(
      fastFollowJoint->getPosition(0), fastRef->getPosition(0), angleTol);
  EXPECT_NEAR(slowJoint->getVelocity(0), slowRef->getVelocity(0), velTol);
  EXPECT_NEAR(fastFollowJoint->getVelocity(0), fastRef->getVelocity(0), velTol);
}

//==============================================================================
TEST(MimicConstraint, FollowersMatchMiddlePendulum)
{
  const std::string worldUri
      = "dart://sample/sdf/test/mimic_fast_slow_pendulums_world.sdf";

  auto retriever = std::make_shared<dart::utils::DartResourceRetriever>();
  dart::io::ReadOptions options;
  options.resourceRetriever = retriever;

  WorldPtr world = dart::io::readWorld(Uri(worldUri), options);
  ASSERT_TRUE(world);

  retargetMimicJoints(world, "pendulum_with_base");
  setCollisionDetector(world, /*useOde=*/true);
  setBoxedSolver(world, /*usePgs=*/false);

  const auto baseline = world->getSkeleton("pendulum_with_base");
  const auto slowFollower
      = world->getSkeleton("pendulum_with_base_mimic_slow_follows_fast");
  const auto fastFollower
      = world->getSkeleton("pendulum_with_base_mimic_fast_follows_slow");
  ASSERT_TRUE(baseline);
  ASSERT_TRUE(slowFollower);
  ASSERT_TRUE(fastFollower);

  auto* baseSlowJoint = baseline->getJoint("slow_joint");
  auto* baseFastJoint = baseline->getJoint("fast_joint");
  auto* slowFollowJoint = slowFollower->getJoint("slow_joint");
  auto* slowReference = baseline->getJoint("fast_joint");
  auto* fastFollowJoint = fastFollower->getJoint("fast_joint");
  auto* fastReference = baseline->getJoint("slow_joint");
  ASSERT_NE(nullptr, baseSlowJoint);
  ASSERT_NE(nullptr, baseFastJoint);
  ASSERT_NE(nullptr, slowFollowJoint);
  ASSERT_NE(nullptr, slowReference);
  ASSERT_NE(nullptr, fastFollowJoint);
  ASSERT_NE(nullptr, fastReference);

  // Followers must use mimic motors (unilateral), never couplers.
  auto expectMotorOnly = [](Joint* follower, const Joint* reference) {
    const auto props = follower->getMimicDofProperties();
    ASSERT_FALSE(props.empty());
    EXPECT_EQ(props[0].mConstraintType, MimicConstraintType::Motor);
    EXPECT_EQ(props[0].mReferenceJoint, reference);
    EXPECT_EQ(props[0].mReferenceDofIndex, 0u);
    EXPECT_FALSE(follower->isUsingCouplerConstraint());
    EXPECT_EQ(follower->getActuatorType(), Joint::MIMIC);
  };
  expectMotorOnly(slowFollowJoint, slowReference);
  expectMotorOnly(fastFollowJoint, fastReference);

  const Eigen::Vector3d baselineBaseStart
      = getTranslation(baseline->getBodyNode("base"));
  const Eigen::Vector3d slowBaseStart
      = getTranslation(slowFollower->getBodyNode("base"));
  const Eigen::Vector3d fastBaseStart
      = getTranslation(fastFollower->getBodyNode("base"));

  double maxDriftBaseline = 0.0;
  double maxDriftSlow = 0.0;
  double maxDriftFast = 0.0;
  double maxAngleDiffSlow = 0.0;
  double maxAngleDiffFast = 0.0;
  double maxVelDiffSlow = 0.0;
  double maxVelDiffFast = 0.0;
  double maxBaseSlowVel = 0.0;
  double maxBaseFastVel = 0.0;
  double maxSlowFollowerVel = 0.0;
  double maxFastFollowerVel = 0.0;

  const double angleTol = 2e-2; // ~1.15 degrees
  const double velTol = 1e-1;   // 0.1 rad/s
  const double driftTol = 2e-3; // 2 mm
  const int steps = 800;

  for (int i = 0; i < steps; ++i) {
    world->step();
    ASSERT_TRUE(hasFiniteState(slowFollower));
    ASSERT_TRUE(hasFiniteState(fastFollower));
    ASSERT_TRUE(hasFiniteState(baseline));

    maxDriftBaseline = std::max(
        maxDriftBaseline,
        (getTranslation(baseline->getBodyNode("base")) - baselineBaseStart)
            .norm());
    maxDriftSlow = std::max(
        maxDriftSlow,
        (getTranslation(slowFollower->getBodyNode("base")) - slowBaseStart)
            .norm());
    maxDriftFast = std::max(
        maxDriftFast,
        (getTranslation(fastFollower->getBodyNode("base")) - fastBaseStart)
            .norm());

    const double baseSlowVel = baseSlowJoint->getVelocity(0);
    const double baseFastVel = baseFastJoint->getVelocity(0);

    const double followerSlow = slowFollowJoint->getPosition(0);
    const double followerFast = fastFollowJoint->getPosition(0);
    const double followerSlowVel = slowFollowJoint->getVelocity(0);
    const double followerFastVel = fastFollowJoint->getVelocity(0);

    maxAngleDiffSlow = std::max(
        maxAngleDiffSlow,
        std::abs(followerSlow - slowReference->getPosition(0)));
    maxAngleDiffFast = std::max(
        maxAngleDiffFast,
        std::abs(followerFast - fastReference->getPosition(0)));
    maxVelDiffSlow = std::max(
        maxVelDiffSlow,
        std::abs(followerSlowVel - slowReference->getVelocity(0)));
    maxVelDiffFast = std::max(
        maxVelDiffFast,
        std::abs(followerFastVel - fastReference->getVelocity(0)));
    maxBaseSlowVel = std::max(maxBaseSlowVel, std::abs(baseSlowVel));
    maxBaseFastVel = std::max(maxBaseFastVel, std::abs(baseFastVel));
    maxSlowFollowerVel
        = std::max(maxSlowFollowerVel, std::abs(followerSlowVel));
    maxFastFollowerVel
        = std::max(maxFastFollowerVel, std::abs(followerFastVel));
  }

  EXPECT_LT(maxDriftBaseline, driftTol);
  EXPECT_LT(maxDriftSlow, driftTol);
  EXPECT_LT(maxDriftFast, driftTol);

  // Mimic relationships should hold at every step.
  EXPECT_LT(maxAngleDiffSlow, angleTol);
  EXPECT_LT(maxAngleDiffFast, angleTol);
  EXPECT_LT(maxVelDiffSlow, velTol);
  EXPECT_LT(maxVelDiffFast, velTol);

  // Baseline pendulum rods should actually swing (not stuck at rest).
  EXPECT_GT(maxBaseSlowVel, 1e-1);
  EXPECT_GT(maxBaseFastVel, 1e-1);
  EXPECT_GT(maxSlowFollowerVel, 1e-1);
  EXPECT_GT(maxFastFollowerVel, 1e-1);
}

//==============================================================================
TEST(MimicConstraint, OdeMimicDoesNotExplode)
{
#if !DART_HAVE_ODE
  GTEST_SKIP() << "ODE collision is not available in this build";
#endif

  const std::string worldUri
      = "dart://sample/sdf/test/mimic_fast_slow_pendulums_world.sdf";

  auto retriever = std::make_shared<dart::utils::DartResourceRetriever>();
  dart::io::ReadOptions options;
  options.resourceRetriever = retriever;

  WorldPtr world = dart::io::readWorld(Uri(worldUri), options);
  ASSERT_TRUE(world);

  retargetMimicJoints(world, "pendulum_with_base");
  setCollisionDetector(world, /*useOde=*/true);
  setBoxedSolver(world, /*usePgs=*/false);

  const auto baseline = world->getSkeleton("pendulum_with_base");
  const auto slowFollower
      = world->getSkeleton("pendulum_with_base_mimic_slow_follows_fast");
  const auto fastFollower
      = world->getSkeleton("pendulum_with_base_mimic_fast_follows_slow");
  ASSERT_TRUE(baseline);
  ASSERT_TRUE(slowFollower);
  ASSERT_TRUE(fastFollower);

  const auto slowBase = slowFollower->getBodyNode("base");
  const auto fastBase = fastFollower->getBodyNode("base");
  const auto baselineBase = baseline->getBodyNode("base");
  ASSERT_NE(nullptr, slowBase);
  ASSERT_NE(nullptr, fastBase);
  ASSERT_NE(nullptr, baselineBase);

  auto* baselineSlow = baseline->getJoint("slow_joint");
  auto* baselineFast = baseline->getJoint("fast_joint");
  auto* slowJoint = slowFollower->getJoint("slow_joint");
  auto* slowReference = baseline->getJoint("fast_joint");
  auto* fastJoint = fastFollower->getJoint("fast_joint");
  auto* fastReference = baseline->getJoint("slow_joint");
  ASSERT_NE(nullptr, baselineSlow);
  ASSERT_NE(nullptr, baselineFast);
  ASSERT_NE(nullptr, slowJoint);
  ASSERT_NE(nullptr, slowReference);
  ASSERT_NE(nullptr, fastJoint);
  ASSERT_NE(nullptr, fastReference);

  const Eigen::Vector3d slowBaseStart = getTranslation(slowBase);
  const Eigen::Vector3d fastBaseStart = getTranslation(fastBase);
  const Eigen::Vector3d baselineBaseStart = getTranslation(baselineBase);

  const int steps = 1500;
  for (int i = 0; i < steps; ++i) {
    world->step();
    ASSERT_TRUE(hasFiniteState(slowFollower));
    ASSERT_TRUE(hasFiniteState(fastFollower));
    ASSERT_TRUE(hasFiniteState(baseline));
  }

  const Eigen::Vector3d slowBaseNow = getTranslation(slowBase);
  const Eigen::Vector3d fastBaseNow = getTranslation(fastBase);
  const Eigen::Vector3d baselineBaseNow = getTranslation(baselineBase);

  // Bases should not drift significantly (instability moves them meters).
  constexpr double driftTol = 1e-2;
  EXPECT_LT((slowBaseNow - slowBaseStart).norm(), driftTol);
  EXPECT_LT((fastBaseNow - fastBaseStart).norm(), driftTol);
  EXPECT_LT((baselineBaseNow - baselineBaseStart).norm(), driftTol);

  // Followers should track their references within a reasonable band.
  const double slowError
      = slowJoint->getPosition(0) - slowReference->getPosition(0);
  const double fastError
      = fastJoint->getPosition(0) - fastReference->getPosition(0);
  const double slowVelError
      = slowJoint->getVelocity(0) - slowReference->getVelocity(0);
  const double fastVelError
      = fastJoint->getVelocity(0) - fastReference->getVelocity(0);
  const double angleTol = 5e-2;
  const double velTol = 1e-1;
  EXPECT_LT(std::abs(slowError), angleTol);
  EXPECT_LT(std::abs(fastError), angleTol);
  EXPECT_LT(std::abs(slowVelError), velTol);
  EXPECT_LT(std::abs(fastVelError), velTol);
}

//==============================================================================
TEST(MimicConstraint, OdeTracksReferenceLongRun)
{
#if !DART_HAVE_ODE
  GTEST_SKIP() << "ODE collision is not available in this build";
#endif

  const std::string worldUri
      = "dart://sample/sdf/test/mimic_fast_slow_pendulums_world.sdf";

  auto retriever = std::make_shared<dart::utils::DartResourceRetriever>();
  dart::io::ReadOptions options;
  options.resourceRetriever = retriever;

  WorldPtr world = dart::io::readWorld(Uri(worldUri), options);
  ASSERT_TRUE(world);

  setCollisionDetector(world, /*useOde=*/true);
  setBoxedSolver(world, /*usePgs=*/false);

  const auto baseline = world->getSkeleton("pendulum_with_base");
  const auto slowFollower
      = world->getSkeleton("pendulum_with_base_mimic_slow_follows_fast");
  const auto fastFollower
      = world->getSkeleton("pendulum_with_base_mimic_fast_follows_slow");
  ASSERT_TRUE(baseline);
  ASSERT_TRUE(slowFollower);
  ASSERT_TRUE(fastFollower);

  const Eigen::Vector3d baselineBaseStart
      = getTranslation(baseline->getBodyNode("base"));
  const Eigen::Vector3d slowBaseStart
      = getTranslation(slowFollower->getBodyNode("base"));
  const Eigen::Vector3d fastBaseStart
      = getTranslation(fastFollower->getBodyNode("base"));

  auto* baseSlowJoint = baseline->getJoint("slow_joint");
  auto* baseFastJoint = baseline->getJoint("fast_joint");
  auto* slowFollowJoint = slowFollower->getJoint("slow_joint");
  auto* slowReference = baseline->getJoint("fast_joint");
  auto* fastFollowJoint = fastFollower->getJoint("fast_joint");
  auto* fastReference = baseline->getJoint("slow_joint");
  ASSERT_NE(nullptr, baseSlowJoint);
  ASSERT_NE(nullptr, baseFastJoint);
  ASSERT_NE(nullptr, slowFollowJoint);
  ASSERT_NE(nullptr, fastFollowJoint);
  ASSERT_NE(nullptr, slowReference);
  ASSERT_NE(nullptr, fastReference);

  const double initSlowAbs = std::abs(baseSlowJoint->getPosition(0));
  const double initFastAbs = std::abs(baseFastJoint->getPosition(0));

  double maxDriftBase = 0.0;
  double maxDriftSlow = 0.0;
  double maxDriftFast = 0.0;

  double maxAbsSlow = initSlowAbs;
  double maxAbsFast = initFastAbs;
  double maxAbsSlowFollow = std::abs(slowFollowJoint->getPosition(0));
  double maxAbsFastFollow = std::abs(fastFollowJoint->getPosition(0));

  double maxAngleDiffSlow = 0.0;
  double maxVelDiffSlow = 0.0;
  double maxAngleDiffFast = 0.0;
  double maxVelDiffFast = 0.0;

  const double driftTol = 1e-3; // 1mm base drift tolerance
  const double angleTol = 2e-2; // ~1.15 degrees
  const double velTol = 1e-1;   // 0.1 rad/s
  const double maxAngle
      = dart::math::pi + 1e-2; // allow full swing with small margin
  const double minAngle = 1.0;

  const int steps = 10000;
  for (int i = 0; i < steps; ++i) {
    world->step();
    ASSERT_TRUE(hasFiniteState(slowFollower));
    ASSERT_TRUE(hasFiniteState(fastFollower));
    ASSERT_TRUE(hasFiniteState(baseline));

    maxDriftBase = std::max(
        maxDriftBase,
        (getTranslation(baseline->getBodyNode("base")) - baselineBaseStart)
            .norm());
    maxDriftSlow = std::max(
        maxDriftSlow,
        (getTranslation(slowFollower->getBodyNode("base")) - slowBaseStart)
            .norm());
    maxDriftFast = std::max(
        maxDriftFast,
        (getTranslation(fastFollower->getBodyNode("base")) - fastBaseStart)
            .norm());

    const double baseSlow = baseSlowJoint->getPosition(0);
    const double baseFast = baseFastJoint->getPosition(0);
    const double followerFast = fastFollowJoint->getPosition(0);
    const double followerFastVel = fastFollowJoint->getVelocity(0);
    const double followerSlow = slowFollowJoint->getPosition(0);
    const double followerSlowVel = slowFollowJoint->getVelocity(0);

    maxAbsSlow = std::max(maxAbsSlow, std::abs(baseSlow));
    maxAbsFast = std::max(maxAbsFast, std::abs(baseFast));
    maxAbsSlowFollow = std::max(maxAbsSlowFollow, std::abs(followerSlow));
    maxAbsFastFollow = std::max(maxAbsFastFollow, std::abs(followerFast));

    maxAngleDiffSlow = std::max(
        maxAngleDiffSlow,
        std::abs(followerSlow - slowReference->getPosition(0)));
    maxVelDiffSlow = std::max(
        maxVelDiffSlow,
        std::abs(followerSlowVel - slowReference->getVelocity(0)));
    maxAngleDiffFast = std::max(
        maxAngleDiffFast,
        std::abs(followerFast - fastReference->getPosition(0)));
    maxVelDiffFast = std::max(
        maxVelDiffFast,
        std::abs(followerFastVel - fastReference->getVelocity(0)));

    EXPECT_LT(std::abs(followerFast - fastReference->getPosition(0)), angleTol);
    EXPECT_LT(std::abs(followerSlow - slowReference->getPosition(0)), angleTol);
    EXPECT_LT(
        std::abs(followerFastVel - fastReference->getVelocity(0)), velTol);
    EXPECT_LT(
        std::abs(followerSlowVel - slowReference->getVelocity(0)), velTol);
    EXPECT_LT(std::abs(baseSlow), maxAngle);
    EXPECT_LT(std::abs(baseFast), maxAngle);
    EXPECT_LT(std::abs(followerFast), maxAngle);
    EXPECT_LT(std::abs(followerSlow), maxAngle);
  }

  // Bases should stay nearly stationary (free-floating but heavy).
  EXPECT_LT(maxDriftBase, driftTol);
  EXPECT_LT(maxDriftSlow, driftTol);
  EXPECT_LT(maxDriftFast, driftTol);

  // Swing amplitude should stay within limits and not collapse to zero.
  EXPECT_LT(maxAbsSlow, maxAngle);
  EXPECT_LT(maxAbsFast, maxAngle);
  EXPECT_LT(maxAbsSlowFollow, maxAngle);
  EXPECT_LT(maxAbsFastFollow, maxAngle);
  EXPECT_GT(maxAbsSlow, minAngle);
  EXPECT_GT(maxAbsFast, minAngle);
  EXPECT_GT(maxAbsSlowFollow, minAngle);
  EXPECT_GT(maxAbsFastFollow, minAngle);

  // Mimic tracking against the SDF-defined references.
  EXPECT_LT(maxAngleDiffSlow, angleTol);
  EXPECT_LT(maxVelDiffSlow, velTol);
  EXPECT_LT(maxAngleDiffFast, angleTol);
  EXPECT_LT(maxVelDiffFast, velTol);
}
