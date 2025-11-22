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

#include "helpers/GTestUtils.hpp"

#include "dart/utils/DartResourceRetriever.hpp"
#include "dart/utils/sdf/SdfParser.hpp"
#include "dart/utils/sdf/detail/SdfHelpers.hpp"

#include <dart/config.hpp>
#include <dart/simulation/World.hpp>
#include <dart/collision/bullet/BulletCollisionDetector.hpp>
#include <dart/constraint/ConstraintSolver.hpp>
#include <dart/constraint/CouplerConstraint.hpp>
#include <dart/constraint/JointConstraint.hpp>
#include <dart/constraint/MimicMotorConstraint.hpp>
#include <dart/constraint/BoxedLcpConstraintSolver.hpp>
#include <dart/constraint/DantzigBoxedLcpSolver.hpp>
#include <dart/constraint/PgsBoxedLcpSolver.hpp>
#if HAVE_ODE
  #include <dart/collision/ode/OdeCollisionDetector.hpp>
#endif

#include <dart/dynamics/BodyNode.hpp>
#include <dart/dynamics/Joint.hpp>
#include <dart/dynamics/Skeleton.hpp>

#include <dart/common/Uri.hpp>

#include <Eigen/Core>
#include <gtest/gtest.h>
#include <sdf/Root.hh>
#include <sdf/sdf.hh>

#include <algorithm>
#include <sstream>
#include <string>
#include <vector>

#include <cmath>

using dart::common::Uri;
using dart::dynamics::Joint;
using dart::dynamics::MimicConstraintType;
using dart::dynamics::SkeletonPtr;
using dart::simulation::WorldPtr;
using dart::utils::SdfParser::detail::ElementEnumerator;
using dart::utils::SdfParser::detail::getAttributeString;
using dart::utils::SdfParser::detail::getElement;
using dart::utils::SdfParser::detail::getValueDouble;
using dart::utils::SdfParser::detail::hasAttribute;
using dart::utils::SdfParser::detail::hasElement;
using dart::collision::BulletCollisionDetector;

namespace {

struct MimicSpec
{
  std::string model;
  std::string followerJoint;
  std::string referenceJoint;
  std::size_t referenceDof = 0;
  double multiplier = 1.0;
  double offset = 0.0;
};

std::vector<MimicSpec> parseMimicSpecs(const std::string& sdfText)
{
  sdf::Root root;
  const auto errors = root.LoadSdfString(sdfText);
  if (!errors.empty()) {
    ADD_FAILURE() << "Failed to load SDF from text: "
                  << errors.front().Message();
    return {};
  }

  const auto rootElement = root.Element();
  if (!rootElement)
    return {};

  const auto worldElement = getElement(rootElement, "world");
  if (!worldElement)
    return {};

  std::vector<MimicSpec> specs;
  ElementEnumerator modelEnum(worldElement, "model");
  while (modelEnum.next()) {
    const auto modelElement = modelEnum.get();
    if (!modelElement)
      continue;

    const auto modelName = getAttributeString(modelElement, "name");
    ElementEnumerator jointEnum(modelElement, "joint");
    while (jointEnum.next()) {
      const auto jointElement = jointEnum.get();
      if (!jointElement || !hasElement(jointElement, "axis"))
        continue;

      const auto axisElement = getElement(jointElement, "axis");
      if (!hasElement(axisElement, "mimic"))
        continue;

      const auto mimicElement = getElement(axisElement, "mimic");
      if (!mimicElement)
        continue;

      MimicSpec spec;
      spec.model = modelName;
      spec.followerJoint = getAttributeString(jointElement, "name");
      spec.referenceJoint = getAttributeString(mimicElement, "joint");
      const auto axisAttribute = hasAttribute(mimicElement, "axis")
                                     ? getAttributeString(mimicElement, "axis")
                                     : std::string();
      spec.referenceDof = axisAttribute == "axis2" ? 1u : 0u;
      spec.multiplier = hasElement(mimicElement, "multiplier")
                            ? getValueDouble(mimicElement, "multiplier")
                            : 1.0;
      spec.offset = hasElement(mimicElement, "offset")
                        ? getValueDouble(mimicElement, "offset")
                        : 0.0;

      specs.push_back(spec);
    }
  }

  return specs;
}

struct MimicTuning
{
  double positionLimit = 1.7;    // ~97 degrees
  double velocityLimit = 20.0;
  double forceLimit = 800.0;
  double damping = 2.0;
  double erp = 0.4;
  double cfm = 1e-6;
};

void setJointStabilization(dart::dynamics::Joint* joint, const MimicTuning& tuning)
{
  if (!joint)
    return;
  joint->setPositionLowerLimit(0, -tuning.positionLimit);
  joint->setPositionUpperLimit(0, tuning.positionLimit);
  joint->setLimitEnforcement(true);
  joint->setVelocityLowerLimit(0, -tuning.velocityLimit);
  joint->setVelocityUpperLimit(0, tuning.velocityLimit);
  joint->setForceLowerLimit(0, -tuning.forceLimit);
  joint->setForceUpperLimit(0, tuning.forceLimit);
  joint->setDampingCoefficient(0, tuning.damping);
}

void configureMimicMotors(
    const std::vector<MimicSpec>& specs,
    const WorldPtr& world,
    const MimicTuning& tuning)
{
  for (const auto& spec : specs) {
    const auto skeleton = world->getSkeleton(spec.model);
    ASSERT_NE(nullptr, skeleton);

    auto* follower = skeleton->getJoint(spec.followerJoint);
    auto* reference = skeleton->getJoint(spec.referenceJoint);
    ASSERT_NE(nullptr, follower);
    ASSERT_NE(nullptr, reference);
    ASSERT_GT(follower->getNumDofs(), 0u);
    ASSERT_GT(reference->getNumDofs(), 0u);

    std::vector<dart::dynamics::MimicDofProperties> mimicProps
        = follower->getMimicDofProperties();
    mimicProps.resize(follower->getNumDofs());

    const std::size_t followerIndex
        = std::min(spec.referenceDof, follower->getNumDofs() - 1);
    const std::size_t referenceIndex
        = std::min(spec.referenceDof, reference->getNumDofs() - 1);
    auto& prop = mimicProps[followerIndex];
    prop.mReferenceJoint = reference;
    prop.mReferenceDofIndex = referenceIndex;
    prop.mMultiplier = spec.multiplier;
    prop.mOffset = spec.offset;
    prop.mConstraintType = MimicConstraintType::Motor;

    follower->setMimicJointDofs(mimicProps);
    follower->setActuatorType(Joint::MIMIC);
    follower->setUseCouplerConstraint(false);

    setJointStabilization(follower, tuning);
    setJointStabilization(
        const_cast<dart::dynamics::Joint*>(prop.mReferenceJoint), tuning);
  }
}

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
#if HAVE_ODE
  if (useOde) {
    world->getConstraintSolver()->setCollisionDetector(
        dart::collision::OdeCollisionDetector::create());
    return;
  }
#else
  (void)useOde;
#endif
  world->getConstraintSolver()->setCollisionDetector(
      BulletCollisionDetector::create());
}

void setBoxedSolver(WorldPtr world, bool usePgs)
{
  auto* boxedSolver = dynamic_cast<dart::constraint::BoxedLcpConstraintSolver*>(
      world->getConstraintSolver());
  if (!boxedSolver)
    return;

  if (usePgs) {
    boxedSolver->setBoxedLcpSolver(
        std::make_shared<dart::constraint::PgsBoxedLcpSolver>());
    boxedSolver->setSecondaryBoxedLcpSolver(nullptr);
  } else {
    boxedSolver->setBoxedLcpSolver(
        std::make_shared<dart::constraint::DantzigBoxedLcpSolver>());
    boxedSolver->setSecondaryBoxedLcpSolver(
        std::make_shared<dart::constraint::PgsBoxedLcpSolver>());
  }
}

double angleError(const SkeletonPtr& skel, const std::string& ref, const std::string& follower)
{
  auto* refJoint = skel->getJoint(ref);
  auto* folJoint = skel->getJoint(follower);
  if (!refJoint || !folJoint)
    return 0.0;
  return folJoint->getPosition(0) - refJoint->getPosition(0);
}

} // namespace

//==============================================================================
TEST(MimicConstraint, PendulumMimicWorldFromSdf)
{
  const std::string worldUri
      = "dart://sample/sdf/test/mimic_fast_slow_pendulums_world.sdf";

  auto retriever = std::make_shared<dart::utils::DartResourceRetriever>();
  dart::utils::SdfParser::Options options(retriever);

  WorldPtr world = dart::utils::SdfParser::readWorld(Uri(worldUri), options);
  ASSERT_TRUE(world);

  auto resource = retriever->retrieve(Uri(worldUri));
  ASSERT_TRUE(resource);
  std::string sdfText(resource->getSize(), '\0');
  const auto read = resource->read(sdfText.data(), 1, sdfText.size());
  ASSERT_EQ(read, resource->getSize());

  const auto specs = parseMimicSpecs(sdfText);
  ASSERT_FALSE(specs.empty());
  MimicTuning tuning;
  configureMimicMotors(specs, world, tuning);

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

  const Eigen::Vector3d slowBaseStart = getTranslation(slowBase);
  const Eigen::Vector3d fastBaseStart = getTranslation(fastBase);

  constexpr int steps = 32;
  for (int i = 0; i < steps; ++i) {
    world->step();
    ASSERT_TRUE(hasFiniteState(slowFollower));
    ASSERT_TRUE(hasFiniteState(fastFollower));
  }

  const Eigen::Vector3d slowBaseNow = getTranslation(slowBase);
  const Eigen::Vector3d fastBaseNow = getTranslation(fastBase);

  EXPECT_LT((slowBaseNow.head<2>() - slowBaseStart.head<2>()).norm(), 0.5);
  EXPECT_LT((fastBaseNow.head<2>() - fastBaseStart.head<2>()).norm(), 0.5);
  EXPECT_LT(std::abs(slowBaseNow.z() - slowBaseStart.z()), 0.5);
  EXPECT_LT(std::abs(fastBaseNow.z() - fastBaseStart.z()), 0.5);

  auto* slowJoint = slowFollower->getJoint("slow_joint");
  auto* fastJoint = slowFollower->getJoint("fast_joint");
  ASSERT_NE(nullptr, slowJoint);
  ASSERT_NE(nullptr, fastJoint);

  const double slowAngle = slowJoint->getPosition(0);
  const double fastAngle = fastJoint->getPosition(0);

  ASSERT_TRUE(std::isfinite(slowAngle));
  ASSERT_TRUE(std::isfinite(fastAngle));
  EXPECT_LT(std::abs(slowAngle - fastAngle), 0.2);
}

//==============================================================================
TEST(MimicConstraint, OdeMimicDoesNotExplode)
{
#if !HAVE_ODE
  GTEST_SKIP() << "ODE collision is not available in this build";
#endif

  const std::string worldUri
      = "dart://sample/sdf/test/mimic_fast_slow_pendulums_world.sdf";

  auto retriever = std::make_shared<dart::utils::DartResourceRetriever>();
  dart::utils::SdfParser::Options options(retriever);

  WorldPtr world = dart::utils::SdfParser::readWorld(Uri(worldUri), options);
  ASSERT_TRUE(world);

  auto resource = retriever->retrieve(Uri(worldUri));
  ASSERT_TRUE(resource);
  std::string sdfText(resource->getSize(), '\0');
  const auto read = resource->read(sdfText.data(), 1, sdfText.size());
  ASSERT_EQ(read, resource->getSize());

  const auto specs = parseMimicSpecs(sdfText);
  ASSERT_FALSE(specs.empty());
  MimicTuning tuning;
  dart::constraint::MimicMotorConstraint::setConstraintForceMixing(tuning.cfm);
  dart::constraint::JointConstraint::setErrorReductionParameter(tuning.erp);

  configureMimicMotors(specs, world, tuning);
  setCollisionDetector(world, /*useOde=*/true);
  setBoxedSolver(world, /*usePgs=*/true);

  const auto slowFollower
      = world->getSkeleton("pendulum_with_base_mimic_slow_follows_fast");
  const auto fastFollower
      = world->getSkeleton("pendulum_with_base_mimic_fast_follows_slow");
  ASSERT_TRUE(slowFollower);
  ASSERT_TRUE(fastFollower);

  const Eigen::Vector3d slowBaseStart
      = getTranslation(slowFollower->getBodyNode("base"));
  const Eigen::Vector3d fastBaseStart
      = getTranslation(fastFollower->getBodyNode("base"));

  const int steps = 1500;
  for (int i = 0; i < steps; ++i) {
    world->step();
    ASSERT_TRUE(hasFiniteState(slowFollower));
    ASSERT_TRUE(hasFiniteState(fastFollower));
  }

  const Eigen::Vector3d slowBaseNow
      = getTranslation(slowFollower->getBodyNode("base"));
  const Eigen::Vector3d fastBaseNow
      = getTranslation(fastFollower->getBodyNode("base"));

  // Bases should not drift significantly (instability moves them meters).
  EXPECT_LT((slowBaseNow - slowBaseStart).norm(), 0.5);
  EXPECT_LT((fastBaseNow - fastBaseStart).norm(), 0.5);

  // Followers should track their references within a reasonable band.
  const double slowError
      = angleError(slowFollower, "fast_joint", "slow_joint");
  const double fastError
      = angleError(fastFollower, "slow_joint", "fast_joint");
  EXPECT_LT(std::abs(slowError), 0.5);
  EXPECT_LT(std::abs(fastError), 0.5);
}

//==============================================================================
TEST(MimicConstraint, OdeTracksReferenceLongRun)
{
#if !HAVE_ODE
  GTEST_SKIP() << "ODE collision is not available in this build";
#endif

  const std::string worldUri
      = "dart://sample/sdf/test/mimic_fast_slow_pendulums_world.sdf";

  auto retriever = std::make_shared<dart::utils::DartResourceRetriever>();
  dart::utils::SdfParser::Options options(retriever);

  WorldPtr world = dart::utils::SdfParser::readWorld(Uri(worldUri), options);
  ASSERT_TRUE(world);

  auto resource = retriever->retrieve(Uri(worldUri));
  ASSERT_TRUE(resource);
  std::string sdfText(resource->getSize(), '\0');
  const auto read = resource->read(sdfText.data(), 1, sdfText.size());
  ASSERT_EQ(read, resource->getSize());

  const auto specs = parseMimicSpecs(sdfText);
  ASSERT_FALSE(specs.empty());
  MimicTuning tuning;
  dart::constraint::MimicMotorConstraint::setConstraintForceMixing(tuning.cfm);
  dart::constraint::JointConstraint::setErrorReductionParameter(tuning.erp);

  configureMimicMotors(specs, world, tuning);
  setCollisionDetector(world, /*useOde=*/true);
  setBoxedSolver(world, /*usePgs=*/true);

  const auto baseline = world->getSkeleton("pendulum_with_base");
  const auto slowFollower
      = world->getSkeleton("pendulum_with_base_mimic_slow_follows_fast");
  const auto fastFollower
      = world->getSkeleton("pendulum_with_base_mimic_fast_follows_slow");
  ASSERT_TRUE(baseline);
  ASSERT_TRUE(slowFollower);
  ASSERT_TRUE(fastFollower);

  setJointStabilization(baseline->getJoint("slow_joint"), tuning);
  setJointStabilization(baseline->getJoint("fast_joint"), tuning);

  const Eigen::Vector3d baselineBaseStart
      = getTranslation(baseline->getBodyNode("base"));
  const Eigen::Vector3d slowBaseStart
      = getTranslation(slowFollower->getBodyNode("base"));
  const Eigen::Vector3d fastBaseStart
      = getTranslation(fastFollower->getBodyNode("base"));

  auto* baseSlowJoint = baseline->getJoint("slow_joint");
  auto* baseFastJoint = baseline->getJoint("fast_joint");
  auto* slowFollowJoint = slowFollower->getJoint("slow_joint");
  auto* fastFollowJoint = fastFollower->getJoint("fast_joint");
  ASSERT_NE(nullptr, baseSlowJoint);
  ASSERT_NE(nullptr, baseFastJoint);
  ASSERT_NE(nullptr, slowFollowJoint);
  ASSERT_NE(nullptr, fastFollowJoint);

  const double initSlowAbs = std::abs(baseSlowJoint->getPosition(0));
  const double initFastAbs = std::abs(baseFastJoint->getPosition(0));

  double maxDriftBase = 0.0;
  double maxDriftSlow = 0.0;
  double maxDriftFast = 0.0;

  double maxAbsSlow = initSlowAbs;
  double maxAbsFast = initFastAbs;
  double maxAbsSlowFollow = std::abs(slowFollowJoint->getPosition(0));
  double maxAbsFastFollow = std::abs(fastFollowJoint->getPosition(0));

  double maxAngleDiffBlue = 0.0;  // blue: fast follows slow baseline
  double maxVelDiffBlue = 0.0;
  double maxAngleDiffRed = 0.0;   // red: slow follows fast baseline
  double maxVelDiffRed = 0.0;

  const double driftTol = 1e-3;  // 1mm base drift tolerance
  const double angleTol = 7e-3;   // ~0.4 degrees
  const double velTol = 7e-2;     // 0.07 rad/s

  const int steps = 10000;
  for (int i = 0; i < steps; ++i) {
    world->step();
    ASSERT_TRUE(hasFiniteState(slowFollower));
    ASSERT_TRUE(hasFiniteState(fastFollower));

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

    EXPECT_LT(maxDriftSlow, driftTol);
    EXPECT_LT(maxDriftFast, driftTol);
    EXPECT_LT(maxDriftBase, driftTol);

    const double baseSlow = baseSlowJoint->getPosition(0);
    const double baseFast = baseFastJoint->getPosition(0);
    const double baseSlowVel = baseSlowJoint->getVelocity(0);
    const double baseFastVel = baseFastJoint->getVelocity(0);

    const double blueFast = fastFollowJoint->getPosition(0);
    const double blueFastVel = fastFollowJoint->getVelocity(0);
    const double redSlow = slowFollowJoint->getPosition(0);
    const double redSlowVel = slowFollowJoint->getVelocity(0);

    maxAbsSlow = std::max(maxAbsSlow, std::abs(baseSlow));
    maxAbsFast = std::max(maxAbsFast, std::abs(baseFast));
    maxAbsSlowFollow = std::max(maxAbsSlowFollow, std::abs(redSlow));
    maxAbsFastFollow = std::max(maxAbsFastFollow, std::abs(blueFast));

    maxAngleDiffBlue = std::max(
        maxAngleDiffBlue, std::abs(blueFast - baseSlow));
    maxVelDiffBlue = std::max(
        maxVelDiffBlue, std::abs(blueFastVel - baseSlowVel));
    maxAngleDiffRed = std::max(
        maxAngleDiffRed, std::abs(redSlow - baseFast));
    maxVelDiffRed = std::max(
        maxVelDiffRed, std::abs(redSlowVel - baseFastVel));

    EXPECT_LT(std::abs(blueFast - baseSlow), angleTol);
    EXPECT_LT(std::abs(redSlow - baseFast), angleTol);
    EXPECT_LT(std::abs(blueFastVel - baseSlowVel), velTol);
    EXPECT_LT(std::abs(redSlowVel - baseFastVel), velTol);
    EXPECT_LT(std::abs(baseSlow), tuning.positionLimit);
    EXPECT_LT(std::abs(baseFast), tuning.positionLimit);
    EXPECT_LT(std::abs(blueFast), tuning.positionLimit);
    EXPECT_LT(std::abs(redSlow), tuning.positionLimit);
  }

  // Bases should stay nearly stationary (free-floating but heavy).
  EXPECT_LT(maxDriftBase, 1e-3);
  EXPECT_LT(maxDriftSlow, 1e-3);
  EXPECT_LT(maxDriftFast, 1e-3);

  // Swing amplitude should stay within limits and not collapse to zero.
  EXPECT_LT(maxAbsSlow, 1.72);
  EXPECT_LT(maxAbsFast, 1.72);
  EXPECT_LT(maxAbsSlowFollow, 1.72);
  EXPECT_LT(maxAbsFastFollow, 1.72);
  EXPECT_GT(maxAbsSlow, 1.2);
  EXPECT_GT(maxAbsFast, 1.2);
  EXPECT_GT(maxAbsSlowFollow, 1.2);
  EXPECT_GT(maxAbsFastFollow, 1.2);

  // Mimic tracking against baseline joints (per-step tracking validated above).
  EXPECT_LT(maxAngleDiffBlue, 1e-2);
  EXPECT_LT(maxVelDiffBlue, 1e-1);
  EXPECT_LT(maxAngleDiffRed, 1e-2);
  EXPECT_LT(maxVelDiffRed, 1e-1);
}
