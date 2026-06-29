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

#include <dart/dynamics/ball_joint.hpp>
#include <dart/dynamics/body_node.hpp>
#include <dart/dynamics/detail/articulated_dynamics_algorithms.hpp>
#include <dart/dynamics/detail/skeleton_dynamics_view.hpp>
#include <dart/dynamics/free_joint.hpp>
#include <dart/dynamics/inertia.hpp>
#include <dart/dynamics/prismatic_joint.hpp>
#include <dart/dynamics/revolute_joint.hpp>
#include <dart/dynamics/skeleton.hpp>
#include <dart/dynamics/weld_joint.hpp>

#include <gtest/gtest.h>

#include <array>
#include <span>
#include <string>
#include <vector>

using namespace dart;
using namespace dart::dynamics;

namespace {

Eigen::VectorXd makeStateVector(int size, double start, double step)
{
  Eigen::VectorXd values(size);
  for (int i = 0; i < size; ++i) {
    values[i] = start + step * static_cast<double>(i);
  }
  return values;
}

BodyNode::Properties makeBodyProperties(const std::string& name, int index)
{
  BodyNode::Properties properties;
  properties.mName = name;
  properties.mInertia.setMass(1.0 + 0.1 * static_cast<double>(index));
  properties.mInertia.setMoment(
      (0.08 + 0.01 * static_cast<double>(index)) * Eigen::Matrix3d::Identity());
  properties.mInertia.setLocalCOM(
      Eigen::Vector3d(0.01 * index, -0.005 * index, 0.002 * index));
  return properties;
}

template <class Properties>
void configureJointProperties(Properties&, int)
{
  // Do nothing.
}

void configureJointProperties(RevoluteJoint::Properties& properties, int index)
{
  properties.mAxis
      = index % 2 == 0 ? Eigen::Vector3d::UnitZ() : Eigen::Vector3d::UnitY();
}

void configureJointProperties(PrismaticJoint::Properties& properties, int index)
{
  properties.mAxis
      = index % 2 == 0 ? Eigen::Vector3d::UnitX() : Eigen::Vector3d::UnitZ();
}

template <class JointType>
std::pair<JointType*, BodyNode*> addBody(
    const SkeletonPtr& skeleton,
    BodyNode* parent,
    const std::string& name,
    int index,
    const Eigen::Vector3d& offset)
{
  typename JointType::Properties jointProperties;
  jointProperties.mName = name + "_joint";
  configureJointProperties(jointProperties, index);

  auto pair = skeleton->createJointAndBodyNodePair<JointType>(
      parent, jointProperties, makeBodyProperties(name + "_body", index));
  pair.first->setTransformFromParentBodyNode(
      Eigen::Isometry3d(Eigen::Translation3d(offset)));
  return pair;
}

SkeletonPtr makeSerialSkeleton()
{
  auto skeleton = Skeleton::create("algorithms_serial");
  auto root = addBody<RevoluteJoint>(
      skeleton, nullptr, "serial_root", 0, Eigen::Vector3d(0.0, 0.0, 0.1));
  auto link1 = addBody<PrismaticJoint>(
      skeleton,
      root.second,
      "serial_prismatic",
      1,
      Eigen::Vector3d(0.2, 0.0, 0.0));
  auto link2 = addBody<BallJoint>(
      skeleton, link1.second, "serial_ball", 2, Eigen::Vector3d(0.0, 0.2, 0.0));
  auto link3 = addBody<WeldJoint>(
      skeleton, link2.second, "serial_weld", 3, Eigen::Vector3d(0.0, 0.0, 0.2));
  addBody<RevoluteJoint>(
      skeleton, link3.second, "serial_tip", 4, Eigen::Vector3d(0.1, 0.0, 0.2));
  return skeleton;
}

SkeletonPtr makeTreeSkeleton()
{
  auto skeleton = Skeleton::create("algorithms_tree");
  auto root = addBody<FreeJoint>(
      skeleton, nullptr, "tree_root", 0, Eigen::Vector3d(0.0, 0.0, 0.0));
  auto left = addBody<RevoluteJoint>(
      skeleton, root.second, "tree_left", 1, Eigen::Vector3d(0.2, 0.0, 0.1));
  addBody<BallJoint>(
      skeleton,
      left.second,
      "tree_left_leaf",
      2,
      Eigen::Vector3d(0.1, 0.2, 0.0));
  auto right = addBody<PrismaticJoint>(
      skeleton, root.second, "tree_right", 3, Eigen::Vector3d(-0.2, 0.0, 0.1));
  addBody<WeldJoint>(
      skeleton,
      right.second,
      "tree_right_weld",
      4,
      Eigen::Vector3d(0.0, 0.2, 0.1));
  return skeleton;
}

SkeletonPtr makeStarSkeleton()
{
  auto skeleton = Skeleton::create("algorithms_star");
  auto root = addBody<WeldJoint>(
      skeleton, nullptr, "star_root", 0, Eigen::Vector3d(0.0, 0.0, 0.0));
  addBody<RevoluteJoint>(
      skeleton,
      root.second,
      "star_revolute",
      1,
      Eigen::Vector3d(0.2, 0.0, 0.0));
  addBody<PrismaticJoint>(
      skeleton,
      root.second,
      "star_prismatic",
      2,
      Eigen::Vector3d(0.0, 0.2, 0.0));
  addBody<BallJoint>(
      skeleton, root.second, "star_ball", 3, Eigen::Vector3d(0.0, 0.0, 0.2));
  addBody<FreeJoint>(
      skeleton, root.second, "star_free", 4, Eigen::Vector3d(-0.2, 0.0, 0.0));
  return skeleton;
}

std::array<SkeletonPtr, 3> makeRepresentativeSkeletons()
{
  return {makeSerialSkeleton(), makeTreeSkeleton(), makeStarSkeleton()};
}

template <class SkeletonRange>
std::vector<detail::SkeletonDynamicsView> makeViews(
    const SkeletonRange& skeletons)
{
  std::vector<detail::SkeletonDynamicsView> models;
  models.reserve(skeletons.size());
  for (const auto& skeleton : skeletons) {
    models.emplace_back(*skeleton);
  }
  return models;
}

SkeletonPtr makeShortSkeleton()
{
  auto skeleton = Skeleton::create("algorithms_short");
  auto root = addBody<RevoluteJoint>(
      skeleton, nullptr, "short_root", 0, Eigen::Vector3d(0.0, 0.0, 0.1));
  addBody<PrismaticJoint>(
      skeleton, root.second, "short_tip", 1, Eigen::Vector3d(0.2, 0.0, 0.0));
  return skeleton;
}

std::vector<SkeletonPtr> makeHeterogeneousSkeletons()
{
  const auto representativeSkeletons = makeRepresentativeSkeletons();
  return {
      representativeSkeletons[0],
      representativeSkeletons[1],
      representativeSkeletons[2],
      makeShortSkeleton()};
}

void configureState(
    const SkeletonPtr& skeleton,
    bool withExternalForces,
    double stateScale = 1.0)
{
  const int dofs = static_cast<int>(skeleton->getNumDofs());
  skeleton->setTimeStep(0.002);
  skeleton->setGravity(Eigen::Vector3d(0.2, -0.3, -9.81));
  skeleton->setPositions(makeStateVector(dofs, 0.02 * stateScale, 0.01));
  skeleton->setVelocities(makeStateVector(dofs, -0.04 * stateScale, 0.015));
  skeleton->setAccelerations(makeStateVector(dofs, 0.08 * stateScale, -0.01));

  for (std::size_t jointIndex = 0; jointIndex < skeleton->getNumJoints();
       ++jointIndex) {
    auto* joint = skeleton->getJoint(jointIndex);
    for (std::size_t dofIndex = 0; dofIndex < joint->getNumDofs(); ++dofIndex) {
      const auto scale = static_cast<double>(1 + jointIndex + dofIndex);
      joint->setDampingCoefficient(dofIndex, 0.01 * scale);
      joint->setSpringStiffness(dofIndex, 0.015 * scale);
      joint->setRestPosition(dofIndex, -0.005 * scale);
    }
  }

  skeleton->clearExternalForces();
  if (withExternalForces) {
    for (std::size_t bodyIndex = 0; bodyIndex < skeleton->getNumBodyNodes();
         ++bodyIndex) {
      auto* body = skeleton->getBodyNode(bodyIndex);
      const auto scale = static_cast<double>(bodyIndex + 1);
      body->setExtForce(
          Eigen::Vector3d(0.1 * scale, -0.05 * scale, 0.03 * scale),
          Eigen::Vector3d(0.02 * scale, 0.01 * scale, -0.01 * scale));
      body->setExtTorque(
          Eigen::Vector3d(-0.02 * scale, 0.04 * scale, 0.01 * scale));
    }
  }
}

Eigen::VectorXd runDirectRnea(
    const SkeletonPtr& skeleton, const detail::RneaOptions& options)
{
  detail::SkeletonDynamicsView model(*skeleton);
  detail::rnea(model, options);
  return skeleton->getForces();
}

Eigen::VectorXd runDirectAba(const SkeletonPtr& skeleton)
{
  detail::SkeletonDynamicsView model(*skeleton);
  detail::aba(model);
  return skeleton->getAccelerations();
}

Eigen::VectorXd runLegacyRnea(
    const SkeletonPtr& skeleton, const detail::RneaOptions& options)
{
  detail::SkeletonDynamicsView model(*skeleton);
  if (model.getNumDofs() == 0) {
    return skeleton->getForces();
  }

  const auto numBodyNodes = model.getNumBodyNodes();
  const auto& gravity = model.getGravity();
  const auto timeStep = model.getTimeStep();
  for (std::size_t i = numBodyNodes; i > 0; --i) {
    auto& bodyNode = model.getBodyNode(i - 1);
    model.updateTransmittedForceID(
        bodyNode, gravity, options.mWithExternalForces);
    model.updateJointForceID(
        bodyNode,
        timeStep,
        options.mWithDampingForces,
        options.mWithSpringForces);
  }

  return skeleton->getForces();
}

Eigen::VectorXd runLegacyAba(const SkeletonPtr& skeleton)
{
  detail::SkeletonDynamicsView model(*skeleton);
  const auto numBodyNodes = model.getNumBodyNodes();
  const auto& gravity = model.getGravity();
  const auto timeStep = model.getTimeStep();

  for (std::size_t i = numBodyNodes; i > 0; --i) {
    model.updateBiasForce(model.getBodyNode(i - 1), gravity, timeStep);
  }

  for (std::size_t i = 0; i < numBodyNodes; ++i) {
    auto& bodyNode = model.getBodyNode(i);
    model.updateAccelerationFD(bodyNode);
    model.updateTransmittedForceFD(bodyNode);
    model.updateJointForceFD(bodyNode, timeStep, true, true);
  }

  return skeleton->getAccelerations();
}

} // namespace

TEST(ArticulatedDynamicsAlgorithms, DirectRneaMatchesLegacyRecursion)
{
  const detail::RneaOptions options{true, true, true};
  for (const auto& skeleton : makeRepresentativeSkeletons()) {
    configureState(skeleton, true);

    runLegacyRnea(skeleton, options);
    const auto expectedForces = skeleton->getForces();

    skeleton->setForces(
        Eigen::VectorXd::Constant(skeleton->getNumDofs(), 123.0));
    const auto directForces = runDirectRnea(skeleton, options);

    EXPECT_TRUE(directForces.isApprox(expectedForces, 1e-12))
        << skeleton->getName();
  }
}

TEST(ArticulatedDynamicsAlgorithms, DirectAbaMatchesLegacyRecursion)
{
  for (const auto& skeleton : makeRepresentativeSkeletons()) {
    configureState(skeleton, true);
    skeleton->setCommands(
        makeStateVector(static_cast<int>(skeleton->getNumDofs()), 0.2, -0.03));

    runLegacyAba(skeleton);
    const auto expectedAccelerations = skeleton->getAccelerations();

    skeleton->setAccelerations(
        Eigen::VectorXd::Constant(skeleton->getNumDofs(), -99.0));
    const auto directAccelerations = runDirectAba(skeleton);

    EXPECT_TRUE(directAccelerations.isApprox(expectedAccelerations, 1e-12))
        << skeleton->getName();
  }
}

TEST(ArticulatedDynamicsAlgorithms, BatchedRneaMatchesIndependentEntryPoints)
{
  const detail::RneaOptions options{true, true, true};
  auto skeletons = makeRepresentativeSkeletons();
  std::vector<Eigen::VectorXd> expectedForces;
  expectedForces.reserve(skeletons.size());

  for (std::size_t i = 0; i < skeletons.size(); ++i) {
    const auto& skeleton = skeletons[i];
    configureState(skeleton, true, 1.0 + static_cast<double>(i));
    runLegacyRnea(skeleton, options);
    expectedForces.push_back(skeleton->getForces());
    skeleton->setForces(
        Eigen::VectorXd::Constant(skeleton->getNumDofs(), 321.0));
  }

  auto models = makeViews(skeletons);
  detail::rneaBatch(
      std::span<detail::SkeletonDynamicsView>(models.data(), models.size()),
      options);

  for (std::size_t i = 0; i < skeletons.size(); ++i) {
    EXPECT_TRUE(skeletons[i]->getForces().isApprox(expectedForces[i], 1e-12))
        << skeletons[i]->getName();
  }
}

TEST(ArticulatedDynamicsAlgorithms, BatchedAbaMatchesIndependentEntryPoints)
{
  auto skeletons = makeRepresentativeSkeletons();
  std::vector<Eigen::VectorXd> expectedAccelerations;
  expectedAccelerations.reserve(skeletons.size());

  for (std::size_t i = 0; i < skeletons.size(); ++i) {
    const auto& skeleton = skeletons[i];
    configureState(skeleton, true, 1.0 + static_cast<double>(i));
    skeleton->setCommands(
        makeStateVector(static_cast<int>(skeleton->getNumDofs()), 0.2, -0.03));

    runLegacyAba(skeleton);
    expectedAccelerations.push_back(skeleton->getAccelerations());
    skeleton->setAccelerations(
        Eigen::VectorXd::Constant(skeleton->getNumDofs(), -321.0));
  }

  auto models = makeViews(skeletons);
  detail::abaBatch(
      std::span<detail::SkeletonDynamicsView>(models.data(), models.size()));

  for (std::size_t i = 0; i < skeletons.size(); ++i) {
    EXPECT_TRUE(
        skeletons[i]->getAccelerations().isApprox(
            expectedAccelerations[i], 1e-12))
        << skeletons[i]->getName();
  }
}

TEST(ArticulatedDynamicsAlgorithms, BatchedAlgorithmsHandleHeterogeneousModels)
{
  const detail::RneaOptions options{true, true, true};
  auto skeletons = makeHeterogeneousSkeletons();
  std::vector<Eigen::VectorXd> expectedForces;
  std::vector<Eigen::VectorXd> expectedAccelerations;
  expectedForces.reserve(skeletons.size());
  expectedAccelerations.reserve(skeletons.size());

  for (std::size_t i = 0; i < skeletons.size(); ++i) {
    const auto& skeleton = skeletons[i];
    configureState(skeleton, true, 1.0 + static_cast<double>(i));
    skeleton->setCommands(
        makeStateVector(static_cast<int>(skeleton->getNumDofs()), 0.2, -0.03));

    runLegacyRnea(skeleton, options);
    expectedForces.push_back(skeleton->getForces());

    skeleton->setForces(
        Eigen::VectorXd::Constant(skeleton->getNumDofs(), 987.0));
  }

  auto models = makeViews(skeletons);
  const auto modelSpan
      = std::span<detail::SkeletonDynamicsView>(models.data(), models.size());
  detail::rneaBatch(modelSpan, options);

  for (std::size_t i = 0; i < skeletons.size(); ++i) {
    EXPECT_TRUE(skeletons[i]->getForces().isApprox(expectedForces[i], 1e-12))
        << skeletons[i]->getName();
  }

  for (std::size_t i = 0; i < skeletons.size(); ++i) {
    const auto& skeleton = skeletons[i];
    configureState(skeleton, true, 1.0 + static_cast<double>(i));
    skeleton->setCommands(
        makeStateVector(static_cast<int>(skeleton->getNumDofs()), 0.2, -0.03));
    runLegacyAba(skeleton);
    expectedAccelerations.push_back(skeleton->getAccelerations());
    skeleton->setAccelerations(
        Eigen::VectorXd::Constant(skeleton->getNumDofs(), -987.0));
  }

  detail::abaBatch(modelSpan);

  for (std::size_t i = 0; i < skeletons.size(); ++i) {
    EXPECT_TRUE(
        skeletons[i]->getAccelerations().isApprox(
            expectedAccelerations[i], 1e-12))
        << skeletons[i]->getName();
  }
}

TEST(ArticulatedDynamicsAlgorithms, BatchedViewsUseUpdatedSkeletonTimeStep)
{
  const detail::RneaOptions rneaOptions{false, true, true};
  auto skeletons = makeRepresentativeSkeletons();

  for (std::size_t i = 0; i < skeletons.size(); ++i) {
    const auto& skeleton = skeletons[i];
    configureState(skeleton, false, 1.0 + static_cast<double>(i));
    skeleton->setCommands(
        makeStateVector(static_cast<int>(skeleton->getNumDofs()), 0.2, -0.03));
  }

  auto models = makeViews(skeletons);
  const auto modelSpan
      = std::span<detail::SkeletonDynamicsView>(models.data(), models.size());

  std::vector<double> updatedTimeSteps;
  std::vector<Eigen::VectorXd> expectedForces;
  updatedTimeSteps.reserve(skeletons.size());
  expectedForces.reserve(skeletons.size());

  for (std::size_t i = 0; i < skeletons.size(); ++i) {
    const auto& skeleton = skeletons[i];
    const double updatedTimeStep = 0.004 + 0.001 * static_cast<double>(i);
    skeleton->setTimeStep(updatedTimeStep);
    updatedTimeSteps.push_back(updatedTimeStep);

    EXPECT_DOUBLE_EQ(models[i].getTimeStep(), updatedTimeStep)
        << skeleton->getName();

    runLegacyRnea(skeleton, rneaOptions);
    expectedForces.push_back(skeleton->getForces());
    skeleton->setForces(
        Eigen::VectorXd::Constant(skeleton->getNumDofs(), 456.0));
  }

  detail::rneaBatch(modelSpan, rneaOptions);

  for (std::size_t i = 0; i < skeletons.size(); ++i) {
    EXPECT_TRUE(skeletons[i]->getForces().isApprox(expectedForces[i], 1e-12))
        << skeletons[i]->getName();
  }

  std::vector<Eigen::VectorXd> expectedAccelerations;
  expectedAccelerations.reserve(skeletons.size());

  for (std::size_t i = 0; i < skeletons.size(); ++i) {
    const auto& skeleton = skeletons[i];
    configureState(skeleton, true, 1.0 + static_cast<double>(i));
    skeleton->setCommands(
        makeStateVector(static_cast<int>(skeleton->getNumDofs()), 0.2, -0.03));
    skeleton->setTimeStep(updatedTimeSteps[i]);

    runLegacyAba(skeleton);
    expectedAccelerations.push_back(skeleton->getAccelerations());
    skeleton->setAccelerations(
        Eigen::VectorXd::Constant(skeleton->getNumDofs(), -456.0));
  }

  detail::abaBatch(modelSpan);

  for (std::size_t i = 0; i < skeletons.size(); ++i) {
    EXPECT_TRUE(
        skeletons[i]->getAccelerations().isApprox(
            expectedAccelerations[i], 1e-12))
        << skeletons[i]->getName();
  }
}

TEST(ArticulatedDynamicsAlgorithms, RneaAndAbaRoundTripRepresentativeJoints)
{
  for (const auto& skeleton : makeRepresentativeSkeletons()) {
    configureState(skeleton, false);
    const auto targetAccelerations = skeleton->getAccelerations();

    skeleton->computeInverseDynamics(false, true, true);
    const auto commands = skeleton->getForces();

    skeleton->setCommands(commands);
    skeleton->setAccelerations(Eigen::VectorXd::Zero(skeleton->getNumDofs()));
    skeleton->computeForwardDynamics();

    EXPECT_TRUE(
        skeleton->getAccelerations().isApprox(targetAccelerations, 1e-9))
        << skeleton->getName();
  }
}

TEST(ArticulatedDynamicsAlgorithms, ZeroDofSkeletonPathsAreNoOps)
{
  auto skeleton = Skeleton::create("algorithms_zero_dof");
  addBody<WeldJoint>(
      skeleton, nullptr, "zero_root", 0, Eigen::Vector3d(0.0, 0.0, 0.0));
  ASSERT_EQ(skeleton->getNumDofs(), 0u);

  detail::SkeletonDynamicsView model(*skeleton);
  detail::rnea(model, detail::RneaOptions{true, true, true});
  detail::aba(model);
  skeleton->computeInverseDynamics(true, true, true);
  skeleton->computeForwardDynamics();

  EXPECT_EQ(skeleton->getForces().size(), 0);
  EXPECT_EQ(skeleton->getAccelerations().size(), 0);
}

//==============================================================================
// A kinematic child joint (ACCELERATION actuator) makes its parent fold the
// child's articulated inertia in through GenericJoint's
// addChildArtInertia*ToKinematic paths -- the non-implicit variant via
// getArticulatedInertia() and the implicit variant via forward dynamics. The
// dynamic-joint tests above never reach those branches. Verify both run and
// stay finite.
TEST(ArticulatedDynamicsAlgorithms, KinematicChildJointArtInertiaPathsAreFinite)
{
  auto skeleton = Skeleton::create("algorithms_kinematic_child");
  auto root = addBody<RevoluteJoint>(
      skeleton, nullptr, "kin_root", 0, Eigen::Vector3d(0.0, 0.0, 0.1));
  auto child = addBody<RevoluteJoint>(
      skeleton, root.second, "kin_child", 1, Eigen::Vector3d(0.2, 0.0, 0.0));

  // Root stays dynamic (FORCE); the child becomes kinematic (ACCELERATION).
  child.first->setActuatorType(Joint::ACCELERATION);

  skeleton->setPositions((Eigen::VectorXd(2) << 0.3, -0.2).finished());
  skeleton->setVelocities((Eigen::VectorXd(2) << 0.1, -0.05).finished());
  // Force command for the dynamic root DoF; acceleration command for the
  // kinematic child DoF.
  skeleton->setCommands((Eigen::VectorXd(2) << 0.4, 0.7).finished());

  // Non-implicit kinematic articulated-inertia path.
  EXPECT_TRUE(root.second->getArticulatedInertia().allFinite());

  // Implicit kinematic articulated-inertia path (forward dynamics).
  skeleton->computeForwardDynamics();
  EXPECT_TRUE(skeleton->getAccelerations().allFinite());
  EXPECT_TRUE(skeleton->getForces().allFinite());
}
