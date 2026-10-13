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
 *     copyright notice, this list of conditions and the following disclaimer
 *     in the documentation and/or other materials provided with the
 *     distribution.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "demo_models/OperationalSpaceController.hpp"

#include <dart/simulation/World.hpp>

#include <dart/dynamics/BodyNode.hpp>
#include <dart/dynamics/RevoluteJoint.hpp>

#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>

namespace {

struct Model
{
  dart::simulation::WorldPtr world = dart::simulation::World::create();
  dart::dynamics::SkeletonPtr robot = dart::dynamics::Skeleton::create("arm");
  dart::dynamics::SimpleFramePtr target;

  Model()
  {
    dart::dynamics::BodyNode* parent = nullptr;
    const Eigen::Vector3d axes[]
        = {Eigen::Vector3d::UnitZ(),
           Eigen::Vector3d::UnitY(),
           Eigen::Vector3d::UnitX()};
    for (const auto& axis : axes) {
      auto pair
          = robot->createJointAndBodyNodePair<dart::dynamics::RevoluteJoint>(
              parent);
      pair.first->setAxis(axis);
      Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
      tf.translation() = Eigen::Vector3d(0.2, 0, 0.1);
      pair.first->setTransformFromParentBodyNode(tf);
      pair.first->setDampingCoefficient(0, 0.5);
      parent = pair.second;
    }
    robot->setPositions(Eigen::Vector3d(0.1, 0.2, -0.3));
    world->addSkeleton(robot);
    Eigen::Isometry3d tf = parent->getWorldTransform();
    tf.translation() += Eigen::Vector3d(0.1, 0.05, 0.05);
    target = std::make_shared<dart::dynamics::SimpleFrame>(
        dart::dynamics::Frame::World(), "target", tf);
    world->addSimpleFrame(target);
  }
};

} // namespace

//==============================================================================
TEST(OperationalSpaceController, GravityAndTargetTracking)
{
  Model model;
  dart_demos::OperationalSpaceController controller(model.robot, model.target);
  EXPECT_EQ(controller.getRobot(), model.robot);
  EXPECT_EQ(controller.getTarget(), model.target);
  EXPECT_TRUE(controller.getKp().isApprox(50 * Eigen::Matrix3d::Identity()));
  EXPECT_TRUE(controller.getKd().isApprox(5 * Eigen::Matrix3d::Identity()));

  controller.setKp(Eigen::Matrix3d::Zero());
  EXPECT_TRUE(controller.computeForces().isApprox(
      model.robot->getCoriolisAndGravityForces(), 1e-14));
  controller.update();
  EXPECT_TRUE(model.robot->getForces().isApprox(controller.computeForces()));

  controller.setKp(50 * Eigen::Matrix3d::Identity());
  const Eigen::VectorXd first = controller.computeForces();
  ASSERT_TRUE(first.allFinite());
  Eigen::Isometry3d tf = model.target->getWorldTransform();
  tf.translation() += Eigen::Vector3d(0.03, -0.04, 0.02);
  model.target->setTransform(tf);
  EXPECT_GT((controller.computeForces() - first).norm(), 1e-6);
  controller.setOffset(Eigen::Vector3d(0, 0, 0.1));
  EXPECT_GT((controller.computeForces() - first).norm(), 1e-6);

  auto target = std::make_shared<dart::dynamics::SimpleFrame>(
      dart::dynamics::Frame::World(), "replacement", tf);
  controller.setTarget(target);
  EXPECT_EQ(controller.getTarget(), target);
  controller.setKd(3 * Eigen::Matrix3d::Identity());
  EXPECT_TRUE(controller.getKd().isApprox(3 * Eigen::Matrix3d::Identity()));
}

//==============================================================================
TEST(OperationalSpaceController, RepeatsControlledStepsDeterministically)
{
  Model first;
  Model second;
  dart_demos::OperationalSpaceController left(first.robot, first.target);
  dart_demos::OperationalSpaceController right(second.robot, second.target);
  for (int i = 0; i < 40; ++i) {
    left.update();
    right.update();
    first.world->step();
    second.world->step();
    ASSERT_TRUE(first.robot->getPositions().allFinite());
    ASSERT_TRUE(second.robot->getPositions().allFinite());
  }
  EXPECT_EQ(first.robot->getPositions(), second.robot->getPositions());
  EXPECT_EQ(first.robot->getVelocities(), second.robot->getVelocities());
}

//==============================================================================
TEST(OperationalSpaceController, RejectsInvalidInputs)
{
  Model model;
  EXPECT_THROW(
      dart_demos::OperationalSpaceController(nullptr, model.target),
      std::invalid_argument);
  EXPECT_THROW(
      dart_demos::OperationalSpaceController(
          dart::dynamics::Skeleton::create(), model.target),
      std::invalid_argument);
  EXPECT_THROW(
      dart_demos::OperationalSpaceController(model.robot, nullptr),
      std::invalid_argument);
  dart_demos::OperationalSpaceController controller(model.robot, model.target);
  EXPECT_THROW(controller.setTarget(nullptr), std::invalid_argument);
  EXPECT_THROW(
      controller.setKd(Eigen::MatrixXd::Zero(2, 2)), std::invalid_argument);
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(
      controller.setOffset(Eigen::Vector3d(nan, 0, 0)), std::invalid_argument);
  EXPECT_THROW(
      controller.setKp(Eigen::Matrix3d::Constant(nan)), std::invalid_argument);
  EXPECT_THROW(
      controller.setKd(Eigen::Matrix3d::Constant(nan)), std::invalid_argument);
}

//==============================================================================
TEST(OperationalSpaceController, RejectsRemovedEndEffector)
{
  Model model;
  dart_demos::OperationalSpaceController controller(model.robot, model.target);
  auto detached
      = model.robot->getBodyNode(model.robot->getNumBodyNodes() - 1)->remove();
  detached.reset();
  EXPECT_THROW(controller.computeForces(), std::runtime_error);
  EXPECT_THROW(controller.update(), std::runtime_error);
}

//==============================================================================
TEST(OperationalSpaceController, RejectsChangedDofCount)
{
  Model model;
  dart_demos::OperationalSpaceController controller(model.robot, model.target);
  model.robot->createJointAndBodyNodePair<dart::dynamics::RevoluteJoint>();
  EXPECT_THROW(controller.computeForces(), std::runtime_error);
}
