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

#include <dart/gui/application.hpp>
#include <dart/gui/gizmo.hpp>
#include <dart/gui/panel.hpp>
#include <dart/gui/viewer.hpp>

#include <dart/simulation/world.hpp>

#include <dart/dynamics/body_node.hpp>
#include <dart/dynamics/frame.hpp>
#include <dart/dynamics/joint.hpp>
#include <dart/dynamics/simple_frame.hpp>
#include <dart/dynamics/skeleton.hpp>

#include <dart/math/constants.hpp>

#include <dart/io/read.hpp>

#include <Eigen/Geometry>

#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

constexpr const char* kRobotUri = "dart://sample/urdf/KR5/KR5 sixx R650.urdf";
constexpr const char* kGroundUri = "dart://sample/urdf/KR5/ground.urdf";
constexpr const char* kRobotSkeletonName = "KR5";
constexpr const char* kTargetFrameName = "operational_space_control_target";
constexpr const char* kGroundSkeletonName
    = "visual_operational_space_control_ground";

dart::dynamics::SkeletonPtr loadOperationalSpaceRobot()
{
  auto robot = dart::io::readSkeleton(kRobotUri);
  if (robot == nullptr) {
    throw std::runtime_error(
        "Failed to load operational-space robot from "
        + std::string(kRobotUri));
  }

  if (robot->getNumJoints() == 0 || robot->getJoint(0) == nullptr) {
    throw std::runtime_error("Operational-space KR5 is missing its root joint");
  }
  robot->getJoint(0)->setTransformFromParentBodyNode(
      Eigen::Isometry3d::Identity());
  robot->setName(kRobotSkeletonName);

  robot->eachJoint([](dart::dynamics::Joint* joint) {
    joint->setLimitEnforcement(false);
    if (joint->getNumDofs() > 0) {
      joint->setDampingCoefficient(0, 0.5);
    }
  });
  return robot;
}

dart::dynamics::SkeletonPtr loadOperationalSpaceGround()
{
  auto ground = dart::io::readSkeleton(kGroundUri);
  if (ground == nullptr || ground->getNumJoints() == 0
      || ground->getJoint(0) == nullptr) {
    throw std::runtime_error(
        "Failed to load operational-space ground from "
        + std::string(kGroundUri));
  }

  ground->setName(kGroundSkeletonName);
  Eigen::Isometry3d transform
      = ground->getJoint(0)->getTransformFromParentBodyNode();
  transform.pretranslate(Eigen::Vector3d(0.0, 0.0, 0.5));
  transform.rotate(
      Eigen::AngleAxisd(dart::math::pi / 2.0, Eigen::Vector3d::UnitX()));
  ground->getJoint(0)->setTransformFromParentBodyNode(transform);
  return ground;
}

class OperationalSpaceControlState
{
public:
  OperationalSpaceControlState(
      dart::dynamics::SkeletonPtr robot, dart::dynamics::SimpleFramePtr target)
    : mRobot(std::move(robot)),
      mEndEffector(
          mRobot && mRobot->getNumBodyNodes() > 0
              ? mRobot->getBodyNode(mRobot->getNumBodyNodes() - 1)
              : nullptr),
      mTarget(std::move(target)),
      mOffset(0.05, 0.0, 0.0)
  {
    if (mRobot == nullptr || mEndEffector == nullptr || mTarget == nullptr) {
      throw std::runtime_error(
          "Operational-space control is missing KR5, end-effector, or target");
    }

    const std::size_t dofs = mEndEffector->getNumDependentGenCoords();
    mKp.setZero();
    for (std::size_t i = 0; i < 3; ++i) {
      mKp(i, i) = 50.0;
    }
    mKd.setZero(dofs, dofs);
    for (std::size_t i = 0; i < dofs; ++i) {
      mKd(i, i) = 5.0;
    }

    Eigen::Isometry3d targetTransform = mEndEffector->getWorldTransform();
    targetTransform.pretranslate(mOffset);
    mTarget->setTransform(targetTransform);
    mOffset
        = mEndEffector->getWorldTransform().rotation().transpose() * mOffset;
  }

  void preStep()
  {
    const Eigen::MatrixXd mass = mRobot->getMassMatrix();
    const dart::math::LinearJacobian jacobian
        = mEndEffector->getLinearJacobian(mOffset);
    const Eigen::MatrixXd jacobianInverse
        = jacobian.transpose()
          * (jacobian * jacobian.transpose()
             + 0.0025 * Eigen::Matrix3d::Identity())
                .inverse();

    const dart::math::LinearJacobian jacobianDeriv
        = mEndEffector->getLinearJacobianDeriv(mOffset);
    const Eigen::MatrixXd jacobianDerivInverse
        = jacobianDeriv.transpose()
          * (jacobianDeriv * jacobianDeriv.transpose()
             + 0.0025 * Eigen::Matrix3d::Identity())
                .inverse();

    const Eigen::Vector3d positionError
        = mTarget->getWorldTransform().translation()
          - mEndEffector->getWorldTransform() * mOffset;
    const Eigen::Vector3d velocityError
        = -mEndEffector->getLinearVelocity(mOffset);
    const Eigen::VectorXd coriolisAndGravity
        = mRobot->getCoriolisAndGravityForces();

    const Eigen::VectorXd forces
        = mass
              * (jacobianInverse * mKp * velocityError
                 + jacobianDerivInverse * mKp * positionError)
          + coriolisAndGravity + mKd * jacobianInverse * mKp * positionError;
    mRobot->setForces(forces);
  }

private:
  dart::dynamics::SkeletonPtr mRobot;
  dart::dynamics::BodyNode* mEndEffector = nullptr;
  dart::dynamics::SimpleFramePtr mTarget;
  Eigen::Vector3d mOffset;
  Eigen::Matrix3d mKp;
  Eigen::MatrixXd mKd;
};

struct OperationalSpaceScene
{
  dart::simulation::WorldPtr world;
  std::shared_ptr<OperationalSpaceControlState> controller;
  std::vector<dart::gui::InverseKinematicsHandle> ikHandles;
  std::vector<dart::gui::Gizmo> gizmos;
};

OperationalSpaceScene createOperationalSpaceScene()
{
  OperationalSpaceScene scene;
  scene.world = dart::simulation::World::create("dartsim_operational_space");
  scene.world->setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  scene.world->addSkeleton(loadOperationalSpaceGround());

  auto robot = loadOperationalSpaceRobot();
  if (robot->getNumBodyNodes() == 0) {
    throw std::runtime_error(
        "Operational-space KR5 is missing an end-effector body node");
  }
  auto* endEffector = robot->getBodyNode(robot->getNumBodyNodes() - 1);

  Eigen::Isometry3d targetTransform = endEffector->getWorldTransform();
  targetTransform.pretranslate(Eigen::Vector3d(0.05, 0.0, 0.0));
  auto target = dart::dynamics::SimpleFrame::createShared(
      dart::dynamics::Frame::World(), kTargetFrameName, targetTransform);
  scene.world->addSimpleFrame(target);

  scene.controller
      = std::make_shared<OperationalSpaceControlState>(robot, target);
  dart::gui::InverseKinematicsHandle handle;
  handle.label = "1 operational-space target";
  handle.hotkey = '1';
  handle.target = target;
  scene.ikHandles.push_back(std::move(handle));

  dart::gui::Gizmo gizmo;
  gizmo.label = kTargetFrameName;
  gizmo.target = target;
  gizmo.size = 0.16;
  scene.gizmos.push_back(std::move(gizmo));

  scene.world->addSkeleton(robot);
  return scene;
}

dart::gui::RunOptions makeOperationalSpaceRunDefaults()
{
  dart::gui::RunOptions options;
  options.width = 640;
  options.height = 480;
  return options;
}

dart::gui::OrbitCamera makeOperationalSpaceCamera()
{
  dart::gui::OrbitCamera camera;
  camera.target = Eigen::Vector3d::Zero();
  camera.yaw = 0.8848934155088675;
  camera.pitch = 0.38410042777133657;
  camera.distance = 4.376539729055364;
  return camera;
}

void printOperationalSpaceInstructions()
{
  std::cout
      << "Operational Space Control Example Controls:\n"
      << "Left-drag the target gizmo arrows/planes/rings to move the "
         "operational-space target.\n"
      << "Press 1 to select the target for arrow/PageUp/PageDown nudges.\n"
      << "Space: Toggle simulation\n";
}

dart::gui::Panel createOperationalSpacePanel()
{
  dart::gui::Panel panel;
  panel.title = "Operational Space";
  panel.buildWithContext = [](dart::gui::PanelBuilder& builder,
                              dart::gui::PanelContext& context) {
    builder.text("KR5 operational-space target control");
    builder.text("Left-drag the target gizmo arrows/planes/rings.");
    builder.text("Press 1 to select it for keyboard nudges.");
    builder.text("Arrow keys and PageUp/PageDown nudge selected targets.");
    builder.separator();
    if (context.lifecycle != nullptr) {
      if (builder.button(context.lifecycle->paused ? "Resume" : "Pause")) {
        dart::gui::togglePaused(*context.lifecycle);
      }
      builder.sameLine();
      if (builder.button("Step")) {
        dart::gui::requestSingleStep(*context.lifecycle);
      }
    }
    builder.text("time: " + std::to_string(context.simulationTime));
    builder.text("selected: " + context.selectedLabel);
  };
  return panel;
}

} // namespace

int main(int argc, char* argv[])
{
  try {
    OperationalSpaceScene scene = createOperationalSpaceScene();

    dart::gui::ApplicationOptions options;
    options.world = scene.world;
    options.ikHandles = scene.ikHandles;
    options.gizmos = scene.gizmos;
    options.runDefaults = makeOperationalSpaceRunDefaults();
    options.camera = makeOperationalSpaceCamera();
    options.preStep = [controller = scene.controller]() {
      controller->preStep();
    };
    options.panels.push_back(createOperationalSpacePanel());

    printOperationalSpaceInstructions();
    return dart::gui::runApplication(argc, argv, options);
  } catch (const std::exception& e) {
    std::cerr << "operational_space_control: " << e.what() << "\n";
    return 1;
  }
}
