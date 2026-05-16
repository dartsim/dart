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

#include <dart/config.hpp>

#include <dart/gui/application.hpp>
#include <dart/gui/panel.hpp>
#include <dart/gui/viewer.hpp>

#include <dart/simulation/world.hpp>

#include <dart/dynamics/body_node.hpp>
#include <dart/dynamics/box_shape.hpp>
#include <dart/dynamics/degree_of_freedom.hpp>
#include <dart/dynamics/frame.hpp>
#include <dart/dynamics/joint.hpp>
#include <dart/dynamics/simple_frame.hpp>
#include <dart/dynamics/skeleton.hpp>
#include <dart/dynamics/sphere_shape.hpp>
#include <dart/dynamics/weld_joint.hpp>

#include <dart/common/uri.hpp>

#include <dart/io/read.hpp>

#include <Eigen/Geometry>

#include <array>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

namespace {

constexpr const char* kWamPackageName = "herb_description";
constexpr const char* kWamPackagePath = "urdf/wam";
constexpr const char* kWamUrdfPath = "urdf/wam/wam.urdf";
constexpr const char* kWamSkeletonName = "visual_operational_space_control_wam";
constexpr const char* kTargetFrameName = "operational_space_control_target";
constexpr const char* kGroundSkeletonName
    = "visual_operational_space_control_ground";

dart::dynamics::SkeletonPtr loadOperationalSpaceControlWamSkeleton()
{
  dart::io::ReadOptions options;
  options.addPackageDirectory(
      kWamPackageName, dart::config::dataPath(kWamPackagePath));
  const auto wamUri
      = dart::common::Uri::createFromPath(dart::config::dataPath(kWamUrdfPath));
  auto wam = dart::io::readSkeleton(wamUri, options);
  if (wam == nullptr) {
    throw std::runtime_error(
        "Failed to load operational-space WAM from " + wamUri.toString());
  }

  wam->setName(kWamSkeletonName);
  const std::array<std::pair<const char*, double>, 7> jointPositions{{
      {"/j1", 0.0},
      {"/j2", 0.0},
      {"/j3", 0.0},
      {"/j4", 0.0},
      {"/j5", 0.0},
      {"/j6", 0.0},
      {"/j7", 0.0},
  }};
  for (const auto& [name, position] : jointPositions) {
    auto* dof = wam->getDof(name);
    if (dof == nullptr) {
      throw std::runtime_error(
          "Operational-space WAM is missing expected DOF " + std::string(name));
    }
    dof->setPosition(position);
  }

  wam->eachJoint([](dart::dynamics::Joint* joint) {
    joint->setLimitEnforcement(false);
    for (std::size_t i = 0; i < joint->getNumDofs(); ++i) {
      joint->setDampingCoefficient(i, 0.5);
    }
  });
  return wam;
}

dart::dynamics::SkeletonPtr createGround()
{
  auto ground = dart::dynamics::Skeleton::create(kGroundSkeletonName);
  auto* body
      = ground->createJointAndBodyNodePair<dart::dynamics::WeldJoint>().second;
  auto* shapeNode = body->createShapeNodeWith<dart::dynamics::VisualAspect>(
      std::make_shared<dart::dynamics::BoxShape>(
          Eigen::Vector3d(5.0, 5.0, 0.01)));
  shapeNode->setRelativeTranslation(Eigen::Vector3d(0.0, 0.0, -0.005));
  shapeNode->getVisualAspect()->setRGBA(Eigen::Vector4d(0.18, 0.32, 0.58, 1.0));
  return ground;
}

class OperationalSpaceControlState
{
public:
  OperationalSpaceControlState(
      dart::dynamics::SkeletonPtr robot, dart::dynamics::SimpleFramePtr target)
    : mRobot(std::move(robot)),
      mEndEffector(mRobot ? mRobot->getBodyNode("/wam7") : nullptr),
      mTarget(std::move(target)),
      mOffset(0.05, 0.0, 0.0)
  {
    if (mRobot == nullptr || mEndEffector == nullptr || mTarget == nullptr) {
      throw std::runtime_error(
          "Operational-space control is missing WAM, end-effector, or target");
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
};

OperationalSpaceScene createOperationalSpaceScene()
{
  OperationalSpaceScene scene;
  scene.world = dart::simulation::World::create("dartsim_operational_space");
  scene.world->setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  scene.world->addSkeleton(createGround());

  auto wam = loadOperationalSpaceControlWamSkeleton();
  auto* endEffector = wam->getBodyNode("/wam7");
  if (endEffector == nullptr) {
    throw std::runtime_error(
        "Operational-space WAM is missing /wam7 body node");
  }

  Eigen::Isometry3d targetTransform = endEffector->getWorldTransform();
  targetTransform.pretranslate(Eigen::Vector3d(0.05, 0.0, 0.0));
  auto target = dart::dynamics::SimpleFrame::createShared(
      dart::dynamics::Frame::World(), kTargetFrameName, targetTransform);
  target->setShape(std::make_shared<dart::dynamics::SphereShape>(0.04));
  target->getVisualAspect(true)->setRGBA(
      Eigen::Vector4d(0.92, 0.08, 0.08, 1.0));
  scene.world->addSimpleFrame(target);

  scene.controller
      = std::make_shared<OperationalSpaceControlState>(wam, target);
  scene.world->addSkeleton(wam);
  return scene;
}

dart::gui::Panel createOperationalSpacePanel()
{
  dart::gui::Panel panel;
  panel.title = "Operational Space";
  panel.buildWithContext = [](dart::gui::PanelBuilder& builder,
                              dart::gui::PanelContext& context) {
    builder.text("WAM operational-space target control");
    builder.text("Drag or nudge the red target frame.");
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
    options.preStep = [controller = scene.controller]() {
      controller->preStep();
    };
    options.panels.push_back(createOperationalSpacePanel());

    return dart::gui::runApplication(argc, argv, options);
  } catch (const std::exception& e) {
    std::cerr << "operational_space_control: " << e.what() << "\n";
    return 1;
  }
}
