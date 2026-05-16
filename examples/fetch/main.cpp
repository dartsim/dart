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
#include <dart/gui/panel.hpp>
#include <dart/gui/viewer.hpp>

#include <dart/simulation/world.hpp>

#include <dart/constraint/constraint_solver.hpp>
#include <dart/constraint/weld_joint_constraint.hpp>

#include <dart/dynamics/body_node.hpp>
#include <dart/dynamics/degree_of_freedom.hpp>
#include <dart/dynamics/frame.hpp>
#include <dart/dynamics/joint.hpp>
#include <dart/dynamics/simple_frame.hpp>
#include <dart/dynamics/skeleton.hpp>
#include <dart/dynamics/sphere_shape.hpp>

#include <dart/io/read.hpp>

#include <Eigen/Geometry>

#include <iostream>
#include <memory>
#include <string>

namespace {

constexpr const char* kFetchWorldUri
    = "dart://sample/mjcf/openai/robotics/fetch/pick_and_place.xml";
constexpr const char* kFetchRobotName = "robot0:base_link";
constexpr const char* kFetchObjectName = "object0";
constexpr const char* kFetchMocapName = "robot0:mocap";
constexpr const char* kFetchTargetName = "fetch_target";

struct FetchScene
{
  dart::simulation::WorldPtr world;
  std::shared_ptr<dart::dynamics::SimpleFrame> target;
  dart::dynamics::BodyNode* mocapRoot = nullptr;
};

Eigen::Isometry3d makeTargetTransform()
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(1.3, 0.75, 0.50);
  transform.linear()
      = Eigen::AngleAxisd(1.5707963267948966, Eigen::Vector3d::UnitY())
            .toRotationMatrix();
  return transform;
}

void configureRobot(const dart::dynamics::SkeletonPtr& robot)
{
  robot->getJoint(0)->setActuatorType(
      dart::dynamics::Joint::ActuatorType::LOCKED);
  robot->eachDof([](dart::dynamics::DegreeOfFreedom* dof) {
    dof->setSpringStiffness(1e+3);
  });

  if (auto* torso = robot->getJoint("robot0:torso_lift_joint")) {
    torso->setSpringStiffness(0, 1e+7);
  }

  robot->setPosition(0, 0.405);
  robot->setPosition(1, 0.480);
  robot->setPosition(2, 0.000);
  robot->setPosition(6, 0.01);
  robot->setPosition(7, -0.73);
  robot->setPosition(8, 0.00);
  robot->setPosition(9, 1.64);
  robot->setPosition(10, 0.0);
  robot->setPosition(11, 0.66);
  robot->setPosition(12, 0.01);
}

void configureObject(const dart::dynamics::SkeletonPtr& object)
{
  object->setPosition(3, 1.25);
  object->setPosition(4, 0.53);
  object->setPosition(5, 0.40);
}

void resetFirstWeldConstraint(const dart::simulation::WorldPtr& world)
{
  auto* constraintSolver = world->getConstraintSolver();
  if (constraintSolver == nullptr
      || constraintSolver->getNumConstraints() == 0) {
    return;
  }

  auto weldJointConstraint
      = std::dynamic_pointer_cast<dart::constraint::WeldJointConstraint>(
          constraintSolver->getConstraint(0));
  if (weldJointConstraint != nullptr) {
    weldJointConstraint->setRelativeTransform(Eigen::Isometry3d::Identity());
  }
}

std::shared_ptr<dart::dynamics::SimpleFrame> createTargetFrame()
{
  auto target = dart::dynamics::SimpleFrame::createShared(
      dart::dynamics::Frame::World(), kFetchTargetName, makeTargetTransform());
  target->setShape(std::make_shared<dart::dynamics::SphereShape>(0.06));
  target->getVisualAspect(true)->setRGBA(
      Eigen::Vector4d(0.18, 0.86, 0.34, 0.92));
  return target;
}

FetchScene createFetchScene()
{
  FetchScene scene;
  scene.world = dart::io::readWorld(kFetchWorldUri);
  if (scene.world == nullptr) {
    throw std::runtime_error("Failed to load Fetch pick-and-place MJCF world");
  }
  scene.world->setGravity(Eigen::Vector3d::Zero());

  auto robot = scene.world->getSkeleton(kFetchRobotName);
  if (robot == nullptr) {
    throw std::runtime_error("Fetch world is missing robot0:base_link");
  }
  configureRobot(robot);

  auto object = scene.world->getSkeleton(kFetchObjectName);
  if (object == nullptr) {
    throw std::runtime_error("Fetch world is missing object0");
  }
  configureObject(object);

  auto mocap = scene.world->getSkeleton(kFetchMocapName);
  if (mocap == nullptr || mocap->getRootBodyNode() == nullptr) {
    throw std::runtime_error("Fetch world is missing robot0:mocap");
  }
  scene.mocapRoot = mocap->getRootBodyNode();
  resetFirstWeldConstraint(scene.world);

  scene.target = createTargetFrame();
  scene.world->addSimpleFrame(scene.target);
  return scene;
}

void syncMocapTarget(const FetchScene& scene)
{
  if (scene.mocapRoot == nullptr || scene.target == nullptr) {
    return;
  }

  auto* joint = scene.mocapRoot->getParentJoint();
  if (joint != nullptr) {
    joint->setTransformFromParentBodyNode(scene.target->getTransform());
  }
}

dart::gui::Panel createFetchPanel()
{
  dart::gui::Panel panel;
  panel.title = "Fetch";
  panel.buildWithContext = [](dart::gui::PanelBuilder& builder,
                              dart::gui::PanelContext& context) {
    builder.text("Fetch pick-and-place MJCF world");
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
    builder.text("Move the green target to drive the mocap body.");
    builder.text("time: " + std::to_string(context.simulationTime));
    builder.text("contacts: " + std::to_string(context.contactCount));
    builder.text("selected: " + context.selectedLabel);
  };
  return panel;
}

} // namespace

int main(int argc, char* argv[])
{
  try {
    FetchScene scene = createFetchScene();

    dart::gui::ApplicationOptions options;
    options.world = scene.world;
    options.preStep = [scene]() {
      syncMocapTarget(scene);
    };
    options.panels.push_back(createFetchPanel());

    return dart::gui::runApplication(argc, argv, options);
  } catch (const std::exception& e) {
    std::cerr << "fetch: " << e.what() << "\n";
    return 1;
  }
}
