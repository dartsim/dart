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

#include <dart/dynamics/body_node.hpp>
#include <dart/dynamics/box_shape.hpp>
#include <dart/dynamics/frame.hpp>
#include <dart/dynamics/inertia.hpp>
#include <dart/dynamics/joint.hpp>
#include <dart/dynamics/line_segment_shape.hpp>
#include <dart/dynamics/revolute_joint.hpp>
#include <dart/dynamics/shape_node.hpp>
#include <dart/dynamics/simple_frame.hpp>
#include <dart/dynamics/skeleton.hpp>
#include <dart/dynamics/weld_joint.hpp>

#include <dart/math/constants.hpp>

#include <Eigen/Geometry>

#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

namespace {

struct Assembly
{
  dart::dynamics::SkeletonPtr skeleton;
  dart::dynamics::Joint* referenceJoint = nullptr;
  dart::dynamics::Joint* followerJoint = nullptr;
  dart::dynamics::BodyNode* referenceBody = nullptr;
  dart::dynamics::BodyNode* followerBody = nullptr;
};

struct CouplerState
{
  dart::simulation::WorldPtr world;
  dart::dynamics::Joint* couplerReference = nullptr;
  dart::dynamics::Joint* couplerFollower = nullptr;
  dart::dynamics::Joint* motorReference = nullptr;
  dart::dynamics::Joint* motorFollower = nullptr;
};

void addRodVisual(dart::dynamics::BodyNode* body, const Eigen::Vector3d& color)
{
  auto shape = std::make_shared<dart::dynamics::BoxShape>(
      Eigen::Vector3d(0.5, 0.05, 0.05));
  auto* shapeNode = body->createShapeNodeWith<
      dart::dynamics::VisualAspect,
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(shape);
  shapeNode->setRelativeTranslation(Eigen::Vector3d(0.25, 0.0, 0.0));
  shapeNode->getVisualAspect()->setColor(color);
}

Assembly createAssembly(
    const std::string& label,
    const Eigen::Vector3d& baseOffset,
    bool useCouplerConstraint,
    const Eigen::Vector3d& referenceColor,
    const Eigen::Vector3d& followerColor,
    double targetAngle)
{
  auto skeleton = dart::dynamics::Skeleton::create(label + "_rig");

  dart::dynamics::WeldJoint::Properties baseJointProps;
  baseJointProps.mName = label + "_base_joint";
  baseJointProps.mT_ParentBodyToJoint = Eigen::Translation3d(baseOffset);

  dart::dynamics::BodyNode::Properties baseBodyProps;
  baseBodyProps.mName = label + "_base";
  dart::dynamics::Inertia baseInertia;
  baseInertia.setMass(0.1);
  baseInertia.setMoment(1e-4, 1e-4, 1e-4, 0.0, 0.0, 0.0);
  baseBodyProps.mInertia = baseInertia;

  auto basePair
      = skeleton->createJointAndBodyNodePair<dart::dynamics::WeldJoint>(
          nullptr, baseJointProps, baseBodyProps);
  auto* baseBody = basePair.second;

  dart::dynamics::RevoluteJoint::Properties referenceJointProps;
  referenceJointProps.mName = label + "_reference_joint";
  referenceJointProps.mAxis = Eigen::Vector3d::UnitZ();
  referenceJointProps.mT_ParentBodyToJoint
      = Eigen::Translation3d(0.0, 0.15, 0.0);
  referenceJointProps.mT_ChildBodyToJoint
      = Eigen::Translation3d(-0.25, 0.0, 0.0);

  dart::dynamics::BodyNode::Properties referenceBodyProps;
  referenceBodyProps.mName = label + "_reference_body";
  dart::dynamics::Inertia referenceInertia;
  referenceInertia.setMass(1.0);
  referenceInertia.setMoment(0.02, 0.02, 0.03, 0.0, 0.0, 0.0);
  referenceBodyProps.mInertia = referenceInertia;

  auto referencePair
      = skeleton->createJointAndBodyNodePair<dart::dynamics::RevoluteJoint>(
          baseBody, referenceJointProps, referenceBodyProps);
  auto* referenceJoint = referencePair.first;
  auto* referenceBody = referencePair.second;
  referenceJoint->setActuatorType(dart::dynamics::Joint::FORCE);
  referenceJoint->setForceLowerLimit(0, -120.0);
  referenceJoint->setForceUpperLimit(0, 120.0);
  referenceJoint->setDampingCoefficient(0, 0.02);
  addRodVisual(referenceBody, referenceColor);

  dart::dynamics::RevoluteJoint::Properties followerJointProps;
  followerJointProps.mName = label + "_follower_joint";
  followerJointProps.mAxis = Eigen::Vector3d::UnitZ();
  followerJointProps.mT_ParentBodyToJoint
      = Eigen::Translation3d(0.0, -0.15, 0.0);
  followerJointProps.mT_ChildBodyToJoint
      = Eigen::Translation3d(-0.25, 0.0, 0.0);

  dart::dynamics::BodyNode::Properties followerBodyProps;
  followerBodyProps.mName = label + "_follower_body";
  dart::dynamics::Inertia followerInertia;
  followerInertia.setMass(1.0);
  followerInertia.setMoment(0.02, 0.02, 0.03, 0.0, 0.0, 0.0);
  followerBodyProps.mInertia = followerInertia;

  auto followerPair
      = skeleton->createJointAndBodyNodePair<dart::dynamics::RevoluteJoint>(
          baseBody, followerJointProps, followerBodyProps);
  auto* followerJoint = followerPair.first;
  auto* followerBody = followerPair.second;
  followerJoint->setActuatorType(dart::dynamics::Joint::MIMIC);
  followerJoint->setMimicJoint(referenceJoint, -1.0, 0.0);
  followerJoint->setUseCouplerConstraint(useCouplerConstraint);
  followerJoint->setForceLowerLimit(0, -120.0);
  followerJoint->setForceUpperLimit(0, 120.0);
  followerJoint->setDampingCoefficient(0, 0.02);
  followerJoint->setPositionLowerLimit(0, -0.35);
  followerJoint->setPositionUpperLimit(0, 0.35);
  followerJoint->setLimitEnforcement(true);
  addRodVisual(followerBody, followerColor);

  referenceJoint->setPosition(0, targetAngle);
  followerJoint->setPosition(0, followerJoint->getPositionLowerLimit(0));

  return {skeleton, referenceJoint, followerJoint, referenceBody, followerBody};
}

void addLimitGuide(
    dart::simulation::World& world,
    const std::string& name,
    dart::dynamics::Joint* followerJoint,
    double angle,
    const Eigen::Vector3d& color)
{
  auto* revolute = dynamic_cast<dart::dynamics::RevoluteJoint*>(followerJoint);
  if (revolute == nullptr) {
    return;
  }

  auto frame = dart::dynamics::SimpleFrame::createShared(
      dart::dynamics::Frame::World(), name);
  auto line = std::make_shared<dart::dynamics::LineSegmentShape>(0.05f);
  line->addVertex(Eigen::Vector3d::Zero());
  line->addVertex(Eigen::Vector3d::UnitX() * 0.65);
  line->addConnection(0, 1);
  frame->setShape(line);
  frame->createVisualAspect()->setColor(color);

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  if (const auto* parent = followerJoint->getParentBodyNode()) {
    transform = parent->getWorldTransform();
  }
  transform = transform * followerJoint->getTransformFromParentBodyNode();
  transform.rotate(Eigen::AngleAxisd(angle, revolute->getAxis()));
  transform.translate(Eigen::Vector3d(0.0, 0.0, 0.02));
  frame->setRelativeTransform(transform);
  world.addSimpleFrame(frame);
}

void addPairLink(
    dart::simulation::World& world,
    const std::string& name,
    float width,
    const Eigen::Vector3d& color,
    const Assembly& assembly)
{
  auto frame = dart::dynamics::SimpleFrame::createShared(
      dart::dynamics::Frame::World(), name);
  auto line = std::make_shared<dart::dynamics::LineSegmentShape>(width);
  line->addVertex(assembly.referenceBody->getWorldTransform().translation());
  line->addVertex(assembly.followerBody->getWorldTransform().translation());
  line->addConnection(0, 1);
  frame->setShape(line);
  frame->createVisualAspect()->setColor(color);
  world.addSimpleFrame(frame);
}

std::shared_ptr<CouplerState> createCouplerState()
{
  constexpr double targetAngle = 45.0 * dart::math::pi / 180.0;

  auto state = std::make_shared<CouplerState>();
  state->world = dart::simulation::World::create("coupler_constraint");
  state->world->setGravity(Eigen::Vector3d::Zero());
  state->world->setTimeStep(1e-3);

  auto coupler = createAssembly(
      "coupler",
      Eigen::Vector3d(-0.45, 0.0, 0.0),
      true,
      Eigen::Vector3d(0.85, 0.35, 0.25),
      Eigen::Vector3d(0.25, 0.58, 0.92),
      targetAngle);
  auto motor = createAssembly(
      "motor",
      Eigen::Vector3d(0.45, 0.0, 0.0),
      false,
      Eigen::Vector3d(0.68, 0.32, 0.70),
      Eigen::Vector3d(0.25, 0.75, 0.70),
      targetAngle);

  state->couplerReference = coupler.referenceJoint;
  state->couplerFollower = coupler.followerJoint;
  state->motorReference = motor.referenceJoint;
  state->motorFollower = motor.followerJoint;

  state->world->addSkeleton(coupler.skeleton);
  state->world->addSkeleton(motor.skeleton);

  addLimitGuide(
      *state->world,
      "coupler_lower_limit",
      coupler.followerJoint,
      coupler.followerJoint->getPositionLowerLimit(0),
      Eigen::Vector3d(0.92, 0.35, 0.35));
  addLimitGuide(
      *state->world,
      "coupler_upper_limit",
      coupler.followerJoint,
      coupler.followerJoint->getPositionUpperLimit(0),
      Eigen::Vector3d(0.35, 0.92, 0.35));
  addLimitGuide(
      *state->world,
      "motor_lower_limit",
      motor.followerJoint,
      motor.followerJoint->getPositionLowerLimit(0),
      Eigen::Vector3d(0.92, 0.35, 0.35));
  addLimitGuide(
      *state->world,
      "motor_upper_limit",
      motor.followerJoint,
      motor.followerJoint->getPositionUpperLimit(0),
      Eigen::Vector3d(0.35, 0.92, 0.35));
  addPairLink(
      *state->world,
      "coupler_link",
      0.06f,
      Eigen::Vector3d(0.95, 0.95, 0.2),
      coupler);
  addPairLink(
      *state->world,
      "motor_link",
      0.04f,
      Eigen::Vector3d(0.75, 0.75, 0.9),
      motor);

  return state;
}

void setReferenceAngle(CouplerState& state, double degrees)
{
  const double radians = degrees * dart::math::pi / 180.0;
  if (state.couplerReference != nullptr) {
    state.couplerReference->setPosition(0, radians);
  }
  if (state.motorReference != nullptr) {
    state.motorReference->setPosition(0, radians);
  }
}

dart::gui::Panel createControlsPanel(const std::shared_ptr<CouplerState>& state)
{
  double targetDegrees = 45.0;

  dart::gui::Panel panel;
  panel.title = "Coupler Constraint";
  panel.buildWithContext = [state, targetDegrees](
                               dart::gui::PanelBuilder& builder,
                               dart::gui::PanelContext& context) mutable {
    builder.text("Left: mimic joint with coupler constraint.");
    builder.text("Right: motor-style mimic without coupler constraint.");
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
    if (builder.slider("Reference angle", targetDegrees, -60.0, 60.0)) {
      setReferenceAngle(*state, targetDegrees);
    }
    if (builder.button("Reset")) {
      targetDegrees = 45.0;
      setReferenceAngle(*state, targetDegrees);
    }
    if (state->couplerFollower != nullptr && state->motorFollower != nullptr) {
      builder.text(
          "coupler follower: "
          + std::to_string(state->couplerFollower->getPosition(0)));
      builder.text(
          "motor follower: "
          + std::to_string(state->motorFollower->getPosition(0)));
    }
    builder.text("time: " + std::to_string(context.simulationTime));
  };
  return panel;
}

} // namespace

int main(int argc, char* argv[])
{
  try {
    auto state = createCouplerState();

    dart::gui::ApplicationOptions options;
    options.world = state->world;
    options.panels.push_back(createControlsPanel(state));
    return dart::gui::runApplication(argc, argv, options);
  } catch (const std::exception& e) {
    std::cerr << "coupler_constraint: " << e.what() << "\n";
    return 1;
  }
}
