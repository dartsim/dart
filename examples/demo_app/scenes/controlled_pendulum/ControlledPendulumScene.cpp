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

#include "scenes/controlled_pendulum/ControlledPendulumScene.hpp"

#include <dart/gui/osg/IncludeImGui.hpp>

#include <dart/simulation/World.hpp>

#include <dart/dynamics/BodyNode.hpp>
#include <dart/dynamics/BoxShape.hpp>
#include <dart/dynamics/CapsuleShape.hpp>
#include <dart/dynamics/Inertia.hpp>
#include <dart/dynamics/RevoluteJoint.hpp>
#include <dart/dynamics/SimpleFrame.hpp>
#include <dart/dynamics/Skeleton.hpp>
#include <dart/dynamics/SphereShape.hpp>
#include <dart/dynamics/WeldJoint.hpp>

#include <cmath>

namespace dart::demo {

namespace {

constexpr double kDegToRad = 3.14159265358979323846 / 180.0;

} // namespace

ControlledPendulumScene::ControlledPendulumScene()
{
  mMetadata.key = "controlled_pendulum";
  mMetadata.name = "PD-controlled pendulum";
  mMetadata.description
      = "One-degree-of-freedom pendulum stabilized by a PD controller. Adjust "
        "the set point and gains to see how the arm responds.";
  mMetadata.tags = {"controllers", "dynamics"};
  mMetadata.eye = osg::Vec3d(3.0, 2.0, 1.8);
  mMetadata.center = osg::Vec3d(0.0, 0.0, 1.0);

  buildScene();

  mNode = new HookedWorldNode(mWorld, [this](double dt) { this->step(dt); });

  reset();
}

void ControlledPendulumScene::buildScene()
{
  mWorld = dart::simulation::World::create("controlled_pendulum_world");
  mWorld->setGravity(Eigen::Vector3d(0, 0, -9.81));
  mWorld->setTimeStep(0.001);

  createGround();

  using namespace dart::dynamics;

  mPendulum = Skeleton::create("pendulum");

  RevoluteJoint::Properties jointProps;
  jointProps.mName = "shoulder";
  jointProps.mAxis = Eigen::Vector3d::UnitY();
  jointProps.mT_ParentBodyToJoint.translation()
      = Eigen::Vector3d(0.0, 0.0, 1.0);

  BodyNode::AspectProperties bodyProps;
  bodyProps.mName = "arm";

  auto pair = mPendulum->createJointAndBodyNodePair<RevoluteJoint>(
      nullptr, jointProps, bodyProps);

  mShoulder = pair.first;
  auto* body = pair.second;

  const double radius = 0.08;
  auto armShape = std::make_shared<CapsuleShape>(radius, mArmLength);
  auto armNode = body->createShapeNodeWith<
      VisualAspect,
      CollisionAspect,
      DynamicsAspect>(armShape);
  Eigen::Isometry3d armOffset = Eigen::Isometry3d::Identity();
  armOffset.translation() = Eigen::Vector3d(0.0, 0.0, -mArmLength * 0.5);
  armNode->setRelativeTransform(armOffset);
  armNode->getVisualAspect()->setColor(Eigen::Vector3d(0.2, 0.6, 0.9));

  auto endSphere = std::make_shared<SphereShape>(radius * 1.5);
  auto endNode = body->createShapeNodeWith<
      VisualAspect,
      CollisionAspect,
      DynamicsAspect>(endSphere);
  Eigen::Isometry3d endOffset = Eigen::Isometry3d::Identity();
  endOffset.translation() = Eigen::Vector3d(0.0, 0.0, -mArmLength);
  endNode->setRelativeTransform(endOffset);
  endNode->getVisualAspect()->setColor(Eigen::Vector3d(0.9, 0.6, 0.2));

  Inertia inertia;
  inertia.setMass(1.5);
  inertia.setMoment(armShape->computeInertia(1.5));
  body->setInertia(inertia);

  mWorld->addSkeleton(mPendulum);

  mTargetFrame = dart::dynamics::SimpleFrame::createShared(
      dart::dynamics::Frame::World(), "target_frame");
  auto targetShape = std::make_shared<SphereShape>(0.05);
  mTargetFrame->createVisualAspect();
  mTargetFrame->setShape(targetShape);
  if (auto* visual = mTargetFrame->getVisualAspect())
    visual->setColor(Eigen::Vector3d(0.8, 0.15, 0.15));
  mWorld->addSimpleFrame(mTargetFrame);
}

void ControlledPendulumScene::createGround()
{
  using namespace dart::dynamics;

  auto ground = Skeleton::create("ground");
  auto pair = ground->createJointAndBodyNodePair<WeldJoint>();
  auto* joint = pair.first;
  auto* body = pair.second;

  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = Eigen::Vector3d(0.0, 0.0, -0.05);
  joint->setTransformFromParentBodyNode(tf);

  const Eigen::Vector3d size(6.0, 6.0, 0.1);
  auto shape = std::make_shared<BoxShape>(size);
  auto node = body->createShapeNodeWith<
      VisualAspect,
      CollisionAspect,
      DynamicsAspect>(shape);
  node->getVisualAspect()->setColor(Eigen::Vector3d(0.75, 0.75, 0.75));

  mWorld->addSkeleton(ground);
}

const SceneMetadata& ControlledPendulumScene::metadata() const
{
  return mMetadata;
}

dart::simulation::WorldPtr ControlledPendulumScene::world() const
{
  return mWorld;
}

osg::ref_ptr<dart::gui::osg::RealTimeWorldNode> ControlledPendulumScene::node()
{
  return mNode;
}

void ControlledPendulumScene::reset()
{
  if (!mWorld)
    return;

  mWorld->reset();
  mTargetAngle = mHomeAngle;

  if (mShoulder) {
    mShoulder->setPosition(0, mHomeAngle);
    mShoulder->setVelocity(0, 0.0);
    mShoulder->setForce(0, 0.0);
  }

  updateTargetFrame();
}

void ControlledPendulumScene::updateTargetFrame()
{
  if (!mTargetFrame)
    return;

  const double x = std::sin(mTargetAngle) * mArmLength;
  const double z = 1.0 - std::cos(mTargetAngle) * mArmLength;

  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = Eigen::Vector3d(x, 0.0, z);
  mTargetFrame->setTransform(tf);
}

void ControlledPendulumScene::step(double /*dt*/)
{
  if (!mShoulder)
    return;

  const double pos = mShoulder->getPosition(0);
  const double vel = mShoulder->getVelocity(0);

  const double error = mTargetAngle - pos;
  const double command = mKp * error - mKd * vel;
  mShoulder->setCommand(0, command);

  updateTargetFrame();
}

void ControlledPendulumScene::renderImGui()
{
  const double pos = mShoulder ? mShoulder->getPosition(0) : 0.0;
  const double vel = mShoulder ? mShoulder->getVelocity(0) : 0.0;

  float targetDegrees = static_cast<float>(mTargetAngle / kDegToRad);
  if (ImGui::SliderFloat("Target angle (deg)", &targetDegrees, -120.0f, 120.0f))
    mTargetAngle = static_cast<double>(targetDegrees) * kDegToRad;

  float kp = static_cast<float>(mKp);
  float kd = static_cast<float>(mKd);
  if (ImGui::SliderFloat("Kp", &kp, 5.0f, 120.0f))
    mKp = kp;
  if (ImGui::SliderFloat("Kd", &kd, 0.1f, 20.0f))
    mKd = kd;

  ImGui::Spacing();
  ImGui::Text("Current angle: %.1f deg", pos / kDegToRad);
  ImGui::Text("Angular velocity: %.2f rad/s", vel);
  ImGui::Text("Tip height: %.2f m", 1.0 - std::cos(pos) * mArmLength);

  if (ImGui::Button("Reset pose"))
    reset();
}

std::shared_ptr<Scene> createControlledPendulumScene()
{
  return std::make_shared<ControlledPendulumScene>();
}

} // namespace dart::demo
