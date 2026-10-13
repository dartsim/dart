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

#include "SoftWormModel.hpp"

#include <algorithm>
#include <array>
#include <memory>
#include <string>

#include <cmath>

namespace dart_demos {
namespace soft_worm_model {

namespace {

using dart::dynamics::BodyNode;
using dart::dynamics::BoxShape;
using dart::dynamics::CollisionAspect;
using dart::dynamics::DynamicsAspect;
using dart::dynamics::FreeJoint;
using dart::dynamics::Joint;
using dart::dynamics::RevoluteJoint;
using dart::dynamics::Skeleton;
using dart::dynamics::SkeletonPtr;
using dart::dynamics::SoftBodyNode;
using dart::dynamics::SoftBodyNodeHelper;
using dart::dynamics::SoftMeshShape;
using dart::dynamics::VisualAspect;
using dart::dynamics::WeldJoint;
using dart::simulation::World;

constexpr double kTimeStep = 0.001;
constexpr double kLinkLength = 0.52;
constexpr double kLinkWidth = 0.34;
constexpr double kLinkHeight = 0.28;
constexpr double kRigidCoreMass = 0.16;
constexpr double kFleshMass = 0.54;
constexpr double kVertexStiffness = 850.0;
constexpr double kEdgeStiffness = 120.0;
constexpr double kFleshDamping = 8.0;
constexpr double kGaitPhaseOffset = 1.05;
constexpr double kTrackingGain = 7.0;
constexpr double kMaxJointSpeed = 4.0;
constexpr double kGroundFriction = 1.25;

//==============================================================================
void setBoxInertia(BodyNode* body, const Eigen::Vector3d& size, double mass)
{
  dart::dynamics::Inertia inertia;
  inertia.setMass(mass);
  inertia.setMoment(BoxShape(size).computeInertia(mass));
  body->setInertia(inertia);
}

//==============================================================================
SkeletonPtr createGround()
{
  auto ground = Skeleton::create("ground");
  auto* body = ground->createJointAndBodyNodePair<WeldJoint>(nullptr).second;
  auto shape = std::make_shared<BoxShape>(Eigen::Vector3d(14.0, 5.0, 0.08));
  auto* shapeNode = body->createShapeNodeWith<
      VisualAspect,
      CollisionAspect,
      DynamicsAspect>(shape);
  shapeNode->getVisualAspect()->setColor(Eigen::Vector3d(0.33, 0.36, 0.40));
  shapeNode->getDynamicsAspect()->setFrictionCoeff(kGroundFriction);
  shapeNode->getDynamicsAspect()->setRestitutionCoeff(0.0);

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation().z() = -0.04;
  body->getParentJoint()->setTransformFromParentBodyNode(transform);
  ground->setMobile(false);
  return ground;
}

//==============================================================================
SoftBodyNode::Properties makeSoftLinkProperties(std::size_t index)
{
  const std::string name = "soft_link_" + std::to_string(index);
  BodyNode::Properties bodyProperties{BodyNode::AspectProperties(name)};
  bodyProperties.mInertia.setMass(kRigidCoreMass);
  bodyProperties.mInertia.setMoment(
      BoxShape(Eigen::Vector3d(kLinkLength, kLinkWidth, kLinkHeight))
          .computeInertia(kRigidCoreMass));

  auto fleshProperties = SoftBodyNodeHelper::makeBoxProperties(
      Eigen::Vector3d(kLinkLength, kLinkWidth, kLinkHeight),
      Eigen::Isometry3d::Identity(),
      Eigen::Vector3i(3, 3, 3),
      kFleshMass,
      kVertexStiffness,
      kEdgeStiffness,
      kFleshDamping);
  return SoftBodyNode::Properties(bodyProperties, fleshProperties);
}

//==============================================================================
void styleSoftLink(SoftBodyNode* body, std::size_t index)
{
  const std::array<Eigen::Vector4d, kNumLinks> colors = {
      Eigen::Vector4d(0.18, 0.66, 0.83, 0.88),
      Eigen::Vector4d(0.24, 0.77, 0.63, 0.88),
      Eigen::Vector4d(0.78, 0.82, 0.31, 0.88),
      Eigen::Vector4d(0.95, 0.62, 0.25, 0.88),
      Eigen::Vector4d(0.88, 0.35, 0.42, 0.88),
  };

  for (std::size_t i = 0; i < body->getNumShapeNodes(); ++i) {
    auto* shapeNode = body->getShapeNode(i);
    if (auto* dynamics = shapeNode->getDynamicsAspect(false)) {
      dynamics->setFrictionCoeff(kGroundFriction);
      dynamics->setRestitutionCoeff(0.0);
    }
    auto* visual = shapeNode->getVisualAspect(false);
    if (!visual)
      continue;
    const auto shape = shapeNode->getShape();
    if (shape && shape->getType() == SoftMeshShape::getStaticType())
      visual->setRGBA(colors[index % colors.size()]);
  }
}

//==============================================================================
SkeletonPtr createWorm()
{
  auto worm = Skeleton::create("soft_worm");

  dart::dynamics::GenericJoint<dart::math::SE3Space>::Properties rootJoint;
  rootJoint.mName = "root_free_joint";
  auto rootPair = worm->createJointAndBodyNodePair<FreeJoint, SoftBodyNode>(
      nullptr, rootJoint, makeSoftLinkProperties(0));
  rootPair.first->setActuatorType(Joint::PASSIVE);
  setBoxInertia(
      rootPair.second,
      Eigen::Vector3d(kLinkLength, kLinkWidth, kLinkHeight),
      kRigidCoreMass);
  styleSoftLink(rootPair.second, 0);

  Eigen::Isometry3d rootTransform = Eigen::Isometry3d::Identity();
  rootTransform.translation() = Eigen::Vector3d(0.0, 0.0, 0.19);
  rootPair.first->setPositions(FreeJoint::convertToPositions(rootTransform));

  BodyNode* parent = rootPair.second;
  for (std::size_t i = 1; i < kNumLinks; ++i) {
    RevoluteJoint::Properties joint;
    joint.mName = "spine_joint_" + std::to_string(i);
    joint.mAxis = Eigen::Vector3d::UnitY();
    joint.mT_ParentBodyToJoint.translation().x() = 0.5 * kLinkLength;
    joint.mT_ChildBodyToJoint.translation().x() = -0.5 * kLinkLength;
    joint.mPositionLowerLimits[0] = -0.92;
    joint.mPositionUpperLimits[0] = 0.92;

    auto pair = worm->createJointAndBodyNodePair<RevoluteJoint, SoftBodyNode>(
        parent, joint, makeSoftLinkProperties(i));
    pair.first->setActuatorType(Joint::SERVO);
    pair.first->setLimitEnforcement(true);
    pair.first->setVelocityLowerLimit(0, -kMaxJointSpeed);
    pair.first->setVelocityUpperLimit(0, kMaxJointSpeed);
    setBoxInertia(
        pair.second,
        Eigen::Vector3d(kLinkLength, kLinkWidth, kLinkHeight),
        kRigidCoreMass);
    styleSoftLink(pair.second, i);
    parent = pair.second;
  }

  return worm;
}

} // namespace

//==============================================================================
dart::simulation::WorldPtr createWorld(SkeletonPtr& worm)
{
  auto world = World::create("soft_worm_world");
  world->setTimeStep(kTimeStep);
  world->setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  world->addSkeleton(createGround());
  worm = createWorm();
  world->addSkeleton(worm);
  return world;
}

//==============================================================================
void applyGait(const SkeletonPtr& worm, double time, bool enabled)
{
  const double omega = 2.0 * dart::math::constantsd::pi() * kGaitFrequencyHz;
  for (std::size_t i = 1; i < kNumLinks; ++i) {
    auto* joint = worm->getJoint("spine_joint_" + std::to_string(i));
    if (!joint)
      continue;

    double command = 0.0;
    if (enabled) {
      const double phase = omega * time - kGaitPhaseOffset * (i - 1);
      const double target = kGaitAmplitude * std::sin(phase);
      const double targetVelocity = kGaitAmplitude * omega * std::cos(phase);
      command
          = targetVelocity + kTrackingGain * (target - joint->getPosition(0));
      command = std::clamp(command, -kMaxJointSpeed, kMaxJointSpeed);
    }
    joint->setCommand(0, command);
  }
}

//==============================================================================
double getRootX(const SkeletonPtr& worm)
{
  return worm->getRootBodyNode()->getTransform().translation().x();
}

//==============================================================================
double positionChecksum(const SkeletonPtr& worm)
{
  long double checksum = 0.0;
  std::size_t component = 1;
  const auto positions = worm->getPositions();
  for (Eigen::Index i = 0; i < positions.size(); ++i)
    checksum += static_cast<long double>(component++) * positions[i];

  for (std::size_t i = 0; i < worm->getNumSoftBodyNodes(); ++i) {
    const auto* softBody = worm->getSoftBodyNode(i);
    for (std::size_t j = 0; j < softBody->getNumPointMasses(); ++j) {
      const Eigen::Vector3d& position
          = softBody->getPointMass(j)->getWorldPosition();
      for (int axis = 0; axis < 3; ++axis)
        checksum += static_cast<long double>(component++) * position[axis];
    }
  }
  return static_cast<double>(checksum);
}

//==============================================================================
bool isFinite(const SkeletonPtr& worm)
{
  if (!worm->getPositions().allFinite() || !worm->getVelocities().allFinite())
    return false;

  for (std::size_t i = 0; i < worm->getNumSoftBodyNodes(); ++i) {
    const auto* softBody = worm->getSoftBodyNode(i);
    if (!softBody->getTransform().matrix().allFinite())
      return false;
    for (std::size_t j = 0; j < softBody->getNumPointMasses(); ++j) {
      const auto* point = softBody->getPointMass(j);
      if (!point->getPositions().allFinite()
          || !point->getVelocities().allFinite()
          || !point->getWorldPosition().allFinite()) {
        return false;
      }
    }
  }
  return true;
}

} // namespace soft_worm_model
} // namespace dart_demos
