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

#include "AdaptiveSoftContactModel.hpp"

#include <memory>

#include <cmath>

namespace dart_demos {
namespace adaptive_soft_contact_model {

namespace {

using dart::dynamics::BodyNode;
using dart::dynamics::BoxShape;
using dart::dynamics::CollisionAspect;
using dart::dynamics::DynamicsAspect;
using dart::dynamics::FreeJoint;
using dart::dynamics::Skeleton;
using dart::dynamics::SkeletonPtr;
using dart::dynamics::SoftBodyNode;
using dart::dynamics::SoftBodyNodeHelper;
using dart::dynamics::VisualAspect;
using dart::dynamics::WeldJoint;
using dart::simulation::World;

constexpr double kPi = 3.14159265358979323846;

//==============================================================================
SkeletonPtr createGround()
{
  auto ground = Skeleton::create("ground");
  auto* body = ground->createJointAndBodyNodePair<WeldJoint>().second;
  auto* shape = body->createShapeNodeWith<
      VisualAspect,
      CollisionAspect,
      DynamicsAspect>(
      std::make_shared<BoxShape>(Eigen::Vector3d(6.0, 4.0, 0.1)));
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation().z() = -0.05;
  shape->setRelativeTransform(transform);
  shape->getVisualAspect()->setColor(Eigen::Vector3d(0.62, 0.65, 0.69));
  shape->getDynamicsAspect()->setFrictionCoeff(1.0);
  return ground;
}

//==============================================================================
SkeletonPtr createSoftEllipsoid(
    Model& model,
    bool adaptiveEnabled,
    std::size_t ringCount,
    std::size_t lingerSteps)
{
  auto skeleton = Skeleton::create("adaptive_ellipsoid");
  FreeJoint::Properties jointProperties;
  jointProperties.mName = "adaptive_ellipsoid_joint";

  BodyNode::Properties bodyProperties(
      BodyNode::AspectProperties("adaptive_ellipsoid_body"));
  const auto ellipsoidShape = std::make_shared<dart::dynamics::EllipsoidShape>(
      Eigen::Vector3d(0.72, 0.54, 0.60));
  bodyProperties.mInertia = dart::dynamics::Inertia(
      0.8, Eigen::Vector3d::Zero(), ellipsoidShape->computeInertia(0.8));

  const auto uniqueProperties = SoftBodyNodeHelper::makeEllipsoidProperties(
      Eigen::Vector3d(0.72, 0.54, 0.60), 12, 8, 1.6, 4000.0, 400.0, 500.0);
  const SoftBodyNode::Properties properties(bodyProperties, uniqueProperties);
  const auto pair
      = skeleton->createJointAndBodyNodePair<FreeJoint, SoftBodyNode>(
          nullptr, jointProperties, properties);
  model.softBody = pair.second;
  model.softBody->getShapeNode(0)->getDynamicsAspect()->setFrictionCoeff(1.1);
  model.softBody->setAdaptiveContactActivationRingCount(ringCount);
  model.softBody->setAdaptiveContactActivationLingerSteps(lingerSteps);
  model.softBody->setAdaptiveContactActivationVelocityTolerance(0.035);
  model.softBody->setAdaptiveContactActivationPositionTolerance(0.018);
  model.softBody->setAdaptiveContactActivationEnabled(adaptiveEnabled);

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(0.0, 0.0, 1.10);
  pair.first->setPositions(FreeJoint::convertToPositions(transform));
  model.softBody->getShapeNode(0)->getVisualAspect()->setColor(
      Eigen::Vector4d(0.20, 0.48, 0.95, 0.58));
  return skeleton;
}

//==============================================================================
SkeletonPtr createPusher(Model& model)
{
  auto skeleton = Skeleton::create("periodic_pusher");
  const auto pair = skeleton->createJointAndBodyNodePair<FreeJoint>();
  model.pusherJoint = pair.first;
  auto* body = pair.second;
  const auto shape
      = std::make_shared<BoxShape>(Eigen::Vector3d(0.20, 0.70, 0.34));
  auto* shapeNode = body->createShapeNodeWith<
      VisualAspect,
      CollisionAspect,
      DynamicsAspect>(shape);
  body->setInertia(dart::dynamics::Inertia(
      20.0, Eigen::Vector3d::Zero(), shape->computeInertia(20.0)));
  shapeNode->getVisualAspect()->setColor(Eigen::Vector3d(0.96, 0.48, 0.16));
  shapeNode->getDynamicsAspect()->setFrictionCoeff(0.8);
  skeleton->setMobile(false);
  return skeleton;
}

} // namespace

//==============================================================================
Model createModel(
    bool adaptiveEnabled, std::size_t ringCount, std::size_t lingerSteps)
{
  Model model;
  model.world = World::create("adaptive_soft_contact");
  model.world->setTimeStep(kTimeStep);
  model.world->setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  model.softSkeleton
      = createSoftEllipsoid(model, adaptiveEnabled, ringCount, lingerSteps);
  model.pusherSkeleton = createPusher(model);
  model.world->addSkeleton(createGround());
  model.world->addSkeleton(model.softSkeleton);
  model.world->addSkeleton(model.pusherSkeleton);
  setPusherPose(model, 0.0);
  return model;
}

//==============================================================================
void configure(
    Model& model,
    bool adaptiveEnabled,
    std::size_t ringCount,
    std::size_t lingerSteps)
{
  if (model.softBody->getAdaptiveContactActivationRingCount() != ringCount)
    model.softBody->setAdaptiveContactActivationRingCount(ringCount);
  if (model.softBody->getAdaptiveContactActivationLingerSteps() != lingerSteps)
    model.softBody->setAdaptiveContactActivationLingerSteps(lingerSteps);
  if (model.softBody->isAdaptiveContactActivationEnabled() != adaptiveEnabled)
    model.softBody->setAdaptiveContactActivationEnabled(adaptiveEnabled);
}

//==============================================================================
void setPusherPose(Model& model, double time)
{
  const double phase = 2.0 * kPi * time / 1.4;
  const double x = -1.10 + 0.325 * (1.0 - std::cos(phase));
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(x, 0.0, 0.37);
  model.pusherJoint->setPositions(FreeJoint::convertToPositions(transform));
}

//==============================================================================
void prepareStep(Model& model)
{
  setPusherPose(model, model.world->getTime());
}

//==============================================================================
bool step(Model& model)
{
  prepareStep(model);
  model.world->step();
  return isFinite(model);
}

//==============================================================================
Checksum computeChecksum(const Model& model)
{
  Checksum result;
  const Eigen::VectorXd positions = model.softSkeleton->getPositions();
  const Eigen::VectorXd velocities = model.softSkeleton->getVelocities();
  result.skeletonPositionL1 = positions.cwiseAbs().sum();
  result.skeletonPositionSquared = positions.squaredNorm();
  result.finite = positions.allFinite() && velocities.allFinite();
  result.active = model.softBody->getNumActivePointMasses();
  result.total = model.softBody->getNumPointMasses();
  result.contacts = model.world->getLastCollisionResult().getNumContacts();

  for (std::size_t i = 0; i < result.total; ++i) {
    const auto* point = model.softBody->getPointMass(i);
    const Eigen::Vector3d worldPosition = point->getWorldPosition();
    result.pointWorldPositionL1 += worldPosition.cwiseAbs().sum();
    result.pointWorldPositionSquared += worldPosition.squaredNorm();
    result.finite = result.finite && worldPosition.allFinite()
                    && point->getPositions().allFinite()
                    && point->getVelocities().allFinite();
  }
  return result;
}

//==============================================================================
double positionChecksum(const Model& model)
{
  long double checksum = 0.0;
  std::size_t component = 1;
  const auto positions = model.softSkeleton->getPositions();
  for (Eigen::Index i = 0; i < positions.size(); ++i)
    checksum += static_cast<long double>(component++) * positions[i];

  for (std::size_t i = 0; i < model.softBody->getNumPointMasses(); ++i) {
    const Eigen::Vector3d& position
        = model.softBody->getPointMass(i)->getWorldPosition();
    for (int axis = 0; axis < 3; ++axis)
      checksum += static_cast<long double>(component++) * position[axis];
  }
  return static_cast<double>(checksum);
}

//==============================================================================
bool isFinite(const Model& model)
{
  return computeChecksum(model).finite;
}

//==============================================================================
double surfacePoseDelta(const Model& lhs, const Model& rhs)
{
  const Eigen::Isometry3d& lhsTransform = lhs.softBody->getWorldTransform();
  const Eigen::Isometry3d& rhsTransform = rhs.softBody->getWorldTransform();
  const double translationDelta
      = (lhsTransform.translation() - rhsTransform.translation()).norm();
  const Eigen::Matrix3d relativeRotation
      = lhsTransform.linear().transpose() * rhsTransform.linear();
  const double rotationDelta = Eigen::AngleAxisd(relativeRotation).angle();
  return translationDelta + kSoftBodyRadius * rotationDelta;
}

} // namespace adaptive_soft_contact_model
} // namespace dart_demos
