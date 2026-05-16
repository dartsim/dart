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

#include <dart/simulation/world.hpp>

#include <dart/dynamics/body_node.hpp>
#include <dart/dynamics/box_shape.hpp>
#include <dart/dynamics/revolute_joint.hpp>
#include <dart/dynamics/shape_node.hpp>
#include <dart/dynamics/skeleton.hpp>

#include <Eigen/Geometry>

#include <memory>

namespace {

constexpr const char* kSkeletonName = "visual_hardcoded_design";
constexpr double kPi = 3.14159265358979323846;

dart::dynamics::SkeletonPtr createHardcodedDesignSkeleton()
{
  auto skeleton = dart::dynamics::Skeleton::create(kSkeletonName);
  constexpr double mass = 1.0;

  dart::dynamics::BodyNode::Properties body;
  body.mName = "LHY";
  body.mInertia.setMass(mass);
  auto shape = std::make_shared<dart::dynamics::BoxShape>(
      Eigen::Vector3d(0.3, 0.3, 1.0));

  dart::dynamics::RevoluteJoint::Properties joint;
  joint.mName = "LHY";
  joint.mAxis = Eigen::Vector3d::UnitZ();
  joint.mPositionLowerLimits[0] = -kPi;
  joint.mPositionUpperLimits[0] = kPi;

  auto [rootJoint, rootBody]
      = skeleton->createJointAndBodyNodePair<dart::dynamics::RevoluteJoint>(
          nullptr, joint, body);
  (void)rootJoint;
  auto* rootShapeNode = rootBody->createShapeNodeWith<
      dart::dynamics::VisualAspect,
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(shape);
  rootShapeNode->getVisualAspect()->setRGBA(
      Eigen::Vector4d(0.18, 0.38, 0.92, 0.9));
  rootBody->setMass(mass);

  body = dart::dynamics::BodyNode::Properties();
  body.mName = "LHR";
  body.mInertia.setMass(mass);
  shape = std::make_shared<dart::dynamics::BoxShape>(
      Eigen::Vector3d(0.3, 0.3, 1.0));
  joint = dart::dynamics::RevoluteJoint::Properties();
  joint.mName = "LHR";
  joint.mAxis = Eigen::Vector3d::UnitX();
  joint.mPositionLowerLimits[0] = -kPi;
  joint.mPositionUpperLimits[0] = kPi;
  joint.mT_ParentBodyToJoint = Eigen::Translation3d(0.0, 0.0, 0.5);

  auto [hipRollJoint, hipRollBody]
      = skeleton->createJointAndBodyNodePair<dart::dynamics::RevoluteJoint>(
          rootBody, joint, body);
  (void)hipRollJoint;
  auto* hipRollShapeNode = hipRollBody->createShapeNodeWith<
      dart::dynamics::VisualAspect,
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(shape);
  hipRollShapeNode->setRelativeTranslation(Eigen::Vector3d(0.0, 0.0, 0.5));
  hipRollShapeNode->getVisualAspect()->setRGBA(
      Eigen::Vector4d(0.95, 0.65, 0.12, 0.9));
  hipRollBody->setLocalCOM(hipRollShapeNode->getRelativeTranslation());
  hipRollBody->setMass(mass);

  body = dart::dynamics::BodyNode::Properties();
  body.mName = "LHP";
  body.mInertia.setMass(mass);
  shape = std::make_shared<dart::dynamics::BoxShape>(
      Eigen::Vector3d(0.3, 0.3, 1.0));
  joint = dart::dynamics::RevoluteJoint::Properties();
  joint.mName = "LHP";
  joint.mAxis = Eigen::Vector3d::UnitY();
  joint.mPositionLowerLimits[0] = -kPi;
  joint.mPositionUpperLimits[0] = kPi;
  joint.mT_ParentBodyToJoint = Eigen::Translation3d(0.0, 0.0, 1.0);

  auto [hipPitchJoint, hipPitchBody]
      = skeleton->createJointAndBodyNodePair<dart::dynamics::RevoluteJoint>(
          hipRollBody, joint, body);
  (void)hipPitchJoint;
  auto* hipPitchShapeNode = hipPitchBody->createShapeNodeWith<
      dart::dynamics::VisualAspect,
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(shape);
  hipPitchShapeNode->setRelativeTranslation(Eigen::Vector3d(0.0, 0.0, 0.5));
  hipPitchShapeNode->getVisualAspect()->setRGBA(
      Eigen::Vector4d(0.84, 0.18, 0.18, 0.9));
  hipPitchBody->setLocalCOM(hipPitchShapeNode->getRelativeTranslation());
  hipPitchBody->setMass(mass);

  return skeleton;
}

dart::simulation::WorldPtr createHardcodedDesignWorld()
{
  auto world = dart::simulation::World::create("dartsim_hardcoded_design");
  world->setGravity(Eigen::Vector3d::Zero());
  world->addSkeleton(createHardcodedDesignSkeleton());
  return world;
}

} // namespace

int main(int argc, char* argv[])
{
  dart::gui::ApplicationOptions options;
  options.world = createHardcodedDesignWorld();
  return dart::gui::runApplication(argc, argv, options);
}
