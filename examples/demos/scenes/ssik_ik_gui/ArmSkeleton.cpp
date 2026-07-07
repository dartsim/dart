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

#include "ArmSkeleton.hpp"

#include <algorithm>

namespace dart_demos {
namespace ssik_ik_gui {

//==============================================================================
void ArmSkeleton::setWorld(const dart::simulation::WorldPtr& world)
{
  mWorld = world;
}

//==============================================================================
void ArmSkeleton::build(const std::vector<JointSpec>& chain)
{
  clearInstances();
  mChain = chain;
}

//==============================================================================
void ArmSkeleton::showSolutions(
    const std::vector<SsikSolution>& solutions, int selected)
{
  if (mChain.empty()) {
    clearInstances();
    return;
  }

  const std::size_t wanted
      = solutions.empty()
            ? 1
            : std::min<std::size_t>(solutions.size(), kMaxInstances);
  while (mInstances.size() < wanted)
    mInstances.push_back(buildInstance());
  while (mInstances.size() > wanted) {
    mWorld->removeSkeleton(mInstances.back());
    mInstances.pop_back();
  }

  const Eigen::Vector4d selectedColor(0.90, 0.20, 0.18, 1.0);
  const Eigen::Vector4d ghostColor(0.85, 0.32, 0.30, 0.22);
  const Eigen::Vector4d restColor(0.55, 0.58, 0.62, 0.5);

  if (solutions.empty()) {
    setConfig(mInstances[0], Eigen::VectorXd::Zero(numDofs()));
    setColor(mInstances[0], restColor);
    return;
  }

  for (std::size_t i = 0; i < wanted; ++i) {
    setConfig(mInstances[i], solutions[i].q);
    setColor(
        mInstances[i],
        static_cast<int>(i) == selected ? selectedColor : ghostColor);
  }
}

//==============================================================================
std::size_t ArmSkeleton::numDofs() const
{
  return mInstances.empty() ? mChain.size() : mInstances.front()->getNumDofs();
}

//==============================================================================
void ArmSkeleton::clearInstances()
{
  for (const auto& skeleton : mInstances)
    mWorld->removeSkeleton(skeleton);
  mInstances.clear();
}

//==============================================================================
void ArmSkeleton::setConfig(
    const dart::dynamics::SkeletonPtr& skeleton, const Eigen::VectorXd& q)
{
  const std::size_t dofs = skeleton->getNumDofs();
  Eigen::VectorXd positions = Eigen::VectorXd::Zero(dofs);
  const Eigen::Index n
      = std::min<Eigen::Index>(static_cast<Eigen::Index>(dofs), q.size());
  positions.head(n) = q.head(n);
  skeleton->setPositions(positions);
}

//==============================================================================
void ArmSkeleton::setColor(
    const dart::dynamics::SkeletonPtr& skeleton, const Eigen::Vector4d& color)
{
  for (std::size_t b = 0; b < skeleton->getNumBodyNodes(); ++b) {
    auto* body = skeleton->getBodyNode(b);
    for (std::size_t s = 0; s < body->getNumShapeNodes(); ++s) {
      auto* node = body->getShapeNode(s);
      if (auto* visual = node->getVisualAspect())
        visual->setRGBA(color);
    }
  }
}

//==============================================================================
dart::dynamics::SkeletonPtr ArmSkeleton::buildInstance()
{
  auto skeleton = dart::dynamics::Skeleton::create(
      "ssik_arm_" + std::to_string(mInstanceCounter++));

  dart::dynamics::WeldJoint::Properties baseProps;
  baseProps.mName = "base_joint";
  dart::dynamics::BodyNode* parent
      = skeleton
            ->createJointAndBodyNodePair<dart::dynamics::WeldJoint>(
                nullptr, baseProps)
            .second;
  parent->setName("base");
  addSphere(parent, Eigen::Vector3d::Zero());
  addCylinder(
      parent, Eigen::Vector3d::Zero(), mChain.front().tLeft.translation());

  for (std::size_t k = 0; k < mChain.size(); ++k) {
    const JointSpec& spec = mChain[k];
    Eigen::Vector3d axis = spec.axis;
    if (axis.norm() > 1e-9)
      axis.normalize();

    dart::dynamics::BodyNode* body = nullptr;
    if (spec.prismatic) {
      dart::dynamics::PrismaticJoint::Properties props;
      props.mName = "j" + std::to_string(k);
      props.mAxis = axis;
      props.mT_ParentBodyToJoint = spec.tLeft;
      props.mT_ChildBodyToJoint = spec.tRight.inverse();
      body = skeleton
                 ->createJointAndBodyNodePair<dart::dynamics::PrismaticJoint>(
                     parent, props)
                 .second;
    } else {
      dart::dynamics::RevoluteJoint::Properties props;
      props.mName = "j" + std::to_string(k);
      props.mAxis = axis;
      props.mT_ParentBodyToJoint = spec.tLeft;
      props.mT_ChildBodyToJoint = spec.tRight.inverse();
      body = skeleton
                 ->createJointAndBodyNodePair<dart::dynamics::RevoluteJoint>(
                     parent, props)
                 .second;
    }
    body->setName("b" + std::to_string(k));

    addSphere(body, Eigen::Vector3d::Zero());
    // The body's parent and child joints are fixed in its frame, so the link
    // cylinders connecting them are static geometry that moves with the body.
    addCylinder(
        body, spec.tRight.inverse().translation(), Eigen::Vector3d::Zero());
    if (k + 1 < mChain.size())
      addCylinder(
          body, Eigen::Vector3d::Zero(), mChain[k + 1].tLeft.translation());

    parent = body;
  }

  mWorld->addSkeleton(skeleton);
  return skeleton;
}

//==============================================================================
void ArmSkeleton::addCylinder(
    dart::dynamics::BodyNode* body,
    const Eigen::Vector3d& a,
    const Eigen::Vector3d& b)
{
  const Eigen::Vector3d delta = b - a;
  const double length = delta.norm();
  if (length < 1e-5)
    return;
  auto shape
      = std::make_shared<dart::dynamics::CylinderShape>(kLinkRadius, length);
  auto* node = body->createShapeNodeWith<dart::dynamics::VisualAspect>(shape);
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = 0.5 * (a + b);
  tf.linear() = Eigen::Quaterniond::FromTwoVectors(
                    Eigen::Vector3d::UnitZ(), delta / length)
                    .toRotationMatrix();
  node->setRelativeTransform(tf);
}

//==============================================================================
void ArmSkeleton::addSphere(
    dart::dynamics::BodyNode* body, const Eigen::Vector3d& position)
{
  auto shape = std::make_shared<dart::dynamics::SphereShape>(kJointRadius);
  auto* node = body->createShapeNodeWith<dart::dynamics::VisualAspect>(shape);
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = position;
  node->setRelativeTransform(tf);
}

} // namespace ssik_ik_gui
} // namespace dart_demos
