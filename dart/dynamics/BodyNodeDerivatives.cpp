/*
 * Copyright (c) 2016, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016, Humanoid Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016, Personal Robotics Lab, Carnegie Mellon University
 * All rights reserved.
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

#include "dart/dynamics/BodyNodeDerivatives.hpp"

#include "dart/dynamics/BodyNode.hpp"
#include "dart/dynamics/DegreeOfFreedom.hpp"

namespace dart {
namespace dynamics {

//==============================================================================
std::unique_ptr<common::Aspect> BodyNodeDerivatives::cloneAspect() const
{
  return common::make_unique<BodyNodeDerivatives>();
}

//==============================================================================
const BodyNodeDerivatives::GradientMatrix&
BodyNodeDerivatives::getSpatialVelocityDerivativeWrtPos() const
{
  if (mNeedSpatialVelocityDerivativeWrtPosUpdate)
  {
    const auto* bodyNode = mComposite;
    const auto* joint = bodyNode->getParentJoint();
    const auto* parentBodyNode = bodyNode->getParentBodyNode();

    if (parentBodyNode)
    {
      const auto* parentDeriv = parentBodyNode->get<BodyNodeDerivatives>();

      mV_q = math::AdInvTJac(
          joint->getRelativeTransform(),
          parentDeriv->getSpatialVelocityDerivativeWrtPos());
    }
    else
    {
      mV_q.setZero();
    }

    const auto numDofs = joint->getNumDofs();
    if (numDofs != 0)
    {
      const auto index = joint->getDof(0)->getIndexInSkeleton();

      assert(joint->getRelativeJacobian().rows() == 6);
      assert(joint->getRelativeJacobian().cols() == static_cast<int>(numDofs));
      mV_q.block(0, index, 6, numDofs).noalias() += math::adJac(
          bodyNode->getSpatialVelocity(), joint->getRelativeJacobian());

      mV_q.block(0, index, 6, numDofs).noalias()
          += joint->getRelativeJacobianTimeDeriv();
    }

    mNeedSpatialVelocityDerivativeWrtPosUpdate = false;
  }

  return mV_q;
}

//==============================================================================
Eigen::Vector6d BodyNodeDerivatives::getSpatialVelocityDerivativeWrtPos(
    std::size_t indexInSkeleton) const
{
  return getSpatialVelocityDerivativeWrtPos().col(indexInSkeleton);
}

//==============================================================================
Eigen::Vector6d BodyNodeDerivatives::getSpatialVelocityDerivativeWrtPos(
    const DegreeOfFreedom* withRespectTo) const
{
  const auto index = withRespectTo->getIndexInSkeleton();

  return getSpatialVelocityDerivativeWrtPos(index);
}

//==============================================================================
BodyNodeDerivatives::GradientMatrix
BodyNodeDerivatives::getSpatialVelocityDerivativeWrtPos(
    const Joint* withRespectTo) const
{
  const auto index = withRespectTo->getIndexInSkeleton(0);
  const auto numDofs = withRespectTo->getNumDofs();

  return getSpatialVelocityDerivativeWrtPos().block(0, index, 6, numDofs);
}

//==============================================================================
const BodyNodeDerivatives::GradientMatrix&
BodyNodeDerivatives::getSpatialVelocityDerivativeWrtVel() const
{
  if (mNeedSpatialVelocityDerivativeWrtVelUpdate)
  {
    const auto* bodyNode = mComposite;
    const auto* joint = bodyNode->getParentJoint();
    const auto* parentBodyNode = bodyNode->getParentBodyNode();

    if (parentBodyNode)
    {
      const auto* parentDeriv = parentBodyNode->get<BodyNodeDerivatives>();

      mV_dq = math::AdInvTJac(
          joint->getRelativeTransform(),
          parentDeriv->getSpatialVelocityDerivativeWrtVel());
    }
    else
    {
      mV_dq.setZero();
    }

    const auto numDofs = joint->getNumDofs();
    if (numDofs != 0)
    {
      const auto index = joint->getDof(0)->getIndexInSkeleton();

      assert(joint->getRelativeJacobian().rows() == 6);
      assert(joint->getRelativeJacobian().cols() == static_cast<int>(numDofs));
      mV_dq.block(0, index, 6, numDofs).noalias()
          += joint->getRelativeJacobian();
    }

    mNeedSpatialVelocityDerivativeWrtVelUpdate = false;
  }

  return mV_dq;
}

//==============================================================================
Eigen::Vector6d BodyNodeDerivatives::getSpatialVelocityDerivativeWrtVel(
    std::size_t indexInSkeleton) const
{
  return getSpatialVelocityDerivativeWrtVel().col(indexInSkeleton);
}

//==============================================================================
Eigen::Vector6d BodyNodeDerivatives::getSpatialVelocityDerivativeWrtVel(
    const DegreeOfFreedom* withRespectTo) const
{
  const auto index = withRespectTo->getIndexInSkeleton();

  return getSpatialVelocityDerivativeWrtVel(index);
}

//==============================================================================
BodyNodeDerivatives::GradientMatrix
BodyNodeDerivatives::getSpatialVelocityDerivativeWrtVel(
    const Joint* withRespectTo) const
{
  const auto index = withRespectTo->getIndexInSkeleton(0);
  const auto numDofs = withRespectTo->getNumDofs();

  return getSpatialVelocityDerivativeWrtVel().block(0, index, 6, numDofs);
}

//==============================================================================
Eigen::VectorXd BodyNodeDerivatives::computeKineticEnergyGradientWrtPos() const
{
  const BodyNode* thisBodyNode = mComposite;
  const Eigen::Matrix6d& G = thisBodyNode->getInertia().getSpatialTensor();
  const Eigen::Vector6d& V = thisBodyNode->getSpatialVelocity();

  return V.transpose() * G * getSpatialVelocityDerivativeWrtPos();
}

//==============================================================================
Eigen::VectorXd BodyNodeDerivatives::computeKineticEnergyGradientWrtVel() const
{
  const BodyNode* thisBodyNode = mComposite;
  const Eigen::Matrix6d& G = thisBodyNode->getInertia().getSpatialTensor();
  const Eigen::Vector6d& V = thisBodyNode->getSpatialVelocity();

  return V.transpose() * G * getSpatialVelocityDerivativeWrtVel();
}

//==============================================================================
Eigen::VectorXd BodyNodeDerivatives::computeLagrangianGradientWrtPos() const
{
  return computeKineticEnergyGradientWrtPos();
}

//==============================================================================
Eigen::VectorXd BodyNodeDerivatives::computeLagrangianGradientWrtVel() const
{
  return computeKineticEnergyGradientWrtVel();
}

//==============================================================================
void BodyNodeDerivatives::setComposite(common::Composite* newComposite)
{
  Base::setComposite(newComposite);

  auto* bodyNode = mComposite;
  auto* joint = bodyNode->getParentJoint();
  joint->onPositionUpdatedAdded.connect([=](const Joint* /*joint*/) {
    this->dirtySpatialVelocityDerivativeWrtPos();
    this->dirtySpatialVelocityDerivativeWrtVel();
  });
  joint->onVelocityUpdatedAdded.connect([=](const Joint* /*joint*/) {
    this->dirtySpatialVelocityDerivativeWrtPos();
  });

  const auto skeleton = bodyNode->getSkeleton();
  const auto numDofs = skeleton->getNumDofs();

  assert(skeleton);

  mV_q.resize(6, numDofs);
  mV_dq.resize(6, numDofs);
  mdV_q.resize(6, numDofs);
  mdV_dq.resize(6, numDofs);

  // TODO(JS): These should be updated when the structure of skeleton is
  // modified. We might want to consider adding signal to skeleton for this.
}

//==============================================================================
void BodyNodeDerivatives::dirtySpatialVelocityDerivativeWrtPos()
{
  if (mNeedSpatialVelocityDerivativeWrtPosUpdate)
    return;

  mNeedSpatialVelocityDerivativeWrtPosUpdate = true;

  auto* bodyNode = mComposite;
  for (auto i = 0u; i < bodyNode->getNumChildBodyNodes(); ++i)
  {
    auto* childBodyNode = bodyNode->getChildBodyNode(i);
    auto* childDeriv = childBodyNode->get<BodyNodeDerivatives>();
    childDeriv->dirtySpatialVelocityDerivativeWrtPos();
  }
}

//==============================================================================
void BodyNodeDerivatives::dirtySpatialVelocityDerivativeWrtVel()
{
  if (mNeedSpatialVelocityDerivativeWrtVelUpdate)
    return;

  mNeedSpatialVelocityDerivativeWrtVelUpdate = true;

  auto* bodyNode = mComposite;
  for (auto i = 0u; i < bodyNode->getNumChildBodyNodes(); ++i)
  {
    auto* childBodyNode = bodyNode->getChildBodyNode(i);
    auto* childDeriv = childBodyNode->get<BodyNodeDerivatives>();
    childDeriv->dirtySpatialVelocityDerivativeWrtVel();
  }
}

} // namespace dynamics
} // namespace dart
