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

#include "dart/dynamics/DegreeOfFreedom.hpp"
#include "dart/dynamics/BodyNode.hpp"

namespace dart {
namespace dynamics {

//==============================================================================
std::unique_ptr<common::Aspect> BodyNodeDerivatives::cloneAspect() const
{
  return common::make_unique<BodyNodeDerivatives>();
}

//==============================================================================
const BodyNodeDerivatives::GradientMatrix&
BodyNodeDerivatives::getSpatialVelocityDerivativeWrtPositions() const
{
  if (mNeedSpatialVelocityDerivativeWrtPositionsUpdate)
  {
    const auto* thisBodyNode = mComposite;
    const auto* thisJoint = thisBodyNode->getParentJoint();
    const auto* thisParentBodyNode = thisBodyNode->getParentBodyNode();

    if (thisParentBodyNode)
    {
      const auto* parentDerivatives
          = thisParentBodyNode->get<BodyNodeDerivatives>();

      mV_q = math::AdInvTJac(
            thisJoint->getRelativeTransform(),
            parentDerivatives->getSpatialVelocityDerivativeWrtPositions());
    }
    else
    {
      mV_q.setZero();
    }

    const auto numDofs = thisJoint->getNumDofs();
    if (numDofs != 0)
    {
      const auto index = thisJoint->getDof(0)->getIndexInSkeleton();

      assert(thisJoint->getRelativeJacobian().rows() == 6);
      assert(thisJoint->getRelativeJacobian().cols()
             == static_cast<int>(numDofs));
      mV_q.block(0, index, 6, numDofs).noalias()
          += math::adJac(thisBodyNode->getSpatialVelocity(),
                         thisJoint->getRelativeJacobian());

      mV_q.block(0, index, 6, numDofs).noalias()
          += thisJoint->getRelativeJacobianTimeDeriv();
    }

    mNeedSpatialVelocityDerivativeWrtPositionsUpdate = false;
  }

  return mV_q;
}

//==============================================================================
Eigen::Vector6d
BodyNodeDerivatives::getSpatialVelocityDerivativeWrtPositions(
    std::size_t indexInSkeleton) const
{
  return getSpatialVelocityDerivativeWrtPositions().col(indexInSkeleton);
}

//==============================================================================
Eigen::Vector6d
BodyNodeDerivatives::getSpatialVelocityDerivativeWrtPositions(
    const DegreeOfFreedom* withRespectTo) const
{
  const auto index = withRespectTo->getIndexInSkeleton();

  return getSpatialVelocityDerivativeWrtPositions(index);
}

//==============================================================================
BodyNodeDerivatives::GradientMatrix
BodyNodeDerivatives::getSpatialVelocityDerivativeWrtPositions(
    const Joint* withRespectTo) const
{
  const auto index = withRespectTo->getIndexInSkeleton(0);
  const auto numDofs = withRespectTo->getNumDofs();

  return getSpatialVelocityDerivativeWrtPositions().block(0, index, 6, numDofs);
}

//==============================================================================
const BodyNodeDerivatives::GradientMatrix&
BodyNodeDerivatives::getSpatialVelocityDerivativeWrtVelocities() const
{
  if (mNeedSpatialVelocityDerivativeWrtVelocitiesUpdate)
  {
    const auto* thisBodyNode = mComposite;
    const auto* thisJoint = thisBodyNode->getParentJoint();
    const auto* thisParentBodyNode = thisBodyNode->getParentBodyNode();

    if (thisParentBodyNode)
    {
      const auto* parentDerivatives
          = thisParentBodyNode->get<BodyNodeDerivatives>();

      mV_dq = math::AdInvTJac(
            thisJoint->getRelativeTransform(),
            parentDerivatives->getSpatialVelocityDerivativeWrtVelocities());
    }
    else
    {
      mV_dq.setZero();
    }

    const auto numDofs = thisJoint->getNumDofs();
    if (numDofs != 0)
    {
      const auto index = thisJoint->getDof(0)->getIndexInSkeleton();

      assert(thisJoint->getRelativeJacobian().rows() == 6);
      assert(thisJoint->getRelativeJacobian().cols()
             == static_cast<int>(numDofs));
      mV_dq.block(0, index, 6, numDofs).noalias()
          += thisJoint->getRelativeJacobian();
    }

    mNeedSpatialVelocityDerivativeWrtVelocitiesUpdate = false;
  }

  return mV_dq;
}

//==============================================================================
Eigen::Vector6d
BodyNodeDerivatives::getSpatialVelocityDerivativeWrtVelocities(
    std::size_t indexInSkeleton) const
{
  return getSpatialVelocityDerivativeWrtVelocities().col(indexInSkeleton);
}

//==============================================================================
Eigen::Vector6d
BodyNodeDerivatives::getSpatialVelocityDerivativeWrtVelocities(
    const DegreeOfFreedom* withRespectTo) const
{
  const auto index = withRespectTo->getIndexInSkeleton();

  return getSpatialVelocityDerivativeWrtVelocities(index);
}

//==============================================================================
BodyNodeDerivatives::GradientMatrix
BodyNodeDerivatives::getSpatialVelocityDerivativeWrtVelocities(
    const Joint* withRespectTo) const
{
  const auto index = withRespectTo->getIndexInSkeleton(0);
  const auto numDofs = withRespectTo->getNumDofs();

  return getSpatialVelocityDerivativeWrtVelocities().block(0, index, 6, numDofs);
}

//==============================================================================
Eigen::VectorXd
BodyNodeDerivatives::computeKineticEnergyDerivativeWrtPositions() const
{
  const BodyNode* thisBodyNode = mComposite;
  const Eigen::Matrix6d& G = thisBodyNode->getInertia().getSpatialTensor();
  const Eigen::Vector6d& V = thisBodyNode->getSpatialVelocity();

  return V.transpose() * G * getSpatialVelocityDerivativeWrtPositions();
}

//==============================================================================
Eigen::VectorXd
BodyNodeDerivatives::computeKineticEnergyDerivativeWrtVelocities() const
{
  const BodyNode* thisBodyNode = mComposite;
  const Eigen::Matrix6d& G = thisBodyNode->getInertia().getSpatialTensor();
  const Eigen::Vector6d& V = thisBodyNode->getSpatialVelocity();

  return V.transpose() * G * getSpatialVelocityDerivativeWrtVelocities();
}

//==============================================================================
Eigen::VectorXd
BodyNodeDerivatives::computeLagrangianDerivativeWrtPositions() const
{
  return computeKineticEnergyDerivativeWrtPositions();
}

//==============================================================================
Eigen::VectorXd
BodyNodeDerivatives::computeLagrangianDerivativeWrtVelocities() const
{
  return computeKineticEnergyDerivativeWrtVelocities();
}

//==============================================================================
void BodyNodeDerivatives::setComposite(common::Composite* newComposite)
{
  Base::setComposite(newComposite);

  auto* bodyNode = mComposite;
  auto* joint = bodyNode->getParentJoint();
  joint->onPositionUpdatedAdded.connect(
        [=](const Joint* /*joint*/)
        {
          this->dirtySpatialVelocityDerivativeWrtPositions();
          this->dirtySpatialVelocityDerivativeWrtVelocities();
        } );
  joint->onVelocityUpdatedAdded.connect(
        [=](const Joint* /*joint*/)
        {
          this->dirtySpatialVelocityDerivativeWrtPositions();
        } );

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
void BodyNodeDerivatives::dirtySpatialVelocityDerivativeWrtPositions()
{
  if (mNeedSpatialVelocityDerivativeWrtPositionsUpdate)
    return;

  mNeedSpatialVelocityDerivativeWrtPositionsUpdate = true;

  auto* bodyNode = mComposite;
  for (auto i = 0u; i < bodyNode->getNumChildBodyNodes(); ++i)
  {
    auto* childBodyNode = bodyNode->getChildBodyNode(i);
    auto* childDeriv = childBodyNode->get<BodyNodeDerivatives>();
    childDeriv->dirtySpatialVelocityDerivativeWrtPositions();
  }
}

//==============================================================================
void BodyNodeDerivatives::dirtySpatialVelocityDerivativeWrtVelocities()
{
  if (mNeedSpatialVelocityDerivativeWrtVelocitiesUpdate)
    return;

  mNeedSpatialVelocityDerivativeWrtVelocitiesUpdate = true;

  auto* bodyNode = mComposite;
  for (auto i = 0u; i < bodyNode->getNumChildBodyNodes(); ++i)
  {
    auto* childBodyNode = bodyNode->getChildBodyNode(i);
    auto* childDeriv = childBodyNode->get<BodyNodeDerivatives>();
    childDeriv->dirtySpatialVelocityDerivativeWrtVelocities();
  }
}

} // namespace dynamics
} // namespace dart
