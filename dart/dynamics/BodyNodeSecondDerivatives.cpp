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

#include "dart/dynamics/BodyNodeSecondDerivatives.hpp"

#include "dart/dynamics/DegreeOfFreedom.hpp"
#include "dart/dynamics/BodyNode.hpp"

namespace dart {
namespace dynamics {

//==============================================================================
std::unique_ptr<common::Aspect> BodyNodeSecondDerivatives::cloneAspect() const
{
  // TODO(JS): not implemented
  return common::make_unique<BodyNodeSecondDerivatives>();
}

//==============================================================================
const BodyNodeSecondDerivatives::SpatialVelocityDerivative&
BodyNodeSecondDerivatives::getSpatialVelocityDerivativeWrtPositions() const
{
  return mBodyNodeDerivatives->getSpatialVelocityDerivativeWrtPositions();
}

//==============================================================================
Eigen::Vector6d
BodyNodeSecondDerivatives::getSpatialVelocityDerivativeWrtPositions(
    std::size_t indexInSkeleton) const
{
  return mBodyNodeDerivatives->getSpatialVelocityDerivativeWrtPositions(
        indexInSkeleton);
}

//==============================================================================
Eigen::Vector6d
BodyNodeSecondDerivatives::getSpatialVelocityDerivativeWrtPositions(
    const DegreeOfFreedom* withRespectTo) const
{
  return mBodyNodeDerivatives->getSpatialVelocityDerivativeWrtPositions(
        withRespectTo);
}

//==============================================================================
BodyNodeSecondDerivatives::SpatialVelocityDerivative
BodyNodeSecondDerivatives::getSpatialVelocityDerivativeWrtPositions(
    const Joint* withRespectTo) const
{
  return mBodyNodeDerivatives->getSpatialVelocityDerivativeWrtPositions(
        withRespectTo);
}

//==============================================================================
const BodyNodeSecondDerivatives::SpatialVelocityDerivative&
BodyNodeSecondDerivatives::getSpatialVelocityDerivativeWrtVelocities() const
{
  return mBodyNodeDerivatives->getSpatialVelocityDerivativeWrtVelocities();
}

//==============================================================================
Eigen::Vector6d
BodyNodeSecondDerivatives::getSpatialVelocityDerivativeWrtVelocities(
    std::size_t indexInSkeleton) const
{
  return mBodyNodeDerivatives->getSpatialVelocityDerivativeWrtVelocities(
        indexInSkeleton);
}

//==============================================================================
Eigen::Vector6d
BodyNodeSecondDerivatives::getSpatialVelocityDerivativeWrtVelocities(
    const DegreeOfFreedom* withRespectTo) const
{
  return mBodyNodeDerivatives->getSpatialVelocityDerivativeWrtVelocities(
        withRespectTo);
}

//==============================================================================
BodyNodeSecondDerivatives::SpatialVelocityDerivative
BodyNodeSecondDerivatives::getSpatialVelocityDerivativeWrtVelocities(
    const Joint* withRespectTo) const
{
  return mBodyNodeDerivatives->getSpatialVelocityDerivativeWrtVelocities(
        withRespectTo);
}

//==============================================================================
const BodyNodeSecondDerivatives::SpatialVelocitySecondDerivative&
BodyNodeSecondDerivatives::
getSpatialVelocitySecondDerivativeWrtPositions() const
{
  if (mNeedSpatialVelocitySecondDerivativeWrtPositionsUpdate)
  {
    const auto* bodyNode = mComposite;
    const auto* parentBodyNode = bodyNode->getParentBodyNode();

    const auto* joint = bodyNode->getParentJoint();
    const auto* skeleton = bodyNode->getSkeleton().get();
    const auto numSkelDofs = skeleton->getNumDofs();

    if (parentBodyNode)
    {
      const auto numJointDofs = joint->getNumDofs();

      const auto& T = joint->getRelativeTransform();
      const auto& S = joint->getRelativeJacobian();

      const auto* parentDerivatives
          = parentBodyNode->get<BodyNodeDerivatives>();
      const auto* parentSecondDerivatives
          = parentBodyNode->get<BodyNodeSecondDerivatives>();

      const auto& parentV = parentBodyNode->getSpatialVelocity();
      const auto& parentV_q
          = parentDerivatives->getSpatialVelocityDerivativeWrtPositions();
      const auto& parentV_q_q
          = parentSecondDerivatives->getSpatialVelocitySecondDerivativeWrtPositions();

      assert(static_cast<std::size_t>(mV_q_q.size()) == numSkelDofs);
      for (auto i = 0u; i < numSkelDofs; ++i)
        mV_q_q[i] = math::AdInvTJac(T, parentV_q_q[i]);

      for (auto iSkelIndex = 0u; iSkelIndex < numSkelDofs; ++iSkelIndex)
      {
        for (auto j = 0u; j < numJointDofs; ++j)
        {
          auto jSkelIndex = joint->getIndexInSkeleton(j);

          const Eigen::Vector6d tmp =
              -math::ad(S.col(j), math::AdInvT(T, parentV_q.col(iSkelIndex)));

          mV_q_q[iSkelIndex].col(jSkelIndex).noalias() += tmp;
          mV_q_q[jSkelIndex].col(iSkelIndex).noalias() += tmp;
        }
      }

      for (auto i = 0u; i < numJointDofs; ++i)
      {
        auto iSkelIndex = joint->getIndexInSkeleton(i);

        for (auto j = 0u; j < numJointDofs; ++j)
        {
          auto jSkelIndex = joint->getIndexInSkeleton(j);

          const Eigen::Vector6d tmp =
              math::ad(S.col(i), math::ad(S.col(j), math::AdInvT(T, parentV)));

          mV_q_q[iSkelIndex].col(jSkelIndex).noalias() += tmp;
        }
      }
    } // if (parentBodyNode)
    else
    {
      for (auto i = 0u; i < numSkelDofs; ++i)
        mV_q_q[i].setZero(6, numSkelDofs);
    }

    mNeedSpatialVelocitySecondDerivativeWrtPositionsUpdate = false;
  }
  
  return mV_q_q;
}

//==============================================================================
const BodyNodeSecondDerivatives::SpatialVelocitySecondDerivative&
BodyNodeSecondDerivatives::
getSpatialVelocitySecondDerivativeWrtPositionsVelocities() const
{
  if (mNeedSpatialVelocitySecondDerivativeWrtPositionsVelocitiesUpdate)
  {
    const auto* bodyNode = mComposite;
    const auto* parentBodyNode = bodyNode->getParentBodyNode();

    const auto* joint = bodyNode->getParentJoint();
    const auto* skeleton = bodyNode->getSkeleton().get();
    const auto numSkelDofs = skeleton->getNumDofs();

    if (parentBodyNode)
    {
      const auto numJointDofs = joint->getNumDofs();

      const auto& T = joint->getRelativeTransform();
      const auto& S = joint->getRelativeJacobian();

      const auto* parentDerivatives
          = parentBodyNode->get<BodyNodeDerivatives>();
      const auto* parentSecondDerivatives
          = parentBodyNode->get<BodyNodeSecondDerivatives>();

      const auto& parentV_dq
          = parentDerivatives->getSpatialVelocityDerivativeWrtVelocities();
      const auto& parentV_q_dq
          = parentSecondDerivatives->getSpatialVelocitySecondDerivativeWrtPositionsVelocities();

      assert(static_cast<std::size_t>(mV_q_dq.size()) == numSkelDofs);
      for (auto i = 0u; i < numSkelDofs; ++i)
        mV_q_dq[i] = math::AdInvTJac(T, parentV_q_dq[i]);

      for (auto i = 0u; i < numJointDofs; ++i)
      {
        auto iSkelIndex = joint->getIndexInSkeleton(i);

        for (auto jSkelIndex = 0u; jSkelIndex < numSkelDofs; ++jSkelIndex)
        {
          const Eigen::Vector6d tmp =
              -math::ad(S.col(i), math::AdInvT(T, parentV_dq.col(jSkelIndex)));

          mV_q_dq[iSkelIndex].col(jSkelIndex).noalias() += tmp;
        }
      }
    } // if (parentBodyNode)
    else
    {
      for (auto i = 0u; i < numSkelDofs; ++i)
        mV_q_dq[i].setZero(6, numSkelDofs);
    }

    mNeedSpatialVelocitySecondDerivativeWrtPositionsVelocitiesUpdate = false;
  }

  return mV_q_dq;
}

//==============================================================================
const BodyNodeSecondDerivatives::SpatialVelocitySecondDerivative&
BodyNodeSecondDerivatives::
getSpatialVelocitySecondDerivativeWrtVelocities() const
{
#ifndef NDEBUG
  const auto numDofs = mComposite->getSkeleton()->getNumDofs();
  for (auto i = 0u; i < numDofs; ++i)
  {
    assert(mV_dq_dq[i]
           == Eigen::MatrixXd::Zero(6,
                                    mComposite->getSkeleton()->getNumDofs()));
  }
#endif
  return mV_dq_dq;
}

//==============================================================================
Eigen::VectorXd
BodyNodeSecondDerivatives::computeKineticEnergyGradientWrtPositions() const
{
  return mBodyNodeDerivatives->computeKineticEnergyGradientWrtPositions();
}

//==============================================================================
Eigen::VectorXd
BodyNodeSecondDerivatives::computeKineticEnergyGradientWrtVelocities() const
{
  return mBodyNodeDerivatives->computeKineticEnergyGradientWrtVelocities();
}

//==============================================================================
Eigen::VectorXd
BodyNodeSecondDerivatives::computeLagrangianGradientWrtPositions() const
{
  return mBodyNodeDerivatives->computeLagrangianGradientWrtPositions();
}

//==============================================================================
Eigen::VectorXd
BodyNodeSecondDerivatives::computeLagrangianGradientWrtVelocities() const
{
  return mBodyNodeDerivatives->computeLagrangianGradientWrtVelocities();
}

//==============================================================================
Eigen::MatrixXd
BodyNodeSecondDerivatives::computeKineticEnergyHessianWrtPositions() const
{
  const BodyNode* bodyNode = mComposite;
  const auto numSkelDofs = bodyNode->getSkeleton()->getNumDofs();

  const Eigen::Matrix6d& G = bodyNode->getInertia().getSpatialTensor();
  const Eigen::Vector6d& V = bodyNode->getSpatialVelocity();

  const BodyNodeSecondDerivatives::SpatialVelocityDerivative& V_q
      = getSpatialVelocityDerivativeWrtPositions();

  Eigen::MatrixXd hessian = V_q.transpose() * G * V_q;

  const auto& V_q_q = getSpatialVelocitySecondDerivativeWrtPositions();
  const Eigen::Matrix<double, 1, 6> VTG = V.transpose() * G;
  for (auto i = 0u; i < numSkelDofs; ++i)
    hessian.row(i).noalias() += VTG * V_q_q[i];

  return hessian;
}

//==============================================================================
Eigen::MatrixXd
BodyNodeSecondDerivatives::computeKineticEnergyHessianWrtPositionsVelocities() const
{
  const BodyNode* bodyNode = mComposite;
  const auto numSkelDofs = bodyNode->getSkeleton()->getNumDofs();

  const Eigen::Matrix6d& G = bodyNode->getInertia().getSpatialTensor();
  const Eigen::Vector6d& V = bodyNode->getSpatialVelocity();

  const BodyNodeSecondDerivatives::SpatialVelocityDerivative& V_q
      = getSpatialVelocityDerivativeWrtPositions();
  const BodyNodeSecondDerivatives::SpatialVelocityDerivative& V_dq
      = getSpatialVelocityDerivativeWrtVelocities();

  Eigen::MatrixXd hessian = V_q.transpose() * G * V_dq;

  const auto& V_q_dq = getSpatialVelocitySecondDerivativeWrtPositionsVelocities();
  const Eigen::Matrix<double, 1, 6> VTG = V.transpose() * G;
  for (auto i = 0u; i < numSkelDofs; ++i)
    hessian.row(i).noalias() += VTG * V_q_dq[i];

  return hessian;
}

//==============================================================================
Eigen::MatrixXd
BodyNodeSecondDerivatives::computeKineticEnergyHessianWrtVelocities() const
{
  const BodyNode* bodyNode = mComposite;
  //const auto numSkelDofs = bodyNode->getSkeleton()->getNumDofs();

  const Eigen::Matrix6d& G = bodyNode->getInertia().getSpatialTensor();
  //const Eigen::Vector6d& V = bodyNode->getSpatialVelocity();

  const BodyNodeSecondDerivatives::SpatialVelocityDerivative& V_dq
      = getSpatialVelocityDerivativeWrtVelocities();

  Eigen::MatrixXd hessian = V_dq.transpose() * G * V_dq;

  //const auto& V_dq_dq = getSpatialVelocitySecondDerivativeWrtVelocities();
  //const Eigen::MatrixXd VTG = V.transpose() * G;
  //for (auto i = 0u; i < numSkelDofs; ++i)
  //  hessian.row(i).noalias() += VTG * V_dq_dq[i];

  return hessian;
}

//==============================================================================
Eigen::MatrixXd
BodyNodeSecondDerivatives::computeLagrangianHessianWrtPositions() const
{
  return computeKineticEnergyHessianWrtPositions();
}

//==============================================================================
Eigen::MatrixXd
BodyNodeSecondDerivatives::computeLagrangianHessianWrtPositionsVelocities() const
{
  return computeKineticEnergyHessianWrtPositionsVelocities();
}

//==============================================================================
Eigen::MatrixXd
BodyNodeSecondDerivatives::computeLagrangianHessianWrtVelocities() const
{
  return computeKineticEnergyHessianWrtVelocities();
}

//==============================================================================
void BodyNodeSecondDerivatives::dirtySpatialVelocitySecondDerivativeWrtPositions()
{
  if (mNeedSpatialVelocitySecondDerivativeWrtPositionsUpdate)
    return;

  mNeedSpatialVelocitySecondDerivativeWrtPositionsUpdate = true;

  auto* bodyNode = mComposite;
  for (auto i = 0u; i < bodyNode->getNumChildBodyNodes(); ++i)
  {
    auto* childBodyNode = bodyNode->getChildBodyNode(i);
    auto* childDeriv = childBodyNode->get<BodyNodeSecondDerivatives>();
    childDeriv->dirtySpatialVelocitySecondDerivativeWrtPositions();
  }
}

//==============================================================================
void BodyNodeSecondDerivatives::dirtySpatialVelocitySecondDerivativeWrtPositionsVelocities()
{
  if (mNeedSpatialVelocitySecondDerivativeWrtPositionsVelocitiesUpdate)
    return;

  mNeedSpatialVelocitySecondDerivativeWrtPositionsVelocitiesUpdate = true;

  auto* bodyNode = mComposite;
  for (auto i = 0u; i < bodyNode->getNumChildBodyNodes(); ++i)
  {
    auto* childBodyNode = bodyNode->getChildBodyNode(i);
    auto* childDeriv = childBodyNode->get<BodyNodeSecondDerivatives>();
    childDeriv->dirtySpatialVelocitySecondDerivativeWrtPositionsVelocities();
  }
}

//==============================================================================
void BodyNodeSecondDerivatives::setComposite(common::Composite* newComposite)
{
  Base::setComposite(newComposite);

  auto* bodyNode = mComposite;
  auto* joint = bodyNode->getParentJoint();
  joint->onPositionUpdatedAdded.connect(
        [=](const Joint* /*joint*/)
        {
          this->dirtySpatialVelocitySecondDerivativeWrtPositions();
          this->dirtySpatialVelocitySecondDerivativeWrtPositionsVelocities();
        } );
  joint->onVelocityUpdatedAdded.connect(
        [=](const Joint* /*joint*/)
        {
          this->dirtySpatialVelocitySecondDerivativeWrtPositions();
          this->dirtySpatialVelocitySecondDerivativeWrtPositionsVelocities();
        } );

  const auto skeleton = bodyNode->getSkeleton();
  const auto numDofs = skeleton->getNumDofs();

  assert(skeleton);

  mBodyNodeDerivatives = bodyNode->getOrCreateAspect<BodyNodeDerivatives>();

  mV_q_q.resize(numDofs);
  mV_q_dq.resize(numDofs);
  mV_dq_dq.resize(numDofs);

  for (auto i = 0u; i < numDofs; ++i)
  {
    mV_q_q[i].resize(6, numDofs);
    mV_q_dq[i].resize(6, numDofs);
    mV_dq_dq[i].setZero(6, numDofs); // note that this is constantly zero
  }

  // TODO(JS): These should be updated when the structure of skeleton is
  // modified. We might want to consider adding signal to skeleton for this.
}

} // namespace dynamics
} // namespace dart
