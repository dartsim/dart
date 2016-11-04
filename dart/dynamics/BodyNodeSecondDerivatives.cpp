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

#include "dart/dynamics/BodyNode.hpp"
#include "dart/dynamics/DegreeOfFreedom.hpp"

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
BodyNodeSecondDerivatives::getSpatialVelocityDerivativeWrtPos() const
{
  return mBodyNodeDerivatives->getSpatialVelocityDerivativeWrtPos();
}

//==============================================================================
Eigen::Vector6d BodyNodeSecondDerivatives::getSpatialVelocityDerivativeWrtPos(
    std::size_t indexInSkeleton) const
{
  return mBodyNodeDerivatives->getSpatialVelocityDerivativeWrtPos(
      indexInSkeleton);
}

//==============================================================================
Eigen::Vector6d BodyNodeSecondDerivatives::getSpatialVelocityDerivativeWrtPos(
    const DegreeOfFreedom* withRespectTo) const
{
  return mBodyNodeDerivatives->getSpatialVelocityDerivativeWrtPos(
      withRespectTo);
}

//==============================================================================
BodyNodeSecondDerivatives::SpatialVelocityDerivative
BodyNodeSecondDerivatives::getSpatialVelocityDerivativeWrtPos(
    const Joint* withRespectTo) const
{
  return mBodyNodeDerivatives->getSpatialVelocityDerivativeWrtPos(
      withRespectTo);
}

//==============================================================================
const BodyNodeSecondDerivatives::SpatialVelocityDerivative&
BodyNodeSecondDerivatives::getSpatialVelocityDerivativeWrtVel() const
{
  return mBodyNodeDerivatives->getSpatialVelocityDerivativeWrtVel();
}

//==============================================================================
Eigen::Vector6d BodyNodeSecondDerivatives::getSpatialVelocityDerivativeWrtVel(
    std::size_t indexInSkeleton) const
{
  return mBodyNodeDerivatives->getSpatialVelocityDerivativeWrtVel(
      indexInSkeleton);
}

//==============================================================================
Eigen::Vector6d BodyNodeSecondDerivatives::getSpatialVelocityDerivativeWrtVel(
    const DegreeOfFreedom* withRespectTo) const
{
  return mBodyNodeDerivatives->getSpatialVelocityDerivativeWrtVel(
      withRespectTo);
}

//==============================================================================
BodyNodeSecondDerivatives::SpatialVelocityDerivative
BodyNodeSecondDerivatives::getSpatialVelocityDerivativeWrtVel(
    const Joint* withRespectTo) const
{
  return mBodyNodeDerivatives->getSpatialVelocityDerivativeWrtVel(
      withRespectTo);
}

//==============================================================================
const BodyNodeSecondDerivatives::SpatialVelocitySecondDerivative&
BodyNodeSecondDerivatives::getSpatialVelocitySecondDerivativeWrtPos() const
{
  if (mNeedSpatialVelocitySecondDerivativeWrtPosUpdate)
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

      const auto* parentDeriv = parentBodyNode->get<BodyNodeDerivatives>();
      const auto* parent2ndDeriv
          = parentBodyNode->get<BodyNodeSecondDerivatives>();

      const auto& parentV = parentBodyNode->getSpatialVelocity();
      const auto& parentV_q = parentDeriv->getSpatialVelocityDerivativeWrtPos();
      const auto& parentV_q_q
          = parent2ndDeriv->getSpatialVelocitySecondDerivativeWrtPos();

      assert(static_cast<std::size_t>(mV_q_q.size()) == numSkelDofs);
      for (auto i = 0u; i < numSkelDofs; ++i)
        mV_q_q[i] = math::AdInvTJac(T, parentV_q_q[i]);

      for (auto iSkelIndex = 0u; iSkelIndex < numSkelDofs; ++iSkelIndex)
      {
        for (auto j = 0u; j < numJointDofs; ++j)
        {
          auto jSkelIndex = joint->getIndexInSkeleton(j);

          const Eigen::Vector6d tmp
              = -math::ad(S.col(j), math::AdInvT(T, parentV_q.col(iSkelIndex)));

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

          const Eigen::Vector6d tmp = math::ad(
              S.col(i), math::ad(S.col(j), math::AdInvT(T, parentV)));

          mV_q_q[iSkelIndex].col(jSkelIndex).noalias() += tmp;
        }
      }
    } // if (parentBodyNode)
    else
    {
      for (auto i = 0u; i < numSkelDofs; ++i)
        mV_q_q[i].setZero(6, numSkelDofs);
    }

    mNeedSpatialVelocitySecondDerivativeWrtPosUpdate = false;
  }

  return mV_q_q;
}

//==============================================================================
const BodyNodeSecondDerivatives::SpatialVelocitySecondDerivative&
BodyNodeSecondDerivatives::getSpatialVelocitySecondDerivativeWrtPosVel() const
{
  if (mNeedSpatialVelocitySecondDerivativeWrtPosVelUpdate)
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

      const auto* parentDeriv = parentBodyNode->get<BodyNodeDerivatives>();
      const auto* parent2ndDeriv
          = parentBodyNode->get<BodyNodeSecondDerivatives>();

      const auto& parentV_dq
          = parentDeriv->getSpatialVelocityDerivativeWrtVel();
      const auto& parentV_q_dq
          = parent2ndDeriv->getSpatialVelocitySecondDerivativeWrtPosVel();

      assert(static_cast<std::size_t>(mV_q_dq.size()) == numSkelDofs);
      for (auto i = 0u; i < numSkelDofs; ++i)
        mV_q_dq[i] = math::AdInvTJac(T, parentV_q_dq[i]);

      for (auto i = 0u; i < numJointDofs; ++i)
      {
        auto iSkelIndex = joint->getIndexInSkeleton(i);

        for (auto jSkelIndex = 0u; jSkelIndex < numSkelDofs; ++jSkelIndex)
        {
          const Eigen::Vector6d tmp = -math::ad(
              S.col(i), math::AdInvT(T, parentV_dq.col(jSkelIndex)));

          mV_q_dq[iSkelIndex].col(jSkelIndex).noalias() += tmp;
        }
      }
    } // if (parentBodyNode)
    else
    {
      for (auto i = 0u; i < numSkelDofs; ++i)
        mV_q_dq[i].setZero(6, numSkelDofs);
    }

    mNeedSpatialVelocitySecondDerivativeWrtPosVelUpdate = false;
  }

  return mV_q_dq;
}

//==============================================================================
const BodyNodeSecondDerivatives::SpatialVelocitySecondDerivative&
BodyNodeSecondDerivatives::getSpatialVelocitySecondDerivativeWrtVel() const
{
#ifndef NDEBUG
  const auto numDofs = mComposite->getSkeleton()->getNumDofs();
  for (auto i = 0u; i < numDofs; ++i)
  {
    assert(
        mV_dq_dq[i]
        == Eigen::MatrixXd::Zero(6, mComposite->getSkeleton()->getNumDofs()));
  }
#endif
  return mV_dq_dq;
}

//==============================================================================
Eigen::VectorXd
BodyNodeSecondDerivatives::computeKineticEnergyGradientWrtPos() const
{
  return mBodyNodeDerivatives->computeKineticEnergyGradientWrtPos();
}

//==============================================================================
Eigen::VectorXd
BodyNodeSecondDerivatives::computeKineticEnergyGradientWrtVel() const
{
  return mBodyNodeDerivatives->computeKineticEnergyGradientWrtVel();
}

//==============================================================================
Eigen::VectorXd
BodyNodeSecondDerivatives::computeLagrangianGradientWrtPos() const
{
  return mBodyNodeDerivatives->computeLagrangianGradientWrtPos();
}

//==============================================================================
Eigen::VectorXd
BodyNodeSecondDerivatives::computeLagrangianGradientWrtVel() const
{
  return mBodyNodeDerivatives->computeLagrangianGradientWrtVel();
}

//==============================================================================
Eigen::MatrixXd
BodyNodeSecondDerivatives::computeKineticEnergyHessianWrtPosPos() const
{
  const BodyNode* bodyNode = mComposite;
  const auto numSkelDofs = bodyNode->getSkeleton()->getNumDofs();

  const Eigen::Matrix6d& G = bodyNode->getInertia().getSpatialTensor();
  const Eigen::Vector6d& V = bodyNode->getSpatialVelocity();

  const BodyNodeSecondDerivatives::SpatialVelocityDerivative& V_q
      = getSpatialVelocityDerivativeWrtPos();

  Eigen::MatrixXd hessian = V_q.transpose() * G * V_q;

  const auto& V_q_q = getSpatialVelocitySecondDerivativeWrtPos();
  const Eigen::Matrix<double, 1, 6> VTG = V.transpose() * G;
  for (auto i = 0u; i < numSkelDofs; ++i)
    hessian.row(i).noalias() += VTG * V_q_q[i];

  return hessian;
}

//==============================================================================
Eigen::MatrixXd
BodyNodeSecondDerivatives::computeKineticEnergyHessianWrtPosVel() const
{
  const BodyNode* bodyNode = mComposite;
  const auto numSkelDofs = bodyNode->getSkeleton()->getNumDofs();

  const Eigen::Matrix6d& G = bodyNode->getInertia().getSpatialTensor();
  const Eigen::Vector6d& V = bodyNode->getSpatialVelocity();

  const BodyNodeSecondDerivatives::SpatialVelocityDerivative& V_q
      = getSpatialVelocityDerivativeWrtPos();
  const BodyNodeSecondDerivatives::SpatialVelocityDerivative& V_dq
      = getSpatialVelocityDerivativeWrtVel();

  Eigen::MatrixXd hessian = V_q.transpose() * G * V_dq;

  const auto& V_q_dq = getSpatialVelocitySecondDerivativeWrtPosVel();
  const Eigen::Matrix<double, 1, 6> VTG = V.transpose() * G;
  for (auto i = 0u; i < numSkelDofs; ++i)
    hessian.row(i).noalias() += VTG * V_q_dq[i];

  return hessian;
}

//==============================================================================
Eigen::MatrixXd
BodyNodeSecondDerivatives::computeKineticEnergyHessianWrtVelVel() const
{
  const BodyNode* bodyNode = mComposite;
  // const auto numSkelDofs = bodyNode->getSkeleton()->getNumDofs();

  const Eigen::Matrix6d& G = bodyNode->getInertia().getSpatialTensor();
  // const Eigen::Vector6d& V = bodyNode->getSpatialVelocity();

  const BodyNodeSecondDerivatives::SpatialVelocityDerivative& V_dq
      = getSpatialVelocityDerivativeWrtVel();

  Eigen::MatrixXd hessian = V_dq.transpose() * G * V_dq;

  // const auto& V_dq_dq = getSpatialVelocitySecondDerivativeWrtVel();
  // const Eigen::MatrixXd VTG = V.transpose() * G;
  // for (auto i = 0u; i < numSkelDofs; ++i)
  //  hessian.row(i).noalias() += VTG * V_dq_dq[i];

  return hessian;
}

//==============================================================================
Eigen::MatrixXd
BodyNodeSecondDerivatives::computeLagrangianHessianWrtPosPos() const
{
  return computeKineticEnergyHessianWrtPosPos();
}

//==============================================================================
Eigen::MatrixXd
BodyNodeSecondDerivatives::computeLagrangianHessianWrtPosVel() const
{
  return computeKineticEnergyHessianWrtPosVel();
}

//==============================================================================
Eigen::MatrixXd
BodyNodeSecondDerivatives::computeLagrangianHessianWrtVelVel() const
{
  return computeKineticEnergyHessianWrtVelVel();
}

//==============================================================================
void BodyNodeSecondDerivatives::dirtySpatialVelocitySecondDerivativeWrtPos()
{
  if (mNeedSpatialVelocitySecondDerivativeWrtPosUpdate)
    return;

  mNeedSpatialVelocitySecondDerivativeWrtPosUpdate = true;

  auto* bodyNode = mComposite;
  for (auto i = 0u; i < bodyNode->getNumChildBodyNodes(); ++i)
  {
    auto* childBodyNode = bodyNode->getChildBodyNode(i);
    auto* childDeriv = childBodyNode->get<BodyNodeSecondDerivatives>();
    childDeriv->dirtySpatialVelocitySecondDerivativeWrtPos();
  }
}

//==============================================================================
void BodyNodeSecondDerivatives::dirtySpatialVelocitySecondDerivativeWrtPosVel()
{
  if (mNeedSpatialVelocitySecondDerivativeWrtPosVelUpdate)
    return;

  mNeedSpatialVelocitySecondDerivativeWrtPosVelUpdate = true;

  auto* bodyNode = mComposite;
  for (auto i = 0u; i < bodyNode->getNumChildBodyNodes(); ++i)
  {
    auto* childBodyNode = bodyNode->getChildBodyNode(i);
    auto* childDeriv = childBodyNode->get<BodyNodeSecondDerivatives>();
    childDeriv->dirtySpatialVelocitySecondDerivativeWrtPosVel();
  }
}

//==============================================================================
void BodyNodeSecondDerivatives::setComposite(common::Composite* newComposite)
{
  Base::setComposite(newComposite);

  auto* bodyNode = mComposite;
  auto* joint = bodyNode->getParentJoint();
  joint->onPositionUpdatedAdded.connect([=](const Joint* /*joint*/) {
    this->dirtySpatialVelocitySecondDerivativeWrtPos();
    this->dirtySpatialVelocitySecondDerivativeWrtPosVel();
  });
  joint->onVelocityUpdatedAdded.connect([=](const Joint* /*joint*/) {
    this->dirtySpatialVelocitySecondDerivativeWrtPos();
    this->dirtySpatialVelocitySecondDerivativeWrtPosVel();
  });

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
