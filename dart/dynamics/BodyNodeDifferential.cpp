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

#include "dart/dynamics/BodyNodeDifferential.hpp"

#include "dart/dynamics/DegreeOfFreedom.hpp"
#include "dart/dynamics/BodyNode.hpp"

namespace dart {
namespace dynamics {

namespace detail {

//==============================================================================
BodyNodeDifferentialState::BodyNodeDifferentialState()
  : mV_q_q(std::vector<GradientMatrix>()),
    mV_q_dq(std::vector<GradientMatrix>()),
    mV_dq_dq(std::vector<GradientMatrix>())
{
  // Do nothing
}

} // namespace detail

//==============================================================================
BodyNodeDifferential::BodyNodeDifferential(const StateData& state)
{
  mState = state;
}

//==============================================================================
void BodyNodeDifferential::updateBodyVelocityGradients()
{
  const auto* thisBodyNode = mComposite;
  const auto* thisJoint = thisBodyNode->getParentJoint();

  if (thisJoint->getNumDofs() == 0)
    return;

  const auto* thisParentBodyNode = thisBodyNode->getParentBodyNode();

  if (thisParentBodyNode)
  {
    const auto* thisParentBodyNodeDifferential
        = thisParentBodyNode->get<BodyNodeDifferential>();

    mState.mV_q
        = math::AdInvTJac(
          thisJoint->getRelativeTransform(),
          thisParentBodyNodeDifferential->mState.mV_q);

    mState.mV_dq
        = math::AdInvTJac(
          thisJoint->getRelativeTransform(),
          thisParentBodyNodeDifferential->mState.mV_dq);
  }
  else
  {
    mState.mV_q.setZero();
    mState.mV_dq.setZero();
  }

  const auto numDofs = thisJoint->getNumDofs();

  if (numDofs == 0)
    return;

  const auto index = thisJoint->getDof(0)->getIndexInSkeleton();

  mState.mV_q.block(0, index, 6, numDofs)
      += math::adJac(thisBodyNode->getSpatialVelocity(),
                     thisJoint->getRelativeJacobian());

  mState.mV_dq.block(0, index, 6, numDofs) += thisJoint->getRelativeJacobian();
}

//==============================================================================
void BodyNodeDifferential::updateSpatialVelocityHessian()
{
  const auto* thisBodyNode = mComposite;
  const auto* thisJoint = thisBodyNode->getParentJoint();

  if (thisJoint->getNumDofs() == 0)
    return;

  const auto nBodyNodes = thisBodyNode->getSkeleton()->getNumBodyNodes();
  const auto* thisParentBodyNode = thisBodyNode->getParentBodyNode();

//  if (thisParentBodyNode)
//  {
//    const auto* thisParentBodyNodeDifferential
//        = thisParentBodyNode->get<BodyNodeDifferential>();

//    for (auto i = 0u; i < nBodyNodes; ++i))

//    mState.mV_q_q
//        = math::AdInvTJac(
//          thisJoint->getRelativeTransform(),
//          thisParentBodyNodeDifferential->mState.mV_q_q);

//    mState.mV_dq_dq
//        = math::AdInvTJac(
//          thisJoint->getRelativeTransform(),
//          thisParentBodyNodeDifferential->mState.mV_dq_dq);
//  }
//  else
//  {
//    mState.mV_q_q.setZero();
//    mState.mV_q_dq.setZero();
//    mState.mV_dq_dq.setZero();
//  }
}

//==============================================================================
Eigen::Vector6d
BodyNodeDifferential::getBodyVelocityGradientWrtQ(
    std::size_t indexInSkeleton) const
{
  return mState.mV_q.col(indexInSkeleton);
}

//==============================================================================
Eigen::Vector6d
BodyNodeDifferential::getBodyVelocityGradientWrtQ(
    const DegreeOfFreedom* withRespectTo) const
{
  const auto index = withRespectTo->getIndexInSkeleton();

  return getBodyVelocityGradientWrtQ(index);
}

//==============================================================================
BodyNodeDifferential::GradientMatrix
BodyNodeDifferential::getBodyVelocityGradientWrtQ(
    const Joint* withRespectTo) const
{
  const auto index = withRespectTo->getIndexInSkeleton(0);
  const auto numDofs = withRespectTo->getNumDofs();

  return mState.mV_q.block(0, index, 6, numDofs);
}

//==============================================================================
BodyNodeDifferential::GradientMatrix
BodyNodeDifferential::getBodyVelocityGradientWrtQ() const
{
  return mState.mV_q;
}

//==============================================================================
Eigen::Vector6d
BodyNodeDifferential::getBodyVelocityGradientWrtDQ(
    std::size_t indexInSkeleton) const
{
  return mState.mV_dq.col(indexInSkeleton);
}

//==============================================================================
Eigen::Vector6d
BodyNodeDifferential::getBodyVelocityGradientWrtDQ(
    const DegreeOfFreedom* withRespectTo) const
{
  const auto index = withRespectTo->getIndexInSkeleton();

  return getBodyVelocityGradientWrtDQ(index);
}

//==============================================================================
BodyNodeDifferential::GradientMatrix
BodyNodeDifferential::getBodyVelocityGradientWrtDQ(
    const Joint* withRespectTo) const
{
  const auto index = withRespectTo->getIndexInSkeleton(0);
  const auto numDofs = withRespectTo->getNumDofs();

  return mState.mV_dq.block(0, index, 6, numDofs);
}

//==============================================================================
BodyNodeDifferential::GradientMatrix
BodyNodeDifferential::getBodyVelocityGradientWrtDQ() const
{
  return mState.mV_dq;
}

//==============================================================================
void BodyNodeDifferential::print()
{
  std::cout << mState.mV_q << std::endl;;
}

//==============================================================================
void BodyNodeDifferential::setComposite(common::Composite* newComposite)
{
  Base::setComposite(newComposite);

  const auto* bodyNode = dynamic_cast<BodyNode*>(newComposite);
  const auto skeleton = bodyNode->getSkeleton();
  const auto numDofs = skeleton->getNumDofs();

  assert(skeleton);

  mState.mV_q.resize(6, numDofs);
  mState.mV_dq.resize(6, numDofs);

  mState.mV_q_q.resize(numDofs);
  mState.mV_q_dq.resize(numDofs);
  mState.mV_dq_dq.resize(numDofs);

  for (auto i = 0u; i < numDofs; ++i)
  {
    mState.mV_q_q[i].resize(6, numDofs);
    mState.mV_q_dq[i].resize(6, numDofs);
    mState.mV_dq_dq[i].resize(6, numDofs);
  }

  // TODO(JS): These should be updated when the structure of skeleton is
  // modified. We might want to consider adding signal to skeleton for this.
}

} // namespace dynamics
} // namespace dart
