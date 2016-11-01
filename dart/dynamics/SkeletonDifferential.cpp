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

#include "dart/dynamics/SkeletonDifferential.hpp"

#include "dart/dynamics/BodyNode.hpp"
#include "dart/dynamics/BodyNodeDifferential.hpp"

namespace dart {
namespace dynamics {

namespace detail {

//==============================================================================
SkeletonLagrangianAspectState::SkeletonLagrangianAspectState()
{
  // Do nothing
}

} // namespace detail

//==============================================================================
SkeletonDifferential::SkeletonDifferential(const StateData& state)
{
  mState = state;
}

//==============================================================================
void SkeletonDifferential::updateSpatialVelocityGradients()
{
  for (auto* bodyNode : mComposite->getBodyNodes())
    bodyNode->get<BodyNodeDifferential>()->updateSpatialVelocityGradients();
}

//==============================================================================
void SkeletonDifferential::updateBodyVelocityHessians()
{
  for (auto* bodyNode : mComposite->getBodyNodes())
    bodyNode->get<BodyNodeDifferential>()->updateSpatialVelocityHessians();
}

//==============================================================================
const Eigen::VectorXd&
SkeletonDifferential::computeLagrangianGradientWrtPositions()
{
  updateSpatialVelocityGradients();
  // TODO(JS): separate into V/q and V/dq

  mState.mDM_GradientOfLagrangian_q.setZero();

  for (auto* bodyNode : mComposite->getBodyNodes())
  {
    auto bodyNodeDifferential = bodyNode->get<BodyNodeDifferential>();
    mState.mDM_GradientOfLagrangian_q
        += bodyNodeDifferential->computeLagrangianGradientWrtPositions();
  }

  return mState.mDM_GradientOfLagrangian_q;
}

//==============================================================================
const Eigen::VectorXd&
SkeletonDifferential::computeLagrangianGradientWrtVelocities()
{
  updateSpatialVelocityGradients();
  // TODO(JS): separate into V/q and V/dq

  mState.mDM_GradientOfLagrangian_dq.setZero();

  for (auto* bodyNode : mComposite->getBodyNodes())
  {
    auto bodyNodeDifferential = bodyNode->get<BodyNodeDifferential>();
    mState.mDM_GradientOfLagrangian_dq
        += bodyNodeDifferential->computeLagrangianGradientWrtVelocities();
  }

  return mState.mDM_GradientOfLagrangian_dq;
}

//==============================================================================
const Eigen::MatrixXd&
SkeletonDifferential::computeLagrangianHessianWrtPositionsPositions()
{
  updateSpatialVelocityGradients();
  // TODO(JS): separate into V/q and V/dq

  mState.mDM_GradientOfLagrangian_dq.setZero();

  for (auto* bodyNode : mComposite->getBodyNodes())
  {
    auto bodyNodeDifferential = bodyNode->get<BodyNodeDifferential>();
    mState.mDM_GradientOfLagrangian_dq
        += bodyNodeDifferential->computeLagrangianGradientWrtVelocities();
  }

  return mState.mDM_HessianOfLagrangian_q_q;
}

//==============================================================================
const Eigen::MatrixXd&
SkeletonDifferential::computeLagrangianHessianWrtPositionsVelocities()
{
  return mState.mDM_HessianOfLagrangian_q_dq;
}

//==============================================================================
const Eigen::MatrixXd&
SkeletonDifferential::computeLagrangianHessianWrtVelocitiesVelocities()
{
  return mState.mDM_HessianOfLagrangian_dq_dq;
}

//==============================================================================
SkeletonDifferential::GradientMatrix
SkeletonDifferential::getBodyVelocityGradientWrtQ(
    std::size_t bodyNodeIndexInSkeleton) const
{
  // TODO(JS): emit warning if the index is not valid
  const auto* bodyNode = mComposite->getBodyNode(bodyNodeIndexInSkeleton);
  const auto* bodyNodeDifferential = bodyNode->get<BodyNodeDifferential>();

  assert(bodyNodeDifferential);

  return bodyNodeDifferential->getBodyVelocityGradientWrtQ();
}

//==============================================================================
Eigen::Vector6d SkeletonDifferential::getBodyVelocityGradientWrtQ(
    std::size_t bodyNodeIndexInSkeleton,
    std::size_t withRespectTo) const
{
  // TODO(JS): emit warning if the index is not valid
  const auto* bodyNode = mComposite->getBodyNode(bodyNodeIndexInSkeleton);
  const auto* bodyNodeDifferential = bodyNode->get<BodyNodeDifferential>();

  assert(bodyNodeDifferential);

  return bodyNodeDifferential->getBodyVelocityGradientWrtQ(
        withRespectTo);
}

//==============================================================================
Eigen::Vector6d SkeletonDifferential::getBodyVelocityGradientWrtQ(
    std::size_t bodyNodeIndexInSkeleton,
    const DegreeOfFreedom* withRespectTo) const
{
  // TODO(JS): emit warning if the index is not valid
  const auto* bodyNode = mComposite->getBodyNode(bodyNodeIndexInSkeleton);
  const auto* bodyNodeDifferential = bodyNode->get<BodyNodeDifferential>();

  assert(bodyNodeDifferential);

  return bodyNodeDifferential->getBodyVelocityGradientWrtQ(
        withRespectTo);
}

//==============================================================================
SkeletonDifferential::GradientMatrix
SkeletonDifferential::getBodyVelocityGradientWrtDQ(
    std::size_t bodyNodeIndexInSkeleton) const
{
  // TODO(JS): emit warning if the index is not valid
  const auto* bodyNode = mComposite->getBodyNode(bodyNodeIndexInSkeleton);
  const auto* bodyNodeDifferential = bodyNode->get<BodyNodeDifferential>();

  assert(bodyNodeDifferential);

  return bodyNodeDifferential->getBodyVelocityGradientWrtDQ();
}

//==============================================================================
Eigen::Vector6d SkeletonDifferential::getBodyVelocityGradientWrtDQ(
    std::size_t bodyNodeIndexInSkeleton,
    std::size_t withRespectTo) const
{
  // TODO(JS): emit warning if the index is not valid
  const auto* bodyNode = mComposite->getBodyNode(bodyNodeIndexInSkeleton);
  const auto* bodyNodeDifferential = bodyNode->get<BodyNodeDifferential>();

  assert(bodyNodeDifferential);

  return bodyNodeDifferential->getBodyVelocityGradientWrtDQ(
        withRespectTo);
}

//==============================================================================
Eigen::Vector6d SkeletonDifferential::getBodyVelocityGradientWrtDQ(
    std::size_t bodyNodeIndexInSkeleton,
    const DegreeOfFreedom* withRespectTo) const
{
  // TODO(JS): emit warning if the index is not valid
  const auto* bodyNode = mComposite->getBodyNode(bodyNodeIndexInSkeleton);
  const auto* bodyNodeDifferential = bodyNode->get<BodyNodeDifferential>();

  assert(bodyNodeDifferential);

  return bodyNodeDifferential->getBodyVelocityGradientWrtDQ(
        withRespectTo);
}

//==============================================================================
void SkeletonDifferential::print()
{
  updateSpatialVelocityGradients();

  for (auto* bodyNode : mComposite->getBodyNodes())
  {
    bodyNode->get<BodyNodeDifferential>()->print();
  }
}

//==============================================================================
void SkeletonDifferential::setComposite(common::Composite* newComposite)
{
  Base::setComposite(newComposite);

  auto* skel = mComposite;
  const auto numDofs = skel->getNumDofs();

  assert(skel);

  mState.mDM_GradientKineticEnergy_q.resize(numDofs);
  mState.mDM_GradientKineticEnergy_dq.resize(numDofs);
  mState.mDM_HessianKineticEnergy_q_q.resize(numDofs, numDofs);
  mState.mDM_HessianKineticEnergy_q_dq.resize(numDofs, numDofs);
  mState.mDM_HessianKineticEnergy_dq_dq.resize(numDofs, numDofs);

  mState.mDM_GradientOfLagrangian_q.resize(numDofs);
  mState.mDM_GradientOfLagrangian_dq.resize(numDofs);
  mState.mDM_HessianOfLagrangian_q_q.resize(numDofs, numDofs);
  mState.mDM_HessianOfLagrangian_q_dq.resize(numDofs, numDofs);
  mState.mDM_HessianOfLagrangian_dq_dq.resize(numDofs, numDofs);

  mState.mDM_D2LD.resize(numDofs);
  mState.mDM_D1LD.resize(numDofs);
  mState.mDM_D2D1LD.resize(numDofs, numDofs);

  for (auto* bodyNode : skel->getBodyNodes())
  {
    bodyNode->createAspect<BodyNodeDifferential>();
  }
}

//==============================================================================
void SkeletonDifferential::loseComposite(common::Composite* oldComposite)
{
  Base::loseComposite(oldComposite);
}

} // namespace dynamics
} // namespace dart
