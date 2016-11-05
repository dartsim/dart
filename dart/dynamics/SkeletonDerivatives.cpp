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

#include "dart/dynamics/SkeletonDerivatives.hpp"

#include "dart/dynamics/BodyNode.hpp"
#include "dart/dynamics/BodyNodeDerivatives.hpp"

namespace dart {
namespace dynamics {

//==============================================================================
std::unique_ptr<common::Aspect> SkeletonDerivatives::cloneAspect() const
{
  // TODO(JS): not implemented
  return common::make_unique<SkeletonDerivatives>();
}

//==============================================================================
Eigen::VectorXd SkeletonDerivatives::computeLagrangianGradientWrtPos() const
{
  const auto numDofs = mComposite->getNumDofs();
  Eigen::VectorXd gradient = Eigen::VectorXd::Zero(numDofs);

  for (const auto* bodyNode : mComposite->getBodyNodes())
  {
    auto bodyNodeDerivative = bodyNode->get<BodyNodeDerivatives>();
    gradient += bodyNodeDerivative->computeLagrangianGradientWrtPos();
  }

  return gradient;
}

//==============================================================================
Eigen::VectorXd SkeletonDerivatives::computeLagrangianGradientWrtVel() const
{
  const auto numDofs = mComposite->getNumDofs();
  Eigen::VectorXd gradient = Eigen::VectorXd::Zero(numDofs);

  for (const auto* bodyNode : mComposite->getBodyNodes())
  {
    auto bodyNodeDerivative = bodyNode->get<BodyNodeDerivatives>();
    gradient += bodyNodeDerivative->computeLagrangianGradientWrtVel();
  }

  return gradient;
}

//==============================================================================
SkeletonDerivatives::SpatialVelocityDerivative
SkeletonDerivatives::getSpatialVelocityDerivativeWrtPos(
    std::size_t bodyNodeIndexInSkeleton) const
{
  // TODO(JS): emit warning if the index is not valid
  const auto* bodyNode = mComposite->getBodyNode(bodyNodeIndexInSkeleton);
  const auto* bodyNodeDerivative = bodyNode->get<BodyNodeDerivatives>();

  assert(bodyNodeDerivative);

  return bodyNodeDerivative->getSpatialVelocityDerivativeWrtPos();
}

//==============================================================================
Eigen::Vector6d SkeletonDerivatives::getSpatialVelocityDerivativeWrtPos(
    std::size_t bodyNodeIndexInSkeleton, std::size_t withRespectTo) const
{
  // TODO(JS): emit warning if the index is not valid
  const auto* bodyNode = mComposite->getBodyNode(bodyNodeIndexInSkeleton);
  const auto* bodyNodeDerivative = bodyNode->get<BodyNodeDerivatives>();

  assert(bodyNodeDerivative);

  return bodyNodeDerivative->getSpatialVelocityDerivativeWrtPos(withRespectTo);
}

//==============================================================================
Eigen::Vector6d SkeletonDerivatives::getSpatialVelocityDerivativeWrtPos(
    std::size_t bodyNodeIndexInSkeleton,
    const DegreeOfFreedom* withRespectTo) const
{
  // TODO(JS): emit warning if the index is not valid
  const auto* bodyNode = mComposite->getBodyNode(bodyNodeIndexInSkeleton);
  const auto* bodyNodeDerivative = bodyNode->get<BodyNodeDerivatives>();

  assert(bodyNodeDerivative);

  return bodyNodeDerivative->getSpatialVelocityDerivativeWrtPos(withRespectTo);
}

//==============================================================================
SkeletonDerivatives::SpatialVelocityDerivative
SkeletonDerivatives::getSpatialVelocityDerivativeWrtVel(
    std::size_t bodyNodeIndexInSkeleton) const
{
  // TODO(JS): emit warning if the index is not valid
  const auto* bodyNode = mComposite->getBodyNode(bodyNodeIndexInSkeleton);
  const auto* bodyNodeDerivative = bodyNode->get<BodyNodeDerivatives>();

  assert(bodyNodeDerivative);

  return bodyNodeDerivative->getSpatialVelocityDerivativeWrtVel();
}

//==============================================================================
Eigen::Vector6d SkeletonDerivatives::getSpatialVelocityDerivativeWrtVel(
    std::size_t bodyNodeIndexInSkeleton, std::size_t withRespectTo) const
{
  // TODO(JS): emit warning if the index is not valid
  const auto* bodyNode = mComposite->getBodyNode(bodyNodeIndexInSkeleton);
  const auto* bodyNodeDerivative = bodyNode->get<BodyNodeDerivatives>();

  assert(bodyNodeDerivative);

  return bodyNodeDerivative->getSpatialVelocityDerivativeWrtVel(withRespectTo);
}

//==============================================================================
Eigen::Vector6d SkeletonDerivatives::getSpatialVelocityDerivativeWrtVel(
    std::size_t bodyNodeIndexInSkeleton,
    const DegreeOfFreedom* withRespectTo) const
{
  // TODO(JS): emit warning if the index is not valid
  const auto* bodyNode = mComposite->getBodyNode(bodyNodeIndexInSkeleton);
  const auto* bodyNodeDerivative = bodyNode->get<BodyNodeDerivatives>();

  assert(bodyNodeDerivative);

  return bodyNodeDerivative->getSpatialVelocityDerivativeWrtVel(withRespectTo);
}

//==============================================================================
void SkeletonDerivatives::setComposite(common::Composite* newComposite)
{
  Base::setComposite(newComposite);

  auto* skel = mComposite;
  assert(skel);

  for (auto* bodyNode : skel->getBodyNodes())
    bodyNode->getOrCreateAspect<BodyNodeDerivatives>();
}

} // namespace dynamics
} // namespace dart
