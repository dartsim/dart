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

#include "dart/dynamics/SkeletonSecondDerivatives.hpp"

#include "dart/dynamics/BodyNode.hpp"
#include "dart/dynamics/BodyNodeDerivatives.hpp"
#include "dart/dynamics/BodyNodeSecondDerivatives.hpp"

namespace dart {
namespace dynamics {

//==============================================================================
std::unique_ptr<common::Aspect> SkeletonSecondDerivatives::cloneAspect() const
{
  // TODO(JS): not implemented
  return common::make_unique<SkeletonSecondDerivatives>();
}

//==============================================================================
Eigen::VectorXd
SkeletonSecondDerivatives::computeLagrangianGradientWrtPos() const
{
  return mSkeletonDerivatives->computeLagrangianGradientWrtPos();
}

//==============================================================================
Eigen::VectorXd
SkeletonSecondDerivatives::computeLagrangianGradientWrtVel() const
{
  return mSkeletonDerivatives->computeLagrangianGradientWrtVel();
}

//==============================================================================
Eigen::MatrixXd
SkeletonSecondDerivatives::computeLagrangianHessianWrtPosPos() const
{
  const auto numDofs = mComposite->getNumDofs();
  Eigen::MatrixXd hessian = Eigen::MatrixXd::Zero(numDofs, numDofs);

  for (const auto* bodyNode : mComposite->getBodyNodes())
  {
    auto bodyNode2ndDeriv = bodyNode->get<BodyNodeSecondDerivatives>();
    hessian += bodyNode2ndDeriv->computeLagrangianHessianWrtPosPos();
  }

  return hessian;
}

//==============================================================================
Eigen::MatrixXd
SkeletonSecondDerivatives::computeLagrangianHessianWrtPosVel() const
{
  const auto numDofs = mComposite->getNumDofs();
  Eigen::MatrixXd hessian = Eigen::MatrixXd::Zero(numDofs, numDofs);

  for (const auto* bodyNode : mComposite->getBodyNodes())
  {
    auto bodyNode2ndDeriv = bodyNode->get<BodyNodeSecondDerivatives>();
    hessian += bodyNode2ndDeriv->computeLagrangianHessianWrtPosVel();
  }

  return hessian;
}

//==============================================================================
Eigen::MatrixXd
SkeletonSecondDerivatives::computeLagrangianHessianWrtVelVel() const
{
  const auto numDofs = mComposite->getNumDofs();
  Eigen::MatrixXd hessian = Eigen::MatrixXd::Zero(numDofs, numDofs);

  for (const auto* bodyNode : mComposite->getBodyNodes())
  {
    auto bodyNode2ndDeriv = bodyNode->get<BodyNodeSecondDerivatives>();
    hessian += bodyNode2ndDeriv->computeLagrangianHessianWrtVelVel();
  }

  return hessian;
}

//==============================================================================
void SkeletonSecondDerivatives::setComposite(common::Composite* newComposite)
{
  Base::setComposite(newComposite);

  auto* skel = mComposite;
  assert(skel);

  mSkeletonDerivatives = skel->getOrCreateAspect<SkeletonDerivatives>();

  for (auto* bodyNode : skel->getBodyNodes())
    bodyNode->getOrCreateAspect<BodyNodeSecondDerivatives>();
}

} // namespace dynamics
} // namespace dart
