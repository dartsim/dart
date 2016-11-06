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
const Eigen::MatrixXd&
SkeletonSecondDerivatives::computeLagrangianHessianWrtPosPos() const
{
  mLagrangianHessianWrtPosPos.setZero();

  for (const auto* bodyNode : mComposite->getBodyNodes())
  {
    auto bodyNode2ndDeriv = bodyNode->get<BodyNodeSecondDerivatives>();
    mLagrangianHessianWrtPosPos
        += bodyNode2ndDeriv->computeLagrangianHessianWrtPosPos();
  }

  return mLagrangianHessianWrtPosPos;
}

//==============================================================================
const Eigen::MatrixXd&
SkeletonSecondDerivatives::computeLagrangianHessianWrtPosVel() const
{
  mLagrangianHessianWrtPosVel.setZero();

  for (const auto* bodyNode : mComposite->getBodyNodes())
  {
    auto bodyNode2ndDeriv = bodyNode->get<BodyNodeSecondDerivatives>();
    mLagrangianHessianWrtPosVel
        += bodyNode2ndDeriv->computeLagrangianHessianWrtPosVel();
  }

  return mLagrangianHessianWrtPosVel;
}

//==============================================================================
const Eigen::MatrixXd&
SkeletonSecondDerivatives::computeLagrangianHessianWrtVelVel() const
{
  mLagrangianHessianWrtVelVel.setZero();

  for (const auto* bodyNode : mComposite->getBodyNodes())
  {
    auto bodyNode2ndDeriv = bodyNode->get<BodyNodeSecondDerivatives>();
    mLagrangianHessianWrtVelVel
        += bodyNode2ndDeriv->computeLagrangianHessianWrtVelVel();
  }

  return mLagrangianHessianWrtVelVel;
}

//==============================================================================
void SkeletonSecondDerivatives::setComposite(common::Composite* newComposite)
{
  Base::setComposite(newComposite);

  auto* skel = mComposite;
  assert(skel);

  mSkeletonDerivatives = skel->getOrCreateAspect<SkeletonDerivatives>();

  const auto numDofs = mComposite->getNumDofs();

  mLagrangianHessianWrtPosPos.resize(numDofs, numDofs);
  mLagrangianHessianWrtPosVel.resize(numDofs, numDofs);
  mLagrangianHessianWrtVelVel.resize(numDofs, numDofs);

  for (auto* bodyNode : skel->getBodyNodes())
    bodyNode->getOrCreateAspect<BodyNodeSecondDerivatives>();
}

} // namespace dynamics
} // namespace dart
