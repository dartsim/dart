/*
 * Copyright (c) 2011-2018, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/master/LICENSE
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

#include "dart/constraint/ConstraintBase.hpp"

#include <cmath>
#include <cstring>
#include <iomanip>
#include <iostream>

#include "dart/dynamics/PointMass.hpp"
#include "dart/dynamics/Skeleton.hpp"
#include "dart/dynamics/SoftBodyNode.hpp"

namespace dart {
namespace constraint {

//==============================================================================
ConstraintBase::ConstraintBase() : mDim(0)
{
}

//==============================================================================
ConstraintBase::~ConstraintBase()
{
}

//==============================================================================
std::size_t ConstraintBase::getDimension() const
{
  return mDim;
}

//==============================================================================
void ConstraintBase::uniteSkeletons()
{
  // Do nothing
}

//==============================================================================
void ConstraintBase::computeJointImpulseResponses(
    Eigen::Ref<Eigen::VectorXd> responses,
    dynamics::BodyNode& bodyNode,
    const Eigen::Vector6d& bodyImpulse)
{
  bodyNode.applyConstraintImpulse(bodyImpulse);
  bodyNode.getSkeleton()->computeImpulseForwardDynamics(bodyNode);

  Eigen::VectorXd tmp1 = bodyNode.getSkeleton()->getImpulseResponses();
  responses = bodyNode.getSkeleton()->getImpulseResponses();
  bodyNode.clearConstraintImpulse();

  Eigen::VectorXd tmp2 = bodyNode.getSkeleton()->getImpulseResponses();
}

//==============================================================================
void ConstraintBase::computeJointImpulseResponses(
    Eigen::VectorXd& responses,
    dynamics::BodyNode& bodyNodeA,
    const Eigen::Vector6d& impA,
    dynamics::BodyNode& bodyNodeB,
    const Eigen::Vector6d& impB)
{
  bodyNodeA.applyConstraintImpulse(impA);
  bodyNodeB.applyConstraintImpulse(impB);

  assert(bodyNodeA.getSkeleton());
  assert(bodyNodeB.getSkeleton());
  assert(bodyNodeA.getSkeleton() == bodyNodeB.getSkeleton());
  dynamics::Skeleton& skeleton = *bodyNodeA.getSkeleton();

  // Find which body is placed later in the list of body nodes in this skeleton
  const std::size_t indexA = bodyNodeA.getIndexInSkeleton();
  const std::size_t indexB = bodyNodeB.getIndexInSkeleton();
  if (indexA > indexB)
    skeleton.updateBiasImpulse(bodyNodeA);
  else
    skeleton.updateBiasImpulse(bodyNodeB);

  bodyNodeA.clearConstraintImpulse();
  bodyNodeB.clearConstraintImpulse();

  responses = skeleton.getImpulseResponses();
}

//==============================================================================
void ConstraintBase::computeJointImpulseResponses(
    Eigen::VectorXd& responses,
    dynamics::SoftBodyNode& softBodyNode,
    dynamics::PointMass& pointMass,
    const Eigen::Vector3d& imp)
{
  pointMass.setConstraintImpulse(imp, true);

  // Prepare cache data
  assert(softBodyNode.getSkeleton());
  softBodyNode.getSkeleton()->updateBiasImpulse(softBodyNode);
  softBodyNode.clearConstraintImpulse();

  responses = softBodyNode.getSkeleton()->getImpulseResponses();
}

//==============================================================================
dynamics::SkeletonPtr ConstraintBase::compressPath(
    dynamics::SkeletonPtr _skeleton)
{
  while (_skeleton->mUnionRootSkeleton.lock() != _skeleton)
  {
    _skeleton->mUnionRootSkeleton
        = _skeleton->mUnionRootSkeleton.lock()->mUnionRootSkeleton.lock();
    _skeleton = _skeleton->mUnionRootSkeleton.lock();
  }

  return _skeleton;
}

//==============================================================================
dynamics::SkeletonPtr ConstraintBase::getRootSkeleton(
    dynamics::SkeletonPtr _skeleton)
{
  while (_skeleton->mUnionRootSkeleton.lock() != _skeleton)
    _skeleton = _skeleton->mUnionRootSkeleton.lock();

  return _skeleton;
}

} // namespace constraint
} // namespace dart
