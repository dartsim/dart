/*
 * Copyright (c) 2016, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>
 *
 * Georgia Tech Graphics Lab and Humanoid Robotics Lab
 *
 * Directed by Prof. C. Karen Liu and Prof. Mike Stilman
 * <karenliu@cc.gatech.edu> <mstilman@cc.gatech.edu>
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

#include "dart/dynamics/RecursiveDynamicsAlgorithms.h"

#include "dart/dynamics/BodyNode.h"

namespace dart {
namespace dynamics {

//==============================================================================
HybridDynamicsForBodyNode::HybridDynamicsForBodyNode(
    common::Composite* composite,
    const StateData& state)
  : HybridDynamicsForBodyNode::Base(composite, state)
{
  // Do nothing
}

//==============================================================================
void HybridDynamicsForBodyNode::updateBiasForce(
    const Eigen::Vector3d& gravity, double timeStep)
{
  auto& bn = mComposite;
  const auto& bnState = mComposite->getAspectState();
  const auto& bnProperties = mComposite->getAspectProperties();

  // Gravity force
  const Eigen::Matrix6d& mI = bnProperties.mInertia.getSpatialTensor();
  if (bnProperties.mGravityMode)
  {
    mState.mSpatialGravityForce.noalias()
        = mI * math::AdInvRLinear(bn->getWorldTransform(), gravity);
  }
  else
  {
    mState.mSpatialGravityForce.setZero();
  }

  // Set bias force
  const Eigen::Vector6d& V = bn->getSpatialVelocity();
  mState.mBiasForce = -math::dad(V, mI * V);
  mState.mBiasForce -= bnState.mFext;
  mState.mBiasForce -= mState.mSpatialGravityForce;

  // Verification
  assert(!math::isNan(mState.mBiasForce));

  // And add child bias force
  for (const auto& childBodyNode : bn->mChildBodyNodes)
  {
    Joint* childJoint = childBodyNode->getParentJoint();

    childJoint->addChildBiasForceTo(
          mState.mBiasForce,
          childBodyNode->getArticulatedInertiaImplicit(),
          childBodyNode->getHybridDynamicsForBodyNode()->mState.mBiasForce,
          childBodyNode->getPartialAcceleration());
  }

  // Verification
  assert(!math::isNan(mState.mBiasForce));

  // Update parent joint's total force with implicit joint damping and spring
  // forces
  bn->mParentJoint->updateTotalForce(
        bn->getArticulatedInertiaImplicit() * bn->getPartialAcceleration()
        + mState.mBiasForce,
        timeStep);
}

//==============================================================================
void HybridDynamicsForBodyNode::updateAccelerationFD()
{
  auto& bn = mComposite;

  if (bn->mParentBodyNode)
  {
    // Update joint acceleration
    bn->mParentJoint->updateAcceleration(
          bn->getArticulatedInertiaImplicit(),
          bn->mParentBodyNode->getSpatialAcceleration());
  }
  else
  {
    // Update joint acceleration
    bn->mParentJoint->updateAcceleration(
          bn->getArticulatedInertiaImplicit(),
          Eigen::Vector6d::Zero());
  }

  // Verify the spatial acceleration of this body
  assert(!math::isNan(bn->mAcceleration));
}

//==============================================================================
void HybridDynamicsForSoftBodyNode::updateBiasForce(
    const Eigen::Vector3d& /*gravity*/, double /*timeStep*/)
{
//  auto& bn = mComposite;
//  const auto& bnState = mComposite->getAspectState();
//  const auto& bnProperties = mComposite->getAspectProperties();

//  // Gravity force
//  const Eigen::Matrix6d& mI = bnProperties.mInertia.getSpatialTensor();
//  if (bnProperties.mGravityMode)
//  {
//    mState.mSpatialGravityForce.noalias()
//        = mI * math::AdInvRLinear(bn->getWorldTransform(), gravity);
//  }
//  else
//  {
//    mState.mSpatialGravityForce.setZero();
//  }

//  // Set bias force
//  const Eigen::Vector6d& V = bn->getSpatialVelocity();
//  mState.mBiasForce = -math::dad(V, mI * V);
//  mState.mBiasForce -= bnState.mFext;
//  mState.mBiasForce -= mState.mSpatialGravityForce;

//  // Verification
//  assert(!math::isNan(mState.mBiasForce));

//  // And add child bias force
//  for (const auto& childBodyNode : bn->mChildBodyNodes)
//  {
//    Joint* childJoint = childBodyNode->getParentJoint();

//    childJoint->addChildBiasForceTo(
//          mState.mBiasForce,
//          childBodyNode->getArticulatedInertiaImplicit(),
//          childBodyNode->getHybridDynamicsForBodyNode()->getBiasForce(),
//          childBodyNode->getPartialAcceleration());
//  }

//  for (const auto& pointMass : bn->mPointMasses)
//  {
//    mState.mBiasForce.head<3>() += pointMass->getLocalPosition().cross(pointMass->mBeta);
//    mState.mBiasForce.tail<3>() += pointMass->mBeta;
//  }

//  // Verification
//  assert(!math::isNan(mState.mBiasForce));

//  // Update parent joint's total force with implicit joint damping and spring
//  // forces
//  bn->mParentJoint->updateTotalForce(
//        bn->getArticulatedInertiaImplicit() * bn->getPartialAcceleration()
//        + mState.mBiasForce,
//        timeStep);
}

} // namespace dynamics
} // namespace dart
