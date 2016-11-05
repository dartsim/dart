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

#include "dart/experimental/BodyNodeViRiqnSvi.hpp"

#include "dart/dynamics/BodyNode.hpp"
#include "dart/dynamics/DegreeOfFreedom.hpp"
#include "dart/dynamics/RevoluteJoint.hpp"
#include "dart/experimental/JointViRiqnSvi.hpp"

namespace dart {
namespace dynamics {

namespace detail {

//==============================================================================
BodyNodeViRiqnSviState::BodyNodeViRiqnSviState()
{
  // Do nothing
}

} // namespace detail

//==============================================================================
BodyNodeViRiqnSvi::BodyNodeViRiqnSvi(const StateData& state)
{
  mState = state;
}

//==============================================================================
JointViRiqnSvi* BodyNodeViRiqnSvi::getJointVi()
{
  auto* bodyNode = mComposite;
  auto* joint = bodyNode->getParentJoint();
  assert(joint->get<JointViRiqnSvi>());

  return joint->get<JointViRiqnSvi>();
}

//==============================================================================
const JointViRiqnSvi* BodyNodeViRiqnSvi::getJointVi() const
{
  return const_cast<const BodyNodeViRiqnSvi*>(this)->getJointVi();
}

//==============================================================================
void BodyNodeViRiqnSvi::initialize(double timeStep)
{
  auto* bodyNode = mComposite;

  const Eigen::Matrix6d& G = bodyNode->getInertia().getSpatialTensor();
  const Eigen::Vector6d& V = bodyNode->getSpatialVelocity();

  mState.mPrevMomentum = math::dexp_inv_transpose(V * timeStep, G * V);

  auto* joint = bodyNode->getParentJoint();
  assert(joint->get<JointViRiqnSvi>());
  joint->get<JointViRiqnSvi>()->initialize(timeStep);
}

//==============================================================================
void BodyNodeViRiqnSvi::setComposite(common::Composite* newComposite)
{
  Base::setComposite(newComposite);

  auto* bodyNode = mComposite;
  auto skeleton = bodyNode->getSkeleton();
  //  const auto numDofs = skeleton->getNumDofs();
  //  const auto timeStep = skeleton->getTimeStep();

  assert(skeleton);

  // TODO(JS): These should be updated when the structure of skeleton is
  // modified. We might want to consider adding signal to skeleton for this.

  auto* joint = bodyNode->getParentJoint();
  if (joint->is<RevoluteJoint>())
  {
    JointViRiqnSvi* revVI = new RevoluteJointViRiqnSvi();
    joint->set(revVI);
    assert(joint->get<JointViRiqnSvi>());
  }
  else
  {
    dterr << "[BodyNodeViRiqnSvi::setComposite] Attempting to "
          << "create VI aspect for unsupported joint type '" << joint->getType()
          << "'.\n";
    assert(false);
  }
}

//==============================================================================
void BodyNodeViRiqnSvi::updateNextTransform()
{
  auto* bodyNode = mComposite;
  auto* joint = bodyNode->getParentJoint();
  auto* jointAspect = joint->get<JointViRiqnSvi>();

  jointAspect->updateNextRelativeTransform();
}

//==============================================================================
void BodyNodeViRiqnSvi::updateNextVelocity(double timeStep)
{
  auto* bodyNode = mComposite;
  auto* parentBodyNode = bodyNode->getParentBodyNode();
  auto* joint = bodyNode->getParentJoint();
  auto* jointAspect = joint->get<JointViRiqnSvi>();

  if (parentBodyNode)
  {
    auto parentBodyNodeVi = parentBodyNode->get<BodyNodeViRiqnSvi>();
    assert(parentBodyNodeVi);

    mState.mWorldTransformDisplacement
        = joint->getRelativeTransform().inverse()
          * parentBodyNodeVi->mState.mWorldTransformDisplacement
          * jointAspect->mNextTransform;
  }
  else
  {
    mState.mWorldTransformDisplacement
        = joint->getRelativeTransform().inverse() * jointAspect->mNextTransform;
  }

  mState.mPostAverageSpatialVelocity
      = math::logMap(mState.mWorldTransformDisplacement) / timeStep;
}

//==============================================================================
static Eigen::Vector6d computeSpatialGravityForce(
    const Eigen::Isometry3d& T,
    const Eigen::Matrix6d& inertiaTensor,
    const Eigen::Vector3d& gravityAcceleration)
{
  const Eigen::Matrix6d& I = inertiaTensor;
  const Eigen::Vector3d& g = gravityAcceleration;

  return I * math::AdInvRLinear(T, g);
}

//==============================================================================
void BodyNodeViRiqnSvi::evaluateDel(
    const Eigen::Vector3d& gravity, double timeStep)
{
  auto* bodyNode = mComposite;
  auto* joint = bodyNode->getParentJoint();
  auto* jointAspect = joint->get<JointViRiqnSvi>();

  const Eigen::Matrix6d& G = bodyNode->getInertia().getSpatialTensor();

  mState.mPostMomentum = math::dexp_inv_transpose(
      mState.mPostAverageSpatialVelocity * timeStep,
      G * mState.mPostAverageSpatialVelocity);

  const Eigen::Isometry3d expHPrevVelocity
      = math::expMap(timeStep * mState.mPreAverageSpatialVelocity);

  mState.mParentImpulse
      = mState.mPostMomentum
        - math::dAdT(expHPrevVelocity, mState.mPrevMomentum)
        - computeSpatialGravityForce(bodyNode->getTransform(), G, gravity)
              * timeStep;
  // TODO(JS): subtract external force * timeStep to parentImpulse

  for (auto i = 0u; i < bodyNode->getNumChildBodyNodes(); ++i)
  {
    auto* childBodyNode = bodyNode->getChildBodyNode(i);
    auto* childBodyNodeVi = childBodyNode->get<BodyNodeViRiqnSvi>();

    mState.mParentImpulse += math::dAdInvT(
        childBodyNode->getParentJoint()->getRelativeTransform(),
        childBodyNodeVi->mState.mParentImpulse);
  }

  jointAspect->evaluateDel(mState.mParentImpulse, timeStep);
}

//==============================================================================
void BodyNodeViRiqnSvi::updateNextTransformDeriv()
{
}

//==============================================================================
void BodyNodeViRiqnSvi::updateNextVelocityDeriv(double /*timeStep*/)
{
}

} // namespace dynamics
} // namespace dart
