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

#include "dart/experimental/BodyNodeViNewtonDrnea.hpp"

#include "dart/dynamics/BodyNode.hpp"
#include "dart/dynamics/DegreeOfFreedom.hpp"
#include "dart/dynamics/RevoluteJoint.hpp"
#include "dart/experimental/JointViNewtonDrnea.hpp"

namespace dart {
namespace dynamics {

namespace detail {

//==============================================================================
BodyNodeViNewtonDrneaState::BodyNodeViNewtonDrneaState()
{
  // Do nothing
}

} // namespace detail

//==============================================================================
BodyNodeViNewtonDrnea::BodyNodeViNewtonDrnea(const StateData& state)
{
  mState = state;
}

//==============================================================================
JointViNewtonDrnea* BodyNodeViNewtonDrnea::getJointVi()
{
  auto* bodyNode = mComposite;
  auto* joint = bodyNode->getParentJoint();
  assert(joint->get<JointViNewtonDrnea>());

  return joint->get<JointViNewtonDrnea>();
}

//==============================================================================
const JointViNewtonDrnea* BodyNodeViNewtonDrnea::getJointVi() const
{
  return const_cast<const BodyNodeViNewtonDrnea*>(this)->getJointVi();
}

//==============================================================================
void BodyNodeViNewtonDrnea::initialize(double timeStep)
{
  auto* bodyNode = mComposite;

  const Eigen::Matrix6d& G = bodyNode->getInertia().getSpatialTensor();
  const Eigen::Vector6d& V = bodyNode->getSpatialVelocity();

  mState.mPrevMomentum = math::dexp_inv_transpose(V * timeStep, G * V);
}

//==============================================================================
void BodyNodeViNewtonDrnea::setComposite(common::Composite* newComposite)
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
    JointViNewtonDrnea* revVI = new RevoluteJointViNewtonDrnea();
    joint->set(revVI);
    assert(joint->get<JointViNewtonDrnea>());
  }
  else
  {
    dterr << "[BodyNodeViNewtonDrnea::setComposite] Attempting to "
          << "create VI aspect for unsupported joint type '" << joint->getType()
          << "'.\n";
    assert(false);
  }
}

//==============================================================================
void BodyNodeViNewtonDrnea::updateNextTransform()
{
  auto* bodyNode = mComposite;
  auto* joint = bodyNode->getParentJoint();
  auto* jointAspect = joint->get<JointViNewtonDrnea>();

  jointAspect->updateNextRelativeTransform();
}

//==============================================================================
void BodyNodeViNewtonDrnea::updateNextVelocity(double timeStep)
{
  auto* bodyNode = mComposite;
  auto* parentBodyNode = bodyNode->getParentBodyNode();
  auto* joint = bodyNode->getParentJoint();
  auto* jointAspect = joint->get<JointViNewtonDrnea>();

  if (parentBodyNode)
  {
    auto parentBodyNodeVi = parentBodyNode->get<BodyNodeViNewtonDrnea>();
    assert(parentBodyNodeVi);

    mState.mWorldTransformDisplacement
        = joint->getRelativeTransform().inverse()
          * parentBodyNodeVi->mState.mWorldTransformDisplacement
          * jointAspect->mNextRelativeTransform;
  }
  else
  {
    mState.mWorldTransformDisplacement
        = joint->getRelativeTransform().inverse() * jointAspect->mNextRelativeTransform;
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
void BodyNodeViNewtonDrnea::evaluateDel(
    const Eigen::Vector3d& gravity, double timeStep)
{
  auto* bodyNode = mComposite;
  auto* joint = bodyNode->getParentJoint();
  auto* jointAspect = joint->get<JointViNewtonDrnea>();

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
    auto* childBodyNodeVi = childBodyNode->get<BodyNodeViNewtonDrnea>();

    mState.mParentImpulse += math::dAdInvT(
        childBodyNode->getParentJoint()->getRelativeTransform(),
        childBodyNodeVi->mState.mParentImpulse);
  }

  jointAspect->evaluateDel(mState.mParentImpulse, timeStep);
}

//==============================================================================
void BodyNodeViNewtonDrnea::updateNextTransformDeriv()
{
}

//==============================================================================
void BodyNodeViNewtonDrnea::updateNextVelocityDeriv(double /*timeStep*/)
{
}

} // namespace dynamics
} // namespace dart
