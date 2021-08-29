/*
 * Copyright (c) 2011-2021, The DART development contributors
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

#include "dart/constraint/MimicMotorConstraint.hpp"

#include <iostream>

#include "dart/common/Console.hpp"
#include "dart/dynamics/BodyNode.hpp"
#include "dart/dynamics/Joint.hpp"
#include "dart/dynamics/Skeleton.hpp"
#include "dart/external/odelcpsolver/lcp.h"

#define DART_CFM 1e-9

namespace dart {
namespace constraint {

double MimicMotorConstraint::mConstraintForceMixing = DART_CFM;

//==============================================================================
MimicMotorConstraint::MimicMotorConstraint(
    dynamics::Joint* joint,
    const dynamics::Joint* mimicJoint,
    double multiplier,
    double offset)
  : ConstraintBase(),
    mJoint(joint),
    mMimicJoint(mimicJoint),
    mMultiplier(multiplier),
    mOffset(offset),
    mBodyNode(joint->getChildBodyNode()),
    mAppliedImpulseIndex(0)
{
  assert(joint);
  assert(mimicJoint);
  assert(mBodyNode);
  assert(joint->getNumDofs() == mimicJoint->getNumDofs());

  mLifeTime[0] = 0;
  mLifeTime[1] = 0;
  mLifeTime[2] = 0;
  mLifeTime[3] = 0;
  mLifeTime[4] = 0;
  mLifeTime[5] = 0;

  mActive[0] = false;
  mActive[1] = false;
  mActive[2] = false;
  mActive[3] = false;
  mActive[4] = false;
  mActive[5] = false;
}

//==============================================================================
MimicMotorConstraint::~MimicMotorConstraint()
{
  // Do nothing
}

//==============================================================================
const std::string& MimicMotorConstraint::getType() const
{
  return getStaticType();
}

//==============================================================================
const std::string& MimicMotorConstraint::getStaticType()
{
  static const std::string name = "MimicMotorConstraint";
  return name;
}

//==============================================================================
void MimicMotorConstraint::setConstraintForceMixing(double cfm)
{
  // Clamp constraint force mixing parameter if it is out of the range
  if (cfm < 1e-9)
  {
    dtwarn << "[MimicMotorConstraint::setConstraintForceMixing] "
           << "Constraint force mixing parameter[" << cfm
           << "] is lower than 1e-9. "
           << "It is set to 1e-9.\n";
    mConstraintForceMixing = 1e-9;
  }

  mConstraintForceMixing = cfm;
}

//==============================================================================
double MimicMotorConstraint::getConstraintForceMixing()
{
  return mConstraintForceMixing;
}

//==============================================================================
void MimicMotorConstraint::update()
{
  // Reset dimention
  mDim = 0;

  std::size_t dof = mJoint->getNumDofs();
  for (std::size_t i = 0; i < dof; ++i)
  {
    double timeStep = mJoint->getSkeleton()->getTimeStep();
    double qError = mMimicJoint->getPosition(i) * mMultiplier + mOffset
                    - mJoint->getPosition(i);
    double desiredVelocity = math::clip(
        qError / timeStep,
        mJoint->getVelocityLowerLimit(i),
        mJoint->getVelocityUpperLimit(i));

    mNegativeVelocityError[i] = desiredVelocity - mJoint->getVelocity(i);

    if (mNegativeVelocityError[i] != 0.0)
    {
      // Note that we are computing impulse not force
      mUpperBound[i] = mJoint->getForceUpperLimit(i) * timeStep;
      mLowerBound[i] = mJoint->getForceLowerLimit(i) * timeStep;

      if (mActive[i])
      {
        ++(mLifeTime[i]);
      }
      else
      {
        mActive[i] = true;
        mLifeTime[i] = 0;
      }

      ++mDim;
    }
    else
    {
      mActive[i] = false;
    }
  }
}

//==============================================================================
void MimicMotorConstraint::getInformation(ConstraintInfo* lcp)
{
  std::size_t index = 0;
  std::size_t dof = mJoint->getNumDofs();
  for (std::size_t i = 0; i < dof; ++i)
  {
    if (mActive[i] == false)
      continue;

    assert(lcp->w[index] == 0.0);

    lcp->b[index] = mNegativeVelocityError[i];
    lcp->lo[index] = mLowerBound[i];
    lcp->hi[index] = mUpperBound[i];

    assert(lcp->findex[index] == -1);

    if (mLifeTime[i])
      lcp->x[index] = mOldX[i];
    else
      lcp->x[index] = 0.0;

    index++;
  }
}

//==============================================================================
void MimicMotorConstraint::applyUnitImpulse(std::size_t index)
{
  assert(index < mDim && "Invalid Index.");

  std::size_t localIndex = 0;
  const dynamics::SkeletonPtr& skeleton = mJoint->getSkeleton();

  std::size_t dof = mJoint->getNumDofs();
  for (std::size_t i = 0; i < dof; ++i)
  {
    if (mActive[i] == false)
      continue;

    if (localIndex == index)
    {
      skeleton->clearConstraintImpulses();
      mJoint->setConstraintImpulse(i, 1.0);
      skeleton->updateBiasImpulse(mBodyNode);
      skeleton->updateVelocityChange();
      mJoint->setConstraintImpulse(i, 0.0);
    }

    ++localIndex;
  }

  mAppliedImpulseIndex = index;
}

//==============================================================================
void MimicMotorConstraint::getVelocityChange(double* delVel, bool withCfm)
{
  assert(delVel != nullptr && "Null pointer is not allowed.");

  std::size_t localIndex = 0;
  std::size_t dof = mJoint->getNumDofs();
  for (std::size_t i = 0; i < dof; ++i)
  {
    if (mActive[i] == false)
      continue;

    if (mJoint->getSkeleton()->isImpulseApplied())
      delVel[localIndex] = mJoint->getVelocityChange(i);
    else
      delVel[localIndex] = 0.0;

    ++localIndex;
  }

  // Add small values to diagnal to keep it away from singular, similar to cfm
  // varaible in ODE
  if (withCfm)
  {
    delVel[mAppliedImpulseIndex]
        += delVel[mAppliedImpulseIndex] * mConstraintForceMixing;
  }

  assert(localIndex == mDim);
}

//==============================================================================
void MimicMotorConstraint::excite()
{
  mJoint->getSkeleton()->setImpulseApplied(true);
}

//==============================================================================
void MimicMotorConstraint::unexcite()
{
  mJoint->getSkeleton()->setImpulseApplied(false);
}

//==============================================================================
void MimicMotorConstraint::applyImpulse(double* lambda)
{
  std::size_t localIndex = 0;
  std::size_t dof = mJoint->getNumDofs();
  for (std::size_t i = 0; i < dof; ++i)
  {
    if (mActive[i] == false)
      continue;

    mJoint->setConstraintImpulse(
        i, mJoint->getConstraintImpulse(i) + lambda[localIndex]);
    // TODO(JS): consider to add Joint::addConstraintImpulse()

    mOldX[i] = lambda[localIndex];

    ++localIndex;
  }
}

//==============================================================================
dynamics::SkeletonPtr MimicMotorConstraint::getRootSkeleton() const
{
  return mJoint->getSkeleton()->mUnionRootSkeleton.lock();
}

//==============================================================================
bool MimicMotorConstraint::isActive() const
{
  // Since we are not allowed to set the joint actuator type per each
  // DegreeOfFreedom, we just check if the whole joint is SERVO actuator.
  if (mJoint->getActuatorType() == dynamics::Joint::MIMIC)
    return true;

  return false;
}

} // namespace constraint
} // namespace dart
