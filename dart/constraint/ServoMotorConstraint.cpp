/*
 * Copyright (c) 2015-2016, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2015-2016, Humanoid Lab, Georgia Tech Research Corporation
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

#include "dart/constraint/ServoMotorConstraint.hpp"

#include <iostream>

#include "dart/external/odelcpsolver/lcp.h"

#include "dart/common/Console.hpp"
#include "dart/dynamics/BodyNode.hpp"
#include "dart/dynamics/Joint.hpp"
#include "dart/dynamics/Skeleton.hpp"

#define DART_CFM     1e-9

namespace dart {
namespace constraint {

double ServoMotorConstraint::mConstraintForceMixing = DART_CFM;

//==============================================================================
ServoMotorConstraint::ServoMotorConstraint(dynamics::Joint* joint)
  : ConstraintBase(),
    mJoint(joint),
    mBodyNode(joint->getChildBodyNode()),
    mAppliedImpulseIndex(0)
{
  assert(joint);
  assert(mBodyNode);

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
ServoMotorConstraint::~ServoMotorConstraint()
{
}

//==============================================================================
void ServoMotorConstraint::setConstraintForceMixing(double cfm)
{
  // Clamp constraint force mixing parameter if it is out of the range
  if (cfm < 1e-9)
  {
    dtwarn << "[ServoMotorConstraint::setConstraintForceMixing] "
           << "Constraint force mixing parameter[" << cfm
           << "] is lower than 1e-9. " << "It is set to 1e-9." << std::endl;
    mConstraintForceMixing = 1e-9;
  }
  if (cfm > 1.0)
  {
    dtwarn << "[ServoMotorConstraint::setConstraintForceMixing] "
           << "Constraint force mixing parameter[" << cfm
           << "] is greater than 1.0. " << "It is set to 1.0." << std::endl;
    mConstraintForceMixing = 1.0;
  }

  mConstraintForceMixing = cfm;
}

//==============================================================================
double ServoMotorConstraint::getConstraintForceMixing()
{
  return mConstraintForceMixing;
}

//==============================================================================
void ServoMotorConstraint::update()
{
  // Reset dimention
  mDim = 0;

  std::size_t dof = mJoint->getNumDofs();
  for (std::size_t i = 0; i < dof; ++i)
  {
    mNegativeVelocityError[i] = mJoint->getCommand(i) - mJoint->getVelocity(i);

    if (mNegativeVelocityError[i] != 0.0)
    {
      double timeStep = mJoint->getSkeleton()->getTimeStep();
      // TODO: There are multiple ways to get time step (or its inverse).
      //   - ContactConstraint get it from the constructor parameter
      //   - Skeleton has it itself.
      //   - ConstraintBase::getInformation() passes ConstraintInfo structure
      //     that contains reciprocal time step.
      // We might need to pick one way and remove the others to get rid of
      // redundancy.

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
void ServoMotorConstraint::getInformation(ConstraintInfo* lcp)
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
void ServoMotorConstraint::applyUnitImpulse(std::size_t index)
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
void ServoMotorConstraint::getVelocityChange(double* delVel, bool withCfm)
{
  assert(delVel != nullptr && "Null pointer is not allowed.");

  std::size_t localIndex = 0;
  std::size_t dof = mJoint->getNumDofs();
  for (std::size_t i = 0; i < dof ; ++i)
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
    delVel[mAppliedImpulseIndex] += delVel[mAppliedImpulseIndex]
                                     * mConstraintForceMixing;
  }

  assert(localIndex == mDim);
}

//==============================================================================
void ServoMotorConstraint::excite()
{
  mJoint->getSkeleton()->setImpulseApplied(true);
}

//==============================================================================
void ServoMotorConstraint::unexcite()
{
  mJoint->getSkeleton()->setImpulseApplied(false);
}

//==============================================================================
void ServoMotorConstraint::applyImpulse(double* lambda)
{
  std::size_t localIndex = 0;
  std::size_t dof = mJoint->getNumDofs();
  for (std::size_t i = 0; i < dof ; ++i)
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
dynamics::SkeletonPtr ServoMotorConstraint::getRootSkeleton() const
{
  return mJoint->getSkeleton()->mUnionRootSkeleton.lock();
}

//==============================================================================
bool ServoMotorConstraint::isActive() const
{
  // Since we are not allowed to set the joint actuator type per each
  // DegreeOfFreedom, we just check if the whole joint is SERVO actuator.
  if (mJoint->getActuatorType() == dynamics::Joint::SERVO)
    return true;

  return false;
}

} // namespace constraint
} // namespace dart
