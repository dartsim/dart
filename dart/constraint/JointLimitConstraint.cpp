/*
 * Copyright (c) 2011-2019, The DART development contributors
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

#include "dart/constraint/JointLimitConstraint.hpp"

#include <iostream>

#include "dart/external/odelcpsolver/lcp.h"

#include "dart/common/Console.hpp"
#include "dart/dynamics/BodyNode.hpp"
#include "dart/dynamics/Joint.hpp"
#include "dart/dynamics/Skeleton.hpp"

#define DART_ERROR_ALLOWANCE 0.0
#define DART_ERP 0.01
#define DART_MAX_ERV 1e+1
#define DART_CFM 1e-9

namespace dart {
namespace constraint {

double JointLimitConstraint::mErrorAllowance = DART_ERROR_ALLOWANCE;
double JointLimitConstraint::mErrorReductionParameter = DART_ERP;
double JointLimitConstraint::mMaxErrorReductionVelocity = DART_MAX_ERV;
double JointLimitConstraint::mConstraintForceMixing = DART_CFM;

//==============================================================================
JointLimitConstraint::JointLimitConstraint(dynamics::Joint* joint)
  : ConstraintBase(),
    mJoint(joint),
    mBodyNode(joint->getChildBodyNode()),
    mAppliedImpulseIndex(0)
{
  assert(joint);
  assert(mBodyNode);

  mLifeTime.setZero();

  mIsPositionLimitViolated.setConstant(false);
  mIsVelocityLimitViolated.setConstant(false);
}

//==============================================================================
void JointLimitConstraint::setErrorAllowance(double allowance)
{
  // Clamp error reduction parameter if it is out of the range
  if (allowance < 0.0)
  {
    dtwarn << "Error reduction parameter[" << allowance
           << "] is lower than 0.0. "
           << "It is set to 0.0." << std::endl;
    mErrorAllowance = 0.0;
  }

  mErrorAllowance = allowance;
}

//==============================================================================
double JointLimitConstraint::getErrorAllowance()
{
  return mErrorAllowance;
}

//==============================================================================
void JointLimitConstraint::setErrorReductionParameter(double erp)
{
  // Clamp error reduction parameter if it is out of the range [0, 1]
  if (erp < 0.0)
  {
    dtwarn << "Error reduction parameter[" << erp << "] is lower than 0.0. "
           << "It is set to 0.0." << std::endl;
    mErrorReductionParameter = 0.0;
  }
  if (erp > 1.0)
  {
    dtwarn << "Error reduction parameter[" << erp << "] is greater than 1.0. "
           << "It is set to 1.0." << std::endl;
    mErrorReductionParameter = 1.0;
  }

  mErrorReductionParameter = erp;
}

//==============================================================================
double JointLimitConstraint::getErrorReductionParameter()
{
  return mErrorReductionParameter;
}

//==============================================================================
void JointLimitConstraint::setMaxErrorReductionVelocity(double erv)
{
  // Clamp maximum error reduction velocity if it is out of the range
  if (erv < 0.0)
  {
    dtwarn << "Maximum error reduction velocity[" << erv
           << "] is lower than 0.0. "
           << "It is set to 0.0." << std::endl;
    mMaxErrorReductionVelocity = 0.0;
  }

  mMaxErrorReductionVelocity = erv;
}

//==============================================================================
double JointLimitConstraint::getMaxErrorReductionVelocity()
{
  return mMaxErrorReductionVelocity;
}

//==============================================================================
void JointLimitConstraint::setConstraintForceMixing(double cfm)
{
  // Clamp constraint force mixing parameter if it is out of the range
  if (cfm < 1e-9)
  {
    dtwarn << "Constraint force mixing parameter[" << cfm
           << "] is lower than 1e-9. "
           << "It is set to 1e-9." << std::endl;
    mConstraintForceMixing = 1e-9;
  }

  mConstraintForceMixing = cfm;
}

//==============================================================================
double JointLimitConstraint::getConstraintForceMixing()
{
  return mConstraintForceMixing;
}

//==============================================================================
void JointLimitConstraint::update()
{
  // Reset dimention
  mDim = 0;

  const int dof = static_cast<int>(mJoint->getNumDofs());

  const Eigen::VectorXd positions = mJoint->getPositions();
  const Eigen::VectorXd velocities = mJoint->getVelocities();

  const Eigen::VectorXd positionLowerLimits = mJoint->getPositionLowerLimits();
  const Eigen::VectorXd positionUpperLimits = mJoint->getPositionUpperLimits();

  const Eigen::VectorXd velocityLowerLimits = mJoint->getVelocityLowerLimits();
  const Eigen::VectorXd velocityUpperLimits = mJoint->getVelocityUpperLimits();

  for (int i = 0; i < dof; ++i)
  {
    // Check lower position bound
    mViolation[i] = positions[i] - positionLowerLimits[i];
    if (mViolation[i] < 0.0)
    {
      mNegativeVel[i] = -velocities[i];
      mLowerBound[i] = 0.0;
      mUpperBound[i] = static_cast<double>(dInfinity);

      if (mIsPositionLimitViolated[i])
      {
        ++(mLifeTime[i]);
      }
      else
      {
        mIsPositionLimitViolated[i] = true;
        mLifeTime[i] = 0;
      }

      ++mDim;
      continue;
    }

    // Check upper position bound
    mViolation[i] = positions[i] - positionUpperLimits[i];
    if (mViolation[i] > 0.0)
    {
      mNegativeVel[i] = -velocities[i];
      mLowerBound[i] = -static_cast<double>(dInfinity);
      mUpperBound[i] = 0.0;

      if (mIsPositionLimitViolated[i])
      {
        ++(mLifeTime[i]);
      }
      else
      {
        mIsPositionLimitViolated[i] = true;
        mLifeTime[i] = 0;
      }

      ++mDim;
      continue;
    }

    mIsPositionLimitViolated[i] = false;

    // Check lower velocity bound
    mViolation[i] = velocities[i] - velocityLowerLimits[i];
    if (mViolation[i] < 0.0)
    {
      mNegativeVel[i] = -mViolation[i];
      mLowerBound[i] = 0.0;
      mUpperBound[i] = static_cast<double>(dInfinity);

      if (mIsVelocityLimitViolated[i])
      {
        ++(mLifeTime[i]);
      }
      else
      {
        mIsVelocityLimitViolated[i] = true;
        mLifeTime[i] = 0;
      }

      ++mDim;
      continue;
    }

    // Check upper velocity bound
    mViolation[i] = velocities[i] - velocityUpperLimits[i];
    if (mViolation[i] > 0.0)
    {
      mNegativeVel[i] = -mViolation[i];
      mLowerBound[i] = -static_cast<double>(dInfinity);
      mUpperBound[i] = 0.0;

      if (mIsVelocityLimitViolated[i])
      {
        ++(mLifeTime[i]);
      }
      else
      {
        mIsVelocityLimitViolated[i] = true;
        mLifeTime[i] = 0;
      }

      ++mDim;
      continue;
    }

    mIsVelocityLimitViolated[i] = false;
  }
}

//==============================================================================
void JointLimitConstraint::getInformation(ConstraintInfo* lcp)
{
  std::size_t index = 0;
  const int dof = static_cast<int>(mJoint->getNumDofs());
  for (int i = 0; i < dof; ++i)
  {
    if (mIsPositionLimitViolated[i])
    {
      assert(lcp->w[index] == 0.0);

      double bouncingVel = -mViolation[i];

      if (bouncingVel > 0.0)
        bouncingVel = -mErrorAllowance;
      else
        bouncingVel = +mErrorAllowance;

      bouncingVel *= lcp->invTimeStep * mErrorReductionParameter;

      if (bouncingVel > mMaxErrorReductionVelocity)
        bouncingVel = mMaxErrorReductionVelocity;

      lcp->b[index] = mNegativeVel[i] + bouncingVel;

      lcp->lo[index] = mLowerBound[i];
      lcp->hi[index] = mUpperBound[i];

#ifndef NDEBUG // Debug mode
      if (lcp->lo[index] > lcp->hi[index])
      {
        std::cout << "dim: " << mDim << std::endl;
        std::cout << "lb: " << mLowerBound[i] << std::endl;
        std::cout << "ub: " << mUpperBound[i] << std::endl;
        std::cout << "lb: " << lcp->lo[index] << std::endl;
        std::cout << "ub: " << lcp->hi[index] << std::endl;
      }
#endif

      assert(lcp->findex[index] == -1);

      if (mLifeTime[i])
        lcp->x[index] = mOldX[i];
      else
        lcp->x[index] = 0.0;

      index++;
    }

    if (mIsVelocityLimitViolated[i])
    {
      assert(lcp->w[index] == 0.0);

      lcp->b[index] = mNegativeVel[i];
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
}

//==============================================================================
void JointLimitConstraint::applyUnitImpulse(std::size_t index)
{
  assert(_index < mDim && "Invalid Index.");

  std::size_t localIndex = 0;
  const dynamics::SkeletonPtr& skeleton = mJoint->getSkeleton();

  std::size_t dof = mJoint->getNumDofs();
  for (std::size_t i = 0; i < dof; ++i)
  {
    if (not mIsPositionLimitViolated[static_cast<int>(i)]
        && not mIsVelocityLimitViolated[static_cast<int>(i)])
    {
      continue;
    }

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
void JointLimitConstraint::getVelocityChange(double* delVel, bool withCfm)
{
  assert(_delVel != nullptr && "Null pointer is not allowed.");

  std::size_t localIndex = 0;
  std::size_t dof = mJoint->getNumDofs();
  for (std::size_t i = 0; i < dof; ++i)
  {
    if (not mIsPositionLimitViolated[static_cast<int>(i)]
        && not mIsVelocityLimitViolated[static_cast<int>(i)])
    {
      continue;
    }

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
void JointLimitConstraint::excite()
{
  mJoint->getSkeleton()->setImpulseApplied(true);
}

//==============================================================================
void JointLimitConstraint::unexcite()
{
  mJoint->getSkeleton()->setImpulseApplied(false);
}

//==============================================================================
void JointLimitConstraint::applyImpulse(double* lambda)
{
  std::size_t localIndex = 0;
  std::size_t dof = mJoint->getNumDofs();
  for (std::size_t i = 0; i < dof; ++i)
  {
    if (not mIsPositionLimitViolated[static_cast<int>(i)]
        && not mIsVelocityLimitViolated[static_cast<int>(i)])
    {
      continue;
    }

    mJoint->setConstraintImpulse(
        i, mJoint->getConstraintImpulse(i) + lambda[localIndex]);

    mOldX[static_cast<int>(i)] = lambda[localIndex];

    ++localIndex;
  }
}

//==============================================================================
dynamics::SkeletonPtr JointLimitConstraint::getRootSkeleton() const
{
  return mJoint->getSkeleton()->mUnionRootSkeleton.lock();
}

//==============================================================================
bool JointLimitConstraint::isActive() const
{
  return mIsPositionLimitViolated.array().any()
         || mIsVelocityLimitViolated.array().any();
}

} // namespace constraint
} // namespace dart
