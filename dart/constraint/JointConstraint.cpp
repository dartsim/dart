/*
 * Copyright (c) 2011-2022, The DART development contributors
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

#include "dart/constraint/JointConstraint.hpp"

#include "dart/common/Console.hpp"
#include "dart/dynamics/BodyNode.hpp"
#include "dart/dynamics/Joint.hpp"
#include "dart/dynamics/Skeleton.hpp"
#include "dart/external/odelcpsolver/lcp.h"

#include <algorithm>

#define DART_ERROR_ALLOWANCE 0.0
#define DART_ERP 0.01
#define DART_MAX_ERV 1e+1
#define DART_CFM 1e-9

namespace dart {
namespace constraint {

double JointConstraint::mErrorAllowance = DART_ERROR_ALLOWANCE;
double JointConstraint::mErrorReductionParameter = DART_ERP;
double JointConstraint::mMaxErrorReductionVelocity = DART_MAX_ERV;
double JointConstraint::mConstraintForceMixing = DART_CFM;

//==============================================================================
JointConstraint::JointConstraint(dynamics::Joint* joint)
  : ConstraintBase(),
    mJoint(joint),
    mBodyNode(joint->getChildBodyNode()),
    mAppliedImpulseIndex(0)
{
  assert(joint);
  assert(mBodyNode);
  mLifeTime.setZero();
  mActive.setConstant(false);
}

//==============================================================================
const std::string& JointConstraint::getType() const
{
  return getStaticType();
}

//==============================================================================
const std::string& JointConstraint::getStaticType()
{
  static const std::string name = "JointConstraint";
  return name;
}

//==============================================================================
void JointConstraint::setErrorAllowance(double allowance)
{
  // Clamp error reduction parameter if it is out of the range
  if (allowance < 0.0)
  {
    dtwarn << "Error reduction parameter [" << allowance
           << "] is lower than 0.0. "
           << "It is set to 0.0." << std::endl;
    mErrorAllowance = 0.0;
  }

  mErrorAllowance = allowance;
}

//==============================================================================
double JointConstraint::getErrorAllowance()
{
  return mErrorAllowance;
}

//==============================================================================
void JointConstraint::setErrorReductionParameter(double erp)
{
  // Clamp error reduction parameter if it is out of the range [0, 1]
  if (erp < 0.0)
  {
    dtwarn << "Error reduction parameter [" << erp << "] is lower than 0.0. "
           << "It is set to 0.0." << std::endl;
    mErrorReductionParameter = 0.0;
  }
  if (erp > 1.0)
  {
    dtwarn << "Error reduction parameter [" << erp << "] is greater than 1.0. "
           << "It is set to 1.0." << std::endl;
    mErrorReductionParameter = 1.0;
  }

  mErrorReductionParameter = erp;
}

//==============================================================================
double JointConstraint::getErrorReductionParameter()
{
  return mErrorReductionParameter;
}

//==============================================================================
void JointConstraint::setMaxErrorReductionVelocity(double erv)
{
  // Clamp maximum error reduction velocity if it is out of the range
  if (erv < 0.0)
  {
    dtwarn << "Maximum error reduction velocity [" << erv
           << "] is lower than 0.0. "
           << "It is set to 0.0." << std::endl;
    mMaxErrorReductionVelocity = 0.0;
  }

  mMaxErrorReductionVelocity = erv;
}

//==============================================================================
double JointConstraint::getMaxErrorReductionVelocity()
{
  return mMaxErrorReductionVelocity;
}

//==============================================================================
void JointConstraint::setConstraintForceMixing(double cfm)
{
  // Clamp constraint force mixing parameter if it is out of the range
  if (cfm < 1e-9)
  {
    dtwarn << "Constraint force mixing parameter [" << cfm
           << "] is lower than 1e-9. "
           << "It is set to 1e-9." << std::endl;
    mConstraintForceMixing = 1e-9;
  }

  mConstraintForceMixing = cfm;
}

//==============================================================================
double JointConstraint::getConstraintForceMixing()
{
  return mConstraintForceMixing;
}

//==============================================================================
void JointConstraint::update()
{
  // Reset dimension
  mDim = 0;

  const int dof = static_cast<int>(mJoint->getNumDofs());

  const Eigen::VectorXd positions = mJoint->getPositions();
  const Eigen::VectorXd velocities = mJoint->getVelocities();

  const Eigen::VectorXd positionLowerLimits = mJoint->getPositionLowerLimits();
  const Eigen::VectorXd positionUpperLimits = mJoint->getPositionUpperLimits();

  const Eigen::VectorXd velocityLowerLimits = mJoint->getVelocityLowerLimits();
  const Eigen::VectorXd velocityUpperLimits = mJoint->getVelocityUpperLimits();

  const double timeStep = mJoint->getSkeleton()->getTimeStep();
  // TODO: There are multiple ways to get time step (or its inverse).
  //   - ContactConstraint get it from the constructor parameter
  //   - Skeleton has it itself.
  //   - ConstraintBase::getInformation() passes ConstraintInfo structure
  //     that contains reciprocal time step.
  // We might need to pick one way and remove the others to get rid of
  // redundancy.

  // Constraint priority:
  //
  // 1. Position limits
  // 2. Velocity limits
  // 3. Servo motor

  mActive.setConstant(false);

  for (int i = 0; i < dof; ++i)
  {
    assert(positionLowerLimits[i] <= positionUpperLimits[i]);
    assert(velocityLowerLimits[i] <= velocityUpperLimits[i]);

    // Velocity limits due to position limits
    const double vel_to_pos_lb
        = (positionLowerLimits[i] - positions[i]) / timeStep;
    const double vel_to_pos_ub
        = (positionUpperLimits[i] - positions[i]) / timeStep;

    // Joint position and velocity constraint check
    if (mJoint->areLimitsEnforced())
    {
      const double A1 = positions[i] - positionLowerLimits[i];
      const double B1 = A1 + mErrorAllowance;
      const double A2 = positions[i] - positionUpperLimits[i];
      const double B2 = A2 - mErrorAllowance;
      if (B1 < 0)
      {
        // The current position is lower than the lower bound.
        //
        //    pos            LB                               UB
        //     |              |                               |
        //     v              v                               v
        // ----+---------+----+-------------------------------+----
        //               |--->|  : error_allowance
        //     |<-------------|  : A1
        //     |<--------|       : B1 = A1 + error_allowance
        //     |<----~~~---|     : C1 = ERP * A1

        // Set the desired velocity change to the negated current velocity plus
        // the bouncing velocity so that the joint stops and bounces to correct
        // the position error.

        const double C1 = mErrorAllowance * A1;
        double bouncing_vel = -std::max(B1, C1) / timeStep;
        assert(bouncing_vel >= 0);
        bouncing_vel = std::min(bouncing_vel, mMaxErrorReductionVelocity);

        mDesiredVelocityChange[i] = bouncing_vel - velocities[i];

        // Set the impulse bounds to not to be negative so that the impulse only
        // exerted to push the joint toward the positive direction.
        mImpulseLowerBound[i] = 0.0;
        mImpulseUpperBound[i] = static_cast<double>(dInfinity);

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
        continue;
      }
      else if (0 < B2)
      {
        // The current position is greater than the upper bound.
        //
        //    LB                               UB            pos
        //     |                               |              |
        //     v                               v              v
        // ----+-------------------------------+----+---------+----
        //                   error_allowance : |--->|
        //                                A2 : |------------->|
        //         B2 = A2 - error_allowance :      |-------->|
        //                     C2 = ERP * A2 :    |----~~~--->|

        // Set the desired velocity change to the negated current velocity plus
        // the bouncing velocity so that the joint stops and bounces to correct
        // the position error.

        const double C2 = mErrorAllowance * A2;
        double bouncing_vel = -std::min(B2, C2) / timeStep;
        assert(bouncing_vel <= 0);
        bouncing_vel = std::max(bouncing_vel, -mMaxErrorReductionVelocity);

        mDesiredVelocityChange[i] = bouncing_vel - velocities[i];

        // Set the impulse bounds to not to be positive so that the impulse only
        // exerted to push the joint toward the negative direction.
        mImpulseLowerBound[i] = -static_cast<double>(dInfinity);
        mImpulseUpperBound[i] = 0.0;

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
        continue;
      }

      // Check lower velocity bound
      const double vel_lb = std::max(velocityLowerLimits[i], vel_to_pos_lb);
      const double vel_lb_error = velocities[i] - vel_lb;
      if (vel_lb_error < 0.0)
      {
        mDesiredVelocityChange[i] = -vel_lb_error;
        mImpulseLowerBound[i] = 0.0;
        mImpulseUpperBound[i] = static_cast<double>(dInfinity);

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
        continue;
      }

      // Check upper velocity bound
      const double vel_ub = std::min(velocityUpperLimits[i], vel_to_pos_ub);
      const double vel_ub_error = velocities[i] - vel_ub;
      if (vel_ub_error > 0.0)
      {
        mDesiredVelocityChange[i] = -vel_ub_error;
        mImpulseLowerBound[i] = -static_cast<double>(dInfinity);
        mImpulseUpperBound[i] = 0.0;

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
        continue;
      }
    }

    // Servo motor constraint check
    if (mJoint->getActuatorType() == dynamics::Joint::SERVO)
    {
      // The desired velocity shouldn't be out of the velocity limits
      double desired_velocity = math::clip(
          mJoint->getCommand(static_cast<std::size_t>(i)),
          velocityLowerLimits[i],
          velocityUpperLimits[i]);

      // The next position shouldn't be out of the position limits
      desired_velocity
          = math::clip(desired_velocity, vel_to_pos_lb, vel_to_pos_ub);

      mDesiredVelocityChange[i] = desired_velocity - velocities[i];

      if (mDesiredVelocityChange[i] != 0.0)
      {
        // Note that we are computing impulse but not force
        mImpulseUpperBound[i]
            = mJoint->getForceUpperLimit(static_cast<std::size_t>(i))
              * timeStep;
        mImpulseLowerBound[i]
            = mJoint->getForceLowerLimit(static_cast<std::size_t>(i))
              * timeStep;

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
    }
  }
}

//==============================================================================
void JointConstraint::getInformation(ConstraintInfo* lcp)
{
  std::size_t index = 0;
  const int dof = static_cast<int>(mJoint->getNumDofs());
  for (int i = 0; i < dof; ++i)
  {
    if (!mActive[i])
      continue;

#ifndef NDEBUG // debug
    if (std::abs(lcp->w[index]) > 1e-6)
    {
      dterr << "Invalid " << index
            << "-th slack variable. Expected: 0.0. Actual: " << lcp->w[index]
            << ".\n";
      assert(false);
    }
#endif

    lcp->b[index] = mDesiredVelocityChange[i];

    lcp->lo[index] = mImpulseLowerBound[i];
    lcp->hi[index] = mImpulseUpperBound[i];

#ifndef NDEBUG // debug
    if (lcp->findex[index] != -1)
    {
      dterr << "Invalid " << index
            << "-th friction index. Expected: -1. Actual: "
            << lcp->findex[index] << ".\n";
      assert(false);
    }
#endif

    if (mLifeTime[i])
      lcp->x[index] = mOldX[i];
    else
      lcp->x[index] = 0.0;

    index++;
  }
}

//==============================================================================
void JointConstraint::applyUnitImpulse(std::size_t index)
{
  assert(index < mDim && "Invalid Index.");

  std::size_t localIndex = 0;
  const dynamics::SkeletonPtr& skeleton = mJoint->getSkeleton();

  std::size_t dof = mJoint->getNumDofs();
  for (std::size_t i = 0; i < dof; ++i)
  {
    if (!mActive[static_cast<int>(i)])
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
void JointConstraint::getVelocityChange(double* delVel, bool withCfm)
{
  assert(delVel != nullptr && "Null pointer is not allowed.");

  std::size_t localIndex = 0;
  std::size_t dof = mJoint->getNumDofs();
  for (std::size_t i = 0; i < dof; ++i)
  {
    if (!mActive[static_cast<int>(i)])
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
void JointConstraint::excite()
{
  mJoint->getSkeleton()->setImpulseApplied(true);
}

//==============================================================================
void JointConstraint::unexcite()
{
  mJoint->getSkeleton()->setImpulseApplied(false);
}

//==============================================================================
void JointConstraint::applyImpulse(double* lambda)
{
  std::size_t localIndex = 0;
  std::size_t dof = mJoint->getNumDofs();
  for (std::size_t i = 0; i < dof; ++i)
  {
    if (!mActive[static_cast<int>(i)])
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
dynamics::SkeletonPtr JointConstraint::getRootSkeleton() const
{
  return ConstraintBase::getRootSkeleton(mJoint->getSkeleton()->getSkeleton());
}

//==============================================================================
bool JointConstraint::isActive() const
{
  return mActive.array().any();
}

} // namespace constraint
} // namespace dart
