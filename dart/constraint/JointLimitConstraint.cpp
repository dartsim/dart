/*
 * Copyright (c) 2014, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Karen Liu <karenliu@cc.gatech.edu>,
 *            Jeongseok Lee <jslee02@gmail.com>
 *
 * Geoorgia Tech Graphics Lab and Humanoid Robotics Lab
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

#include "dart/constraint/JointLimitConstraint.h"

#include <iostream>

#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/Joint.h"
#include "dart/dynamics/Skeleton.h"
#include "dart/lcpsolver/lcp.h"

#define DART_DEFAULT_JOINT_POSITION_ERROR_ALLOWANCE 0.0
#define DART_DEFAUT_JOINT_POSITION_ERP 1.0

namespace dart {
namespace constraint {

//==============================================================================
JointLimitConstraint::JointLimitConstraint(dynamics::Joint* _joint)
  : Constraint(CT_DYNAMIC), mJoint(_joint), mBodyNode(NULL)
{
  assert(_joint);

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

  // TODO(JS): We need to be able to access child body node from joint
  for (int i = 0; i < mJoint->getSkeleton()->getNumBodyNodes(); ++i)
  {
    dynamics::BodyNode* bodyNode = mJoint->getSkeleton()->getBodyNode(i);
    if (bodyNode->getParentJoint() == mJoint)
      mBodyNode = bodyNode;
  }

  assert(mBodyNode);
}

//==============================================================================
JointLimitConstraint::~JointLimitConstraint()
{

}

//==============================================================================
void JointLimitConstraint::update()
{
  // Reset dimention
  mDim = 0;

  dynamics::GenCoord* genCoord;

  size_t dof = mJoint->getNumGenCoords();
  for (size_t i = 0; i < dof ; ++i)
  {
    genCoord = mJoint->getGenCoord(i);

    // Lower bound check
    mViolation[i] = genCoord->getPos() - genCoord->getPosMin();
    if (mViolation[i] <= 0.0)
    {
      mNegativeVel[i] = -genCoord->getVel();

      mLowerBound[i] = 0.0;
      mUpperBound[i] = dInfinity;

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

    // Upper bound check
    mViolation[i] = genCoord->getPos() - genCoord->getPosMax();
    if (mViolation[i] >= 0.0)
    {
      mNegativeVel[i] = -genCoord->getVel();

      mLowerBound[i] = -dInfinity;
      mUpperBound[i] = 0.0;

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

    mActive[i] = false;
  }
}

//==============================================================================
void JointLimitConstraint::fillLcpOde(ODELcp* _lcp, int _idx)
{
  size_t localIndex = 0;
  size_t dof = mJoint->getNumGenCoords();
  for (size_t i = 0; i < dof ; ++i)
  {
    if (mActive[i] == false)
      continue;

    if (_lcp->w[_idx + localIndex] != 0.0)
      std::cout << "_lcp->w[_idx + localIndex]: " << _lcp->w[_idx + localIndex]
                << std::endl;
    assert(_lcp->w[_idx + localIndex] == 0.0);

    double bouncingVel = -mViolation[i];

    if (bouncingVel > 0.0)
      bouncingVel = -DART_DEFAULT_JOINT_POSITION_ERROR_ALLOWANCE;
    else
      bouncingVel = +DART_DEFAULT_JOINT_POSITION_ERROR_ALLOWANCE;

    bouncingVel *= _lcp->invTimestep * DART_DEFAUT_JOINT_POSITION_ERP;

    _lcp->b[_idx + localIndex] = mNegativeVel[localIndex] + bouncingVel;

    _lcp->lb[_idx + localIndex] = mLowerBound[localIndex];
    _lcp->ub[_idx + localIndex] = mUpperBound[localIndex];

    assert(_lcp->frictionIndex[_idx] == -1);

    if (mLifeTime[i])
      _lcp->x[_idx + localIndex] = mOldX[localIndex];
    else
      _lcp->x[_idx + localIndex] = 0.0;

    ++localIndex;
  }
}

//==============================================================================
void JointLimitConstraint::applyUnitImpulse(int _localIndex)
{
  assert(0 <= _localIndex && _localIndex < mDim && "Invalid Index.");

  size_t localIndex = 0;
  dynamics::Skeleton* skeleton = mJoint->getSkeleton();

  size_t dof = mJoint->getNumGenCoords();
  for (size_t i = 0; i < dof; ++i)
  {
    if (mActive[i] == false)
      continue;

    skeleton->clearImpulseTest();
    mJoint->getGenCoord(localIndex)->setConstraintImpulse(1.0);
    skeleton->updateBiasImpulse(mBodyNode);
    skeleton->updateVelocityChange();

    ++localIndex;
  }
}

//==============================================================================
void JointLimitConstraint::getVelocityChange(double* _delVel, int _idx,
                                             bool /*_withCfm*/)
{
  assert(_delVel != NULL && "Null pointer is not allowed.");

  size_t localIndex = 0;
  size_t dof = mJoint->getNumGenCoords();
  for (size_t i = 0; i < dof ; ++i)
  {
    if (mActive[i] == false)
      continue;

    if (mJoint->getSkeleton()->isImpulseApplied())
      _delVel[_idx + localIndex] = mJoint->getGenCoord(i)->getVelChange();
    else
      _delVel[_idx + localIndex] = 0.0;

    ++localIndex;
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
void JointLimitConstraint::applyConstraintImpulse(double* _lambda, int _idx)
{
  size_t localIndex = 0;
  size_t dof = mJoint->getNumGenCoords();
  for (size_t i = 0; i < dof ; ++i)
  {
    if (mActive[i] == false)
      continue;

//    std::cout << "lambda[" << localIndex << "]: " << _lambda[_idx + localIndex]
//              << std::endl;

//    std::cout << "mJoint->getGenCoord(i)->getConstraintImpulse(): "
//              << mJoint->getGenCoord(i)->getConstraintImpulse()
//              << std::endl;

    mJoint->getGenCoord(i)->setConstraintImpulse(
//          mJoint->getGenCoord(i)->getConstraintImpulse()
//          +
          _lambda[_idx + localIndex]);

//    std::cout << "mJoint->getGenCoord(i)->getConstraintImpulse(): "
//              << mJoint->getGenCoord(i)->getConstraintImpulse()
//              << std::endl;

    mOldX[i] = _lambda[_idx + localIndex];

    ++localIndex;
  }
}

//==============================================================================
dynamics::Skeleton* JointLimitConstraint::getRootSkeleton() const
{
  return mJoint->getSkeleton()->mUnionRootSkeleton;
}

//==============================================================================
bool JointLimitConstraint::isActive()
{
  for (size_t i = 0; i < 6; ++i)
  {
    if (mActive[i])
      return true;
  }

  return false;
}

} // namespace constraint
} // namespace dart
