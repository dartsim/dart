/*
 * Copyright (c) 2015, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Michael X. Grey <mxgrey@gatech.edu>
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

#include "dart/constraint/BalanceConstraint.h"

#include "dart/dynamics/Skeleton.h"
#include "dart/dynamics/EndEffector.h"
#include "dart/dynamics/Joint.h"

namespace dart {
namespace constraint {

//==============================================================================
BalanceConstraint::BalanceConstraint(
    const std::shared_ptr<dynamics::HierarchicalIK>& _ik,
    BalanceMethod_t _balanceMethod, ErrorMethod_t _errorMethod)
  : mIK(_ik),
    mErrorMethod(_errorMethod),
    mBalanceMethod(_balanceMethod),
    mOptimizationTolerance(1e-8),
    mDamping(0.05),
    mLastError(Eigen::Vector3d::Zero()),
    mLastCOM(Eigen::Vector3d::Constant(std::nan(""))),
    mLastSupportVersion(dynamics::INVALID_INDEX)
{
  // Do nothing
}

//==============================================================================
optimizer::FunctionPtr BalanceConstraint::clone(
    const std::shared_ptr<dynamics::HierarchicalIK>& _newIK) const
{
  return std::make_shared<BalanceConstraint>(_newIK, mBalanceMethod);
}

//==============================================================================
double BalanceConstraint::eval(const Eigen::VectorXd& _x)
{
  const std::shared_ptr<dynamics::HierarchicalIK>& ik = mIK.lock();

  if(nullptr == ik)
  {
    dterr << "[BalanceConstraint::eval] Attempting to call a BalanceConstraint "
          << "function associated to a HierarchicalIK that no longer exists!\n";
    assert(false);
    return 0.0;
  }

  const dynamics::SkeletonPtr& skel = ik->getSkeleton();

  if(nullptr == skel)
  {
    dterr << "[BalanceConstraint::eval] Attempting to call a BalanceConstraint "
          << "function on a Skeleton which no longer exists!\n";
    assert(false);
    return 0.0;
  }

  skel->setPositions(_x);

  const Eigen::Vector3d& com = skel->getCOM();
  if(skel->getSupportVersion() == mLastSupportVersion)
  {
    // Nothing has moved since the last time error was computed, so we just
    // return our last result
    return mLastError.norm();
  }

  const math::SupportPolygon& polygon = skel->getSupportPolygon();
  if(polygon.empty())
  {
    mLastError.setZero();
    return 0.0;
  }

  const std::pair<Eigen::Vector3d, Eigen::Vector3d>& axes =
      skel->getSupportAxes();
  Eigen::Vector2d projected_com(com.dot(axes.first), com.dot(axes.second));
  Eigen::Vector2d projected_error(Eigen::Vector2d::Zero());

  if(FROM_CENTROID == mErrorMethod || OPTIMIZE_BALANCE == mErrorMethod)
  {
    bool zeroError = false;
    if(FROM_CENTROID == mErrorMethod)
    {
      zeroError = math::isInsideSupportPolygon(projected_com, polygon, true);
    }

    if(!zeroError)
    {
      const Eigen::Vector2d& centroid = skel->getSupportCentroid();
      projected_error = projected_com - centroid;
    }

    if(OPTIMIZE_BALANCE == mErrorMethod
       && projected_error.norm() < mOptimizationTolerance)
    {
      // Drop the error to zero if we're within the tolerance
      projected_error.setZero();
    }
  }
  else if(FROM_EDGE == mErrorMethod)
  {
    bool zeroError = math::isInsideSupportPolygon(projected_com, polygon, true);

    if(!zeroError)
    {
      size_t closestIndex1, closestIndex2;
      const Eigen::Vector2d closestPoint =
          math::computeClosestPointOnSupportPolygon(
            closestIndex1, closestIndex2, projected_com, polygon);

      // Save the indices of the EndEffectors that are closest to the center of
      // mass
      const std::vector<size_t>& indexMap = skel->getSupportIndices();
      mClosestEndEffector[0] = indexMap[closestIndex1];
      mClosestEndEffector[1] = indexMap[closestIndex2];

      projected_error = projected_com - closestPoint;
    }
  }

  mLastError = axes.first * projected_error[0]
             + axes.second * projected_error[1];

  return mLastError.norm();
}

//==============================================================================
template <typename JacType>
static void addDampedPseudoInverseToGradient(
    Eigen::Map<Eigen::VectorXd>& grad, const JacType& J,
    const Eigen::MatrixXd& nullspace, const Eigen::Vector3d& error,
    double damping)
{
  int rows = J.rows(), cols = J.cols();
  if(rows <= cols)
  {
    grad += nullspace * J.transpose()*(
          pow(damping,2)*Eigen::MatrixXd::Identity(rows, rows)
          + J*J.transpose() ).inverse() * error;
  }
  else
  {
    grad += nullspace * (
              pow(damping,2)*Eigen::MatrixXd::Identity(cols, cols)
              + J.transpose()*J).inverse() * J.transpose() * error;
  }
}

//==============================================================================
static void convertJacobianMethodOutputToGradient(
    Eigen::Map<Eigen::VectorXd>& grad,
    const std::weak_ptr<dynamics::HierarchicalIK>& mIK)
{
  const dart::dynamics::SkeletonPtr& skel = mIK.lock()->getSkeleton();
  skel->setVelocities(grad);

  const Eigen::VectorXd mInitialPositionsCache = skel->getPositions();

  for(size_t i=0; i < skel->getNumJoints(); ++i)
    skel->getJoint(i)->integratePositions(1.0);

  // Clear out the velocities so we don't interfere with other Jacobian methods
  for(size_t i=0; i < skel->getNumDofs(); ++i)
    skel->setVelocity(i, 0.0);

  grad = skel->getPositions();
  grad -= mInitialPositionsCache;
}

//==============================================================================
void BalanceConstraint::evalGradient(const Eigen::VectorXd& _x,
                                     Eigen::Map<Eigen::VectorXd> _grad)
{
  _grad.setZero();
  if(eval(_x) == 0.0)
      return;

  // If eval(_x) was non-zero, then the IK and Skeleton should still exist, so
  // we shouldn't need to test their existance.
  const dynamics::SkeletonPtr& skel = mIK.lock()->getSkeleton();
  const size_t nDofs = skel->getNumDofs();

  if(SHIFT_COM == mBalanceMethod)
  {
    // Compute the gradient whose negative will move the center of mass
    // towards the support polygon without moving the supporting end effector
    // locations

    mNullSpaceCache.setIdentity(nDofs, nDofs);
    size_t numEE = skel->getNumEndEffectors();
    // Build up the null space of the supporting end effectors
    for(size_t i=0; i < numEE; ++i)
    {
      const dynamics::EndEffector* ee = skel->getEndEffector(i);

      // Skip this EndEffector if it is not being used for support
      if(!ee->getSupport() || !ee->getSupport()->isActive())
        continue;

      mEEJacCache = skel->getLinearJacobian(ee);

      mSVDCache.compute(mEEJacCache, Eigen::ComputeFullV);
      math::extractNullSpace(mSVDCache, mPartialNullSpaceCache);

      if(mPartialNullSpaceCache.rows() > 0
         && mPartialNullSpaceCache.cols() > 0)
      {
        mNullSpaceCache *= mPartialNullSpaceCache
                           * mPartialNullSpaceCache.transpose();
      }
      else
      {
        // There is no null space anymore
        mNullSpaceCache.setZero();
        break;
      }
    }

    mComJacCache = skel->getCOMLinearJacobian();

    addDampedPseudoInverseToGradient(_grad, mComJacCache, mNullSpaceCache,
                                     mLastError, mDamping);
  }
  else if(SHIFT_SUPPORT == mBalanceMethod)
  {
    if(FROM_CENTROID == mErrorMethod || OPTIMIZE_BALANCE == mErrorMethod)
    {
      // Compute the gradient whose negative will move all the supporting end
      // effectors towards the center of mass without moving the center of mass
      // location

      mComJacCache = skel->getCOMLinearJacobian();

      mSVDCache.compute(mComJacCache, Eigen::ComputeFullV);
      math::extractNullSpace(mSVDCache, mPartialNullSpaceCache);

      if(mPartialNullSpaceCache.rows() > 0
         && mPartialNullSpaceCache.cols() > 0)
      {
        mNullSpaceCache = mPartialNullSpaceCache
                          * mPartialNullSpaceCache.transpose();
      }
      else
      {
        mNullSpaceCache.setZero(nDofs, nDofs);
      }

      size_t numEE = skel->getNumEndEffectors();
      for(size_t i=0; i < numEE; ++i)
      {
        const dynamics::EndEffector* ee = skel->getEndEffector(i);

        if(!ee->getSupport() || !ee->getSupport()->isActive())
          continue;

        mEEJacCache = skel->getLinearJacobian(ee);

        addDampedPseudoInverseToGradient(_grad, mEEJacCache, mNullSpaceCache,
                                         -mLastError, mDamping);
        // Error is negative because we want to move the supports towards the
        // center of mass, not the center of mass towards the support
      }
    }
    else if(FROM_EDGE == mErrorMethod)
    {
      // Compute the gradient that will shift the end effectors that are closest
      // to the center of mass

      mComJacCache = skel->getCOMLinearJacobian();

      mSVDCache.compute(mComJacCache, Eigen::ComputeFullV);
      math::extractNullSpace(mSVDCache, mPartialNullSpaceCache);

      if(mPartialNullSpaceCache.rows() > 0
         && mPartialNullSpaceCache.cols() > 0)
      {
        mNullSpaceCache = mPartialNullSpaceCache
                          * mPartialNullSpaceCache.transpose();
      }
      else
      {
        mNullSpaceCache.setZero(nDofs, nDofs);
      }

      for(size_t i=0; i<2; ++i)
      {
        const dynamics::EndEffector* ee =
            skel->getEndEffector(mClosestEndEffector[i]);

        if(!ee->getSupport() || !ee->getSupport()->isActive())
        {
          dtwarn << "[BalanceConstraint::evalGradient] The EndEffector named ["
                 << ee->getName() << "] was identified as the closest "
                 << "supporting EndEffector, but it does not appear to be a "
                 << "supporting EndEffector at all. This is most likely a bug, "
                 << "please report it!\n";
          continue;
        }

        mEEJacCache = skel->getLinearJacobian(ee);

        addDampedPseudoInverseToGradient(_grad, mEEJacCache, mNullSpaceCache,
                                         -mLastError, mDamping);

        // Quit after the first step if there is only one closest end effector
        if(mClosestEndEffector[0] == mClosestEndEffector[1])
          break;
      }
    }
  }

  convertJacobianMethodOutputToGradient(_grad, mIK);
}

//==============================================================================
void BalanceConstraint::setErrorMethod(ErrorMethod_t _method)
{
  if(mErrorMethod == _method)
    return;

  mErrorMethod = _method;
  clearCaches();
}

//==============================================================================
BalanceConstraint::ErrorMethod_t BalanceConstraint::getErrorMethod() const
{
  return mErrorMethod;
}

//==============================================================================
void BalanceConstraint::setBalanceMethod(BalanceMethod_t _method)
{
  if(mBalanceMethod == _method)
    return;

  mBalanceMethod = _method;
  clearCaches();
}

//==============================================================================
BalanceConstraint::BalanceMethod_t BalanceConstraint::getBalanceMethod() const
{
  return mBalanceMethod;
}

//==============================================================================
void BalanceConstraint::setOptimizationTolerance(double _tol)
{
  if(mOptimizationTolerance == _tol)
    return;

  mOptimizationTolerance = _tol;
  clearCaches();
}

//==============================================================================
double BalanceConstraint::getOptimizationTolerance() const
{
  return mOptimizationTolerance;
}

//==============================================================================
void BalanceConstraint::setPseudoInverseDamping(double _damping)
{
  if(mDamping == _damping)
    return;

  mDamping = _damping;
  clearCaches();
}

//==============================================================================
const Eigen::Vector3d& BalanceConstraint::getLastError() const
{
  return mLastError;
}

//==============================================================================
void BalanceConstraint::clearCaches()
{
  // This will ensure that the comparison test in eval() fails
  mLastCOM = Eigen::Vector3d::Constant(std::nan(""));
}

} // namespace constraint
} // namespace dart
