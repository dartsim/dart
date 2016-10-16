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

#include "dart/dynamics/InverseKinematics.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/DegreeOfFreedom.h"
#include "dart/dynamics/SimpleFrame.h"
#include "dart/dynamics/Joint.h"

namespace dart {
namespace dynamics {

//==============================================================================
static void convertJacobianMethodOutputToGradient(
    Eigen::VectorXd& grad, const std::vector<size_t>& dofs,
    InverseKinematics* mIK)
{
  const SkeletonPtr& skel = mIK->getNode()->getSkeleton();
  const Eigen::VectorXd mInitialPositionsCache = skel->getPositions(dofs);

  for(size_t i=0; i < dofs.size(); ++i)
    skel->getDof(dofs[i])->setVelocity(grad[i]);
  // Velocities of unused DOFs should already be set to zero.

  for(size_t i=0; i < dofs.size(); ++i)
  {
    Joint* joint = skel->getDof(dofs[i])->getJoint();
    joint->integratePositions(1.0);

    // Reset this joint's velocities to zero to avoid double-integrating
    const size_t numJointDofs = joint->getNumDofs();
    for(size_t j=0; j < numJointDofs; ++j)
      joint->setVelocity(j, 0.0);
  }

  grad = skel->getPositions(dofs);
  grad -= mInitialPositionsCache;
}

//==============================================================================
InverseKinematicsPtr InverseKinematics::create(JacobianNode* _node)
{
  return InverseKinematicsPtr(std::shared_ptr<InverseKinematics>(
                                new InverseKinematics(_node)));
}

//==============================================================================
InverseKinematics::~InverseKinematics()
{
  mTargetConnection.disconnect();
  mNodeConnection.disconnect();
}

//==============================================================================
bool InverseKinematics::solve(bool _applySolution)
{
  if(nullptr == mSolver)
  {
    dtwarn << "[InverseKinematics::solve] The Solver for an InverseKinematics "
           << "module associated with [" << mNode->getName() << "] is a "
           << "nullptr. You must reset the module's Solver before you can use "
           << "it.\n";
    return false;
  }

  if(nullptr == mProblem)
  {
    dtwarn << "[InverseKinematics::solve] The Problem for an InverseKinematics "
           << "module associated with [" << mNode->getName() << "] is a "
           << "nullptr. You must reset the module's Problem before you can use "
           << "it.\n";
    return false;
  }

  mProblem->setDimension(mDofs.size());

  mProblem->setInitialGuess(getPositions());

  const SkeletonPtr& skel = getNode()->getSkeleton();

  Eigen::VectorXd bounds(mDofs.size());
  for(size_t i=0; i < mDofs.size(); ++i)
    bounds[i] = skel->getDof(mDofs[i])->getPositionLowerLimit();
  mProblem->setLowerBounds(bounds);

  for(size_t i=0; i < mDofs.size(); ++i)
    bounds[i] = skel->getDof(mDofs[i])->getPositionUpperLimit();
  mProblem->setUpperBounds(bounds);

  // Many GradientMethod implementations use Joint::integratePositions, so we
  // need to clear out any velocities that might be in the Skeleton and then
  // reset those velocities later.
  Eigen::VectorXd originalVelocities = skel->getVelocities();
  for(size_t i=0; i < skel->getNumDofs(); ++i)
    skel->getDof(i)->setVelocity(0.0);

  if(_applySolution)
  {
    bool wasSolved = mSolver->solve();
    setPositions(mProblem->getOptimalSolution());
    skel->setVelocities(originalVelocities);
    return wasSolved;
  }

  Eigen::VectorXd originalPositions = getPositions();
  bool wasSolved = mSolver->solve();
  setPositions(originalPositions);
  skel->setVelocities(originalVelocities);
  return wasSolved;
}

//==============================================================================
bool InverseKinematics::solve(Eigen::VectorXd& positions, bool _applySolution)
{
  bool wasSolved = solve(_applySolution);
  positions = mProblem->getOptimalSolution();
  return wasSolved;
}

//==============================================================================
static std::shared_ptr<optimizer::Function> cloneIkFunc(
    const std::shared_ptr<optimizer::Function>& _function,
    InverseKinematics* _ik)
{
  std::shared_ptr<InverseKinematics::Function> ikFunc =
      std::dynamic_pointer_cast<InverseKinematics::Function>(_function);

  if(ikFunc)
    return ikFunc->clone(_ik);

  return _function;
}

//==============================================================================
InverseKinematicsPtr InverseKinematics::clone(JacobianNode* _newNode) const
{
  std::shared_ptr<InverseKinematics> newIK(new InverseKinematics(_newNode));
  newIK->setActive(isActive());
  newIK->setHierarchyLevel(getHierarchyLevel());
  newIK->setDofs(getDofs());
  newIK->setOffset(mOffset);
  newIK->setTarget(mTarget);

  newIK->setObjective(cloneIkFunc(mObjective, newIK.get()));
  newIK->setNullSpaceObjective(cloneIkFunc(mNullSpaceObjective, newIK.get()));

  newIK->mErrorMethod = mErrorMethod->clone(newIK.get());
  newIK->mGradientMethod = mGradientMethod->clone(newIK.get());

  newIK->setSolver(mSolver->clone());

  const std::shared_ptr<optimizer::Problem>& newProblem = newIK->getProblem();
  newProblem->setObjective( cloneIkFunc(mProblem->getObjective(),newIK.get()) );

  newProblem->removeAllEqConstraints();
  for(size_t i=0; i < mProblem->getNumEqConstraints(); ++i)
    newProblem->addEqConstraint(
          cloneIkFunc(mProblem->getEqConstraint(i), newIK.get()) );

  newProblem->removeAllIneqConstraints();
  for(size_t i=0; i < mProblem->getNumIneqConstraints(); ++i)
    newProblem->addIneqConstraint(
          cloneIkFunc(mProblem->getIneqConstraint(i), newIK.get()));

  newProblem->getSeeds() = mProblem->getSeeds();

  return newIK;
}

//==============================================================================
InverseKinematics::ErrorMethod::Properties::Properties(
    const Bounds& _bounds,
    double _errorClamp,
    const Eigen::Vector6d& _errorWeights)
  : mBounds(_bounds),
    mErrorLengthClamp(_errorClamp),
    mErrorWeights(_errorWeights)
{
  // Do nothing
}

//==============================================================================
InverseKinematics::ErrorMethod::ErrorMethod(
    InverseKinematics* _ik,
    const std::string& _methodName,
    const dart::dynamics::InverseKinematics::
        ErrorMethod::Properties& _properties)
  : mIK(_ik),
    mMethodName(_methodName),
    mLastError(Eigen::Vector6d::Constant(std::nan(""))),
    mProperties(_properties)
{
  // Do nothing
}

//==============================================================================
const Eigen::Vector6d& InverseKinematics::ErrorMethod::evalError(
    const Eigen::VectorXd& _q)
{
  if(_q.size() != static_cast<int>(mIK->getDofs().size()))
  {
    dterr << "[InverseKinematics::ErrorMethod::evalError] Mismatch between "
          << "joint positions size [" << _q.size() << "] and the available "
          << "degrees of freedom [" << mIK->getDofs().size() <<"]."
          << "\nSkeleton name: " << mIK->getNode()->getSkeleton()->getName()
          << "\nBody name: " << mIK->getNode()->getName()
          << "\nMethod name: " << mMethodName << "\n";
    mLastError.setZero();
    return mLastError;
  }

  if(_q.size() == 0)
  {
    mLastError.setZero();
    return mLastError;
  }

  if(_q.size() == mLastPositions.size())
  {
    bool repeat = true;
    for(int i=0; i<mLastPositions.size(); ++i)
    {
      if(_q[i] != mLastPositions[i])
      {
        repeat = false;
        break;
      }
    }

    if(repeat)
      return mLastError;
  }

  mIK->setPositions(_q);
  mLastPositions = _q;

  mLastError = computeError();
  return mLastError;
}

//==============================================================================
const std::string& InverseKinematics::ErrorMethod::getMethodName() const
{
  return mMethodName;
}

//==============================================================================
void InverseKinematics::ErrorMethod::setBounds(const Eigen::Vector6d& _lower,
                                               const Eigen::Vector6d& _upper)
{
  mProperties.mBounds.first = _lower;
  mProperties.mBounds.second = _upper;
  clearCache();
}

//==============================================================================
void InverseKinematics::ErrorMethod::setBounds(
    const std::pair<Eigen::Vector6d, Eigen::Vector6d>& _bounds)
{
  mProperties.mBounds = _bounds;
  clearCache();
}

//==============================================================================
const std::pair<Eigen::Vector6d, Eigen::Vector6d>&
InverseKinematics::ErrorMethod::getBounds() const
{
  return mProperties.mBounds;
}

//==============================================================================
void InverseKinematics::ErrorMethod::setAngularBounds(
    const Eigen::Vector3d& _lower, const Eigen::Vector3d& _upper)
{
  mProperties.mBounds.first.head<3>() = _lower;
  mProperties.mBounds.second.head<3>() = _upper;
  clearCache();
}

//==============================================================================
void InverseKinematics::ErrorMethod::setAngularBounds(
    const std::pair<Eigen::Vector3d, Eigen::Vector3d>& _bounds)
{
  setAngularBounds(_bounds.first, _bounds.second);
}

//==============================================================================
std::pair<Eigen::Vector3d, Eigen::Vector3d>
InverseKinematics::ErrorMethod::getAngularBounds() const
{
  return std::pair<Eigen::Vector3d, Eigen::Vector3d>(
        mProperties.mBounds.first.head<3>(),
        mProperties.mBounds.second.head<3>());
}

//==============================================================================
void InverseKinematics::ErrorMethod::setLinearBounds(
    const Eigen::Vector3d& _lower, const Eigen::Vector3d& _upper)
{
  mProperties.mBounds.first.tail<3>() = _lower;
  mProperties.mBounds.second.tail<3>() = _upper;
  clearCache();
}

//==============================================================================
void InverseKinematics::ErrorMethod::setLinearBounds(
    const std::pair<Eigen::Vector3d, Eigen::Vector3d>& _bounds)
{
  setLinearBounds(_bounds.first, _bounds.second);
}

//==============================================================================
std::pair<Eigen::Vector3d, Eigen::Vector3d>
InverseKinematics::ErrorMethod::getLinearBounds() const
{
  return std::pair<Eigen::Vector3d, Eigen::Vector3d>(
        mProperties.mBounds.first.tail<3>(),
        mProperties.mBounds.second.tail<3>());
}

//==============================================================================
void InverseKinematics::ErrorMethod::setErrorLengthClamp(double _clampSize)
{
  mProperties.mErrorLengthClamp = _clampSize;
  clearCache();
}

//==============================================================================
double InverseKinematics::ErrorMethod::getErrorLengthClamp() const
{
  return mProperties.mErrorLengthClamp;
}

//==============================================================================
void InverseKinematics::ErrorMethod::setErrorWeights(
    const Eigen::Vector6d& _weights)
{
  mProperties.mErrorWeights = _weights;
  clearCache();
}

//==============================================================================
const Eigen::Vector6d& InverseKinematics::ErrorMethod::getErrorWeights() const
{
  return mProperties.mErrorWeights;
}

//==============================================================================
void InverseKinematics::ErrorMethod::setAngularErrorWeights(
    const Eigen::Vector3d& _weights)
{
  mProperties.mErrorWeights.head<3>() = _weights;
  clearCache();
}

//==============================================================================
Eigen::Vector3d InverseKinematics::ErrorMethod::getAngularErrorWeights() const
{
  return mProperties.mErrorWeights.head<3>();
}

//==============================================================================
void InverseKinematics::ErrorMethod::setLinearErrorWeights(
    const Eigen::Vector3d& _weights)
{
  mProperties.mErrorWeights.tail<3>() = _weights;
  clearCache();
}

//==============================================================================
Eigen::Vector3d InverseKinematics::ErrorMethod::getLinearErrorWeights() const
{
  return mProperties.mErrorWeights.tail<3>();
}

//==============================================================================
void InverseKinematics::ErrorMethod::clearCache()
{
  // This will force the error to be recomputed the next time computeError is
  // called
  mLastPositions.resize(0);
}

//==============================================================================
InverseKinematics::TaskSpaceRegion::TaskSpaceRegion(
    InverseKinematics* _ik,
    const Properties& _properties,
    bool _computeFromCenter)
  : ErrorMethod(_ik, "TaskSpaceRegion", _properties),
    mComputeErrorFromCenter(_computeFromCenter)
{
  // Do nothing
}

//==============================================================================
std::unique_ptr<InverseKinematics::ErrorMethod>
InverseKinematics::TaskSpaceRegion::clone(InverseKinematics* _newIK) const
{
  return std::unique_ptr<ErrorMethod>(
        new TaskSpaceRegion(_newIK, mProperties, mComputeErrorFromCenter));
}

//==============================================================================
Eigen::Vector6d InverseKinematics::TaskSpaceRegion::computeError()
{
  // This is a slightly modified implementation of the Berenson et al Task Space
  // Region method found in "Task Space Regions: A Framework for
  // Pose-Constrained Manipulation Planning".
  //
  // The modification is that we handle the computation of translational error
  // independently from the computation of rotational error, whereas the work of
  // Berenson computes them simultaneously using homogeneous transform matrices.
  // Handling them in terms of their homogeneous matrices results in less direct
  // paths while performing gradient descent, because it produces a pseudo-error
  // in translation whenever rotational error exists.
  //
  // Also, we offer the option of always computing the error from the center of
  // the Task Space Region, which will often allow the end effector to enter the
  // valid constraint manifold faster than computing it from the edge of the
  // Task Space Region.

  // Use the target's transform with respect to its reference frame
  const Eigen::Isometry3d& targetTf =
      mIK->getTarget()->getRelativeTransform();
  // Use the actual transform with respect to the target's reference frame
  const Eigen::Isometry3d& actualTf =
      mIK->getNode()->getTransform(mIK->getTarget()->getParentFrame());

  // ^ This scheme makes it so that the bounds are expressed in the reference
  // frame of the target

  Eigen::Vector3d p_error = actualTf.translation() - targetTf.translation();
  if(mIK->hasOffset())
    p_error += actualTf.linear()*mIK->getOffset();

  Eigen::Matrix3d R_error = actualTf.linear() * targetTf.linear().transpose();

  Eigen::Vector6d displacement;
  displacement.head<3>() = math::matrixToEulerXYZ(R_error);
  displacement.tail<3>() = p_error;

  Eigen::Vector6d error;
  const Eigen::Vector6d& min = mProperties.mBounds.first;
  const Eigen::Vector6d& max = mProperties.mBounds.second;
  double tolerance = mIK->getSolver()->getTolerance();
  for(int i=0; i<6; ++i)
  {
    if( displacement[i] < min[i] )
    {
      if(mComputeErrorFromCenter)
      {
        if(std::isfinite(max[i]))
          error[i] = displacement[i] - (min[i]+max[i])/2.0;
        else
          error[i] = displacement[i] - (min[i]+tolerance);
      }
      else
      {
        error[i] = displacement[i] - min[i];
      }
    }
    else if( max[i] < displacement[i] )
    {
      if(mComputeErrorFromCenter)
      {
        if(std::isfinite(min[i]))
          error[i] = displacement[i] - (min[i]+max[i])/2.0;
        else
          error[i] = displacement[i] - (max[i]-tolerance);
      }
      else
      {
        error[i] = displacement[i] - max[i];
      }
    }
    else
      error[i] = 0.0;
  }

  error = error.cwiseProduct(mProperties.mErrorWeights);

  if(error.norm() > mProperties.mErrorLengthClamp)
    error = error.normalized()*mProperties.mErrorLengthClamp;

  if(!mIK->getTarget()->getParentFrame()->isWorld())
  {
    // Transform the error term into the world frame if it's not already
    const Eigen::Isometry3d& R =
        mIK->getTarget()->getParentFrame()->getWorldTransform();
    error.head<3>() = R.linear()*error.head<3>();
    error.tail<3>() = R.linear()*error.tail<3>();
  }

  return error;
}

//==============================================================================
InverseKinematics::GradientMethod::GradientMethod(
    InverseKinematics* _ik,
    const std::string& _methodName, double _clamp)
  : mIK(_ik),
    mMethodName(_methodName),
    mComponentWiseClamp(_clamp)
{
  // Do nothing
}

//==============================================================================
void InverseKinematics::GradientMethod::evalGradient(
    const Eigen::VectorXd& _q,
    Eigen::Map<Eigen::VectorXd> _grad)
{
  if(_q.size() != static_cast<int>(mIK->getDofs().size()))
  {
    dterr << "[InverseKinematics::GradientMethod::evalGradient] Mismatch "
          << "between joint positions size [" << _q.size() << "] and the "
          << "available degrees of freedom [" << mIK->getDofs().size() << "]."
          << "\nSkeleton name: " << mIK->getNode()->getSkeleton()->getName()
          << "\nBody name: " << mIK->getNode()->getName()
          << "\nMethod name: " << mMethodName << "\n";
    assert(false);
    mLastGradient.resize(_q.size());
    mLastGradient.setZero();
    _grad = mLastGradient;
    return;
  }

  if(_q.size() == 0)
  {
    _grad.setZero();
    return;
  }

  if(_q.size() == mLastPositions.size())
  {
    bool repeat = true;
    for(int i=0; i<mLastPositions.size(); ++i)
    {
      if(_q[i] != mLastPositions[i])
      {
        repeat = false;
        break;
      }
    }

    if(repeat)
    {
      _grad = mLastGradient;
      return;
    }
  }

  Eigen::Vector6d error = mIK->getErrorMethod().evalError(_q);
  mIK->setPositions(_q);
  mLastGradient.resize(_grad.size());
  computeGradient(error, mLastGradient);
  _grad = mLastGradient;
}

//==============================================================================
const std::string& InverseKinematics::GradientMethod::getMethodName() const
{
  return mMethodName;
}

//==============================================================================
void InverseKinematics::GradientMethod::clampGradient(
    Eigen::VectorXd& _grad) const
{
  for(int i=0; i<_grad.size(); ++i)
  {
    if(std::abs(_grad[i]) > mComponentWiseClamp)
      _grad[i] = _grad[i] > 0 ?  mComponentWiseClamp :
                                -mComponentWiseClamp;
  }
}

//==============================================================================
void InverseKinematics::GradientMethod::setComponentWiseClamp(double _clamp)
{
  mComponentWiseClamp = std::abs(_clamp);
}

//==============================================================================
double InverseKinematics::GradientMethod::getComponentWiseClamp() const
{
  return mComponentWiseClamp;
}

//==============================================================================
void InverseKinematics::GradientMethod::applyWeights(
    Eigen::VectorXd& _grad) const
{
  size_t numComponents = std::min(_grad.size(), mComponentWeights.size());
  for(size_t i = 0; i < numComponents; ++i)
    _grad[i] = mComponentWeights[i] * _grad[i];
}

//==============================================================================
void InverseKinematics::GradientMethod::setComponentWeights(
    const Eigen::VectorXd& _weights)
{
  mComponentWeights = _weights;
}

//==============================================================================
const Eigen::VectorXd&
InverseKinematics::GradientMethod::getComponentWeights() const
{
  return mComponentWeights;
}

//==============================================================================
void InverseKinematics::GradientMethod::clearCache()
{
  // This will force the gradient to be recomputed the next time computeGradient
  // is called
  mLastPositions.resize(0);
}

//==============================================================================
InverseKinematics::JacobianDLS::JacobianDLS(
    InverseKinematics* _ik, double _clamp, double _damping)
  : GradientMethod(_ik, "JacobianDLS", _clamp),
    mDamping(_damping)
{
  // Do nothing
}

//==============================================================================
std::unique_ptr<InverseKinematics::GradientMethod>
InverseKinematics::JacobianDLS::clone(InverseKinematics* _newIK) const
{
  return std::unique_ptr<GradientMethod>(
        new JacobianDLS(_newIK, mComponentWiseClamp, mDamping));
}

//==============================================================================
void InverseKinematics::JacobianDLS::computeGradient(
    const Eigen::Vector6d& _error,
    Eigen::VectorXd& _grad)
{
  const math::Jacobian& J = mIK->computeJacobian();

  int rows = J.rows(), cols = J.cols();
  if(rows <= cols)
  {
    _grad = J.transpose()*(pow(mDamping,2)*Eigen::MatrixXd::Identity(rows, rows)
            + J*J.transpose() ).inverse() * _error;
  }
  else
  {
    _grad = ( pow(mDamping,2)*Eigen::MatrixXd::Identity(cols, cols) +
            J.transpose()*J).inverse() * J.transpose() * _error;
  }

  convertJacobianMethodOutputToGradient(_grad, mIK->getDofs(), mIK);
  applyWeights(_grad);
  clampGradient(_grad);
}

//==============================================================================
void InverseKinematics::JacobianDLS::setDampingCoefficient(double _damping)
{
  mDamping = _damping;
}

//==============================================================================
double InverseKinematics::JacobianDLS::getDampingCoefficient() const
{
  return mDamping;
}

//==============================================================================
InverseKinematics::JacobianTranspose::JacobianTranspose(
    InverseKinematics* _ik, double _clamp)
  : GradientMethod(_ik, "JacobianTranspose", _clamp)
{
  // Do nothing
}

//==============================================================================
std::unique_ptr<InverseKinematics::GradientMethod>
InverseKinematics::JacobianTranspose::clone(InverseKinematics* _newIK) const
{
  return std::unique_ptr<GradientMethod>(
        new JacobianTranspose(_newIK, mComponentWiseClamp));
}

//==============================================================================
void InverseKinematics::JacobianTranspose::computeGradient(
    const Eigen::Vector6d& _error,
    Eigen::VectorXd& _grad)
{
  const math::Jacobian& J = mIK->computeJacobian();
  _grad = J.transpose() * _error;

  convertJacobianMethodOutputToGradient(_grad, mIK->getDofs(), mIK);
  applyWeights(_grad);
  clampGradient(_grad);
}

//==============================================================================
void InverseKinematics::setActive(bool _active)
{
  mActive = _active;
}

//==============================================================================
void InverseKinematics::setInactive()
{
  mActive = false;
}

//==============================================================================
bool InverseKinematics::isActive() const
{
  return mActive;
}

//==============================================================================
void InverseKinematics::setHierarchyLevel(size_t _level)
{
  mHierarchyLevel = _level;
}

//==============================================================================
size_t InverseKinematics::getHierarchyLevel() const
{
  return mHierarchyLevel;
}

//==============================================================================
void InverseKinematics::useChain()
{
  mDofs.clear();

  if(mNode->getNumDependentGenCoords() == 0)
  {
    setDofs(mDofs);
    return;
  }

  setDofs(mNode->getChainDofs());
}

//==============================================================================
void InverseKinematics::useWholeBody()
{
  setDofs(mNode->getDependentGenCoordIndices());
}

//==============================================================================
void InverseKinematics::setDofs(const std::vector<size_t>& _dofs)
{
  mDofs = _dofs;
  const std::vector<size_t>& entityDependencies =
      mNode->getDependentGenCoordIndices();

  mDofMap.resize(entityDependencies.size());
  for(size_t i=0; i<mDofMap.size(); ++i)
  {
    mDofMap[i] = -1;
    for(size_t j=0; j<mDofs.size(); ++j)
    {
      if(mDofs[j] == entityDependencies[i])
      {
        mDofMap[i] = j;
      }
    }
  }

  mProblem->setDimension(mDofs.size());
}

//==============================================================================
const std::vector<size_t>& InverseKinematics::getDofs() const
{
  return mDofs;
}

//==============================================================================
const std::vector<int>& InverseKinematics::getDofMap() const
{
  return mDofMap;
}

//==============================================================================
void InverseKinematics::setObjective(
    const std::shared_ptr<optimizer::Function>& _objective)
{
  mObjective = _objective;
}

//==============================================================================
const std::shared_ptr<optimizer::Function>& InverseKinematics::getObjective()
{
  return mObjective;
}

//==============================================================================
std::shared_ptr<const optimizer::Function>
InverseKinematics::getObjective() const
{
  return mObjective;
}

//==============================================================================
void InverseKinematics::setNullSpaceObjective(
    const std::shared_ptr<optimizer::Function>& _nsObjective)
{
  mNullSpaceObjective = _nsObjective;
}

//==============================================================================
const std::shared_ptr<optimizer::Function>&
InverseKinematics::getNullSpaceObjective()
{
  return mNullSpaceObjective;
}

//==============================================================================
std::shared_ptr<const optimizer::Function>
InverseKinematics::getNullSpaceObjective() const
{
  return mNullSpaceObjective;
}

//==============================================================================
bool InverseKinematics::hasNullSpaceObjective() const
{
  return (nullptr != mNullSpaceObjective);
}

//==============================================================================
InverseKinematics::ErrorMethod& InverseKinematics::getErrorMethod()
{
  return *mErrorMethod;
}

//==============================================================================
const InverseKinematics::ErrorMethod& InverseKinematics::getErrorMethod() const
{
  return *mErrorMethod;
}

//==============================================================================
InverseKinematics::GradientMethod& InverseKinematics::getGradientMethod()
{
  return *mGradientMethod;
}

//==============================================================================
const InverseKinematics::GradientMethod&
InverseKinematics::getGradientMethod() const
{
  return *mGradientMethod;
}

//==============================================================================
const std::shared_ptr<optimizer::Problem>& InverseKinematics::getProblem()
{
  return mProblem;
}

//==============================================================================
std::shared_ptr<const optimizer::Problem> InverseKinematics::getProblem() const
{
  return mProblem;
}

//==============================================================================
void InverseKinematics::resetProblem(bool _clearSeeds)
{
  mProblem->removeAllEqConstraints();
  mProblem->removeAllIneqConstraints();

  if(_clearSeeds)
    mProblem->clearAllSeeds();

  mProblem->setObjective(Eigen::make_aligned_shared<Objective>(this));
  mProblem->addEqConstraint(std::make_shared<Constraint>(this));

  mProblem->setDimension(mDofs.size());
}

//==============================================================================
void InverseKinematics::setSolver(
    const std::shared_ptr<optimizer::Solver>& _newSolver)
{
  mSolver = _newSolver;
  if(nullptr == mSolver)
    return;

  mSolver->setProblem(getProblem());
}

//==============================================================================
const std::shared_ptr<optimizer::Solver>& InverseKinematics::getSolver()
{
  return mSolver;
}

//==============================================================================
std::shared_ptr<const optimizer::Solver> InverseKinematics::getSolver() const
{
  return mSolver;
}

//==============================================================================
void InverseKinematics::setOffset(const Eigen::Vector3d& _offset)
{
  if(Eigen::Vector3d::Zero() == _offset)
    mHasOffset = false;
  else
    mHasOffset = true;

  clearCaches();
  mOffset = _offset;
}

//==============================================================================
const Eigen::Vector3d& InverseKinematics::getOffset() const
{
  return mOffset;
}

//==============================================================================
bool InverseKinematics::hasOffset() const
{
  return mHasOffset;
}

//==============================================================================
void InverseKinematics::setTarget(std::shared_ptr<SimpleFrame> _newTarget)
{
  if(nullptr == _newTarget)
  {
    _newTarget = SimpleFramePtr(
          new SimpleFrame(Frame::World(), mNode->getName()+"_target",
                          mNode->getWorldTransform()));
  }

  mTarget = _newTarget;
  resetTargetConnection();
}

//==============================================================================
std::shared_ptr<SimpleFrame> InverseKinematics::getTarget()
{
  return mTarget;
}

//==============================================================================
std::shared_ptr<const SimpleFrame> InverseKinematics::getTarget() const
{
  return mTarget;
}

//==============================================================================
JacobianNode* InverseKinematics::getNode()
{
  return getAffiliation();
}

//==============================================================================
const JacobianNode* InverseKinematics::getNode() const
{
  return getAffiliation();
}

//==============================================================================
JacobianNode* InverseKinematics::getAffiliation()
{
  return mNode;
}

//==============================================================================
const JacobianNode* InverseKinematics::getAffiliation() const
{
  return mNode;
}

//==============================================================================
const math::Jacobian& InverseKinematics::computeJacobian() const
{
  // TODO(MXG): Test whether we can safely use a const reference here instead of
  // just a regular const
  const math::Jacobian fullJacobian = hasOffset()?
        getNode()->getWorldJacobian(mOffset) : getNode()->getWorldJacobian();

  mJacobian.setZero(6, getDofs().size());

  for(int i=0; i< static_cast<int>(getDofMap().size()); ++i)
  {
    int j = getDofMap()[i];
    if(j >= 0)
      mJacobian.block<6,1>(0,j) = fullJacobian.block<6,1>(0,i);
  }

  return mJacobian;
}

//==============================================================================
Eigen::VectorXd InverseKinematics::getPositions() const
{
  return mNode->getSkeleton()->getPositions(mDofs);
}

//==============================================================================
void InverseKinematics::setPositions(const Eigen::VectorXd& _q)
{
  if(_q.size() != static_cast<int>(mDofs.size()))
  {
    dterr << "[InverseKinematics::setPositions] Mismatch between joint "
          << "positions size [" << _q.size() << "] and number of available "
          << "degrees of freedom [" << mDofs.size() << "]\n";
    return;
  }

  const dart::dynamics::SkeletonPtr& skel = getNode()->getSkeleton();
  skel->setPositions(mDofs, _q);
}

//==============================================================================
void InverseKinematics::clearCaches()
{
  mErrorMethod->clearCache();
  mGradientMethod->clearCache();
}

//==============================================================================
InverseKinematics::Objective::Objective(InverseKinematics* _ik)
  : mIK(_ik)
{
  // Do nothing
}

//==============================================================================
optimizer::FunctionPtr InverseKinematics::Objective::clone(
    InverseKinematics* _newIK) const
{
  return std::make_shared<Objective>(_newIK);
}

//==============================================================================
double InverseKinematics::Objective::eval(const Eigen::VectorXd& _x)
{
  if(nullptr == mIK)
  {
    dterr << "[InverseKinematics::Objective::eval] Attempting to use an "
          << "Objective function of an expired InverseKinematics module!\n";
    assert(false);
    return 0;
  }

  double cost = 0.0;

  if(mIK->mObjective)
    cost += mIK->mObjective->eval(_x);

  if(mIK->mNullSpaceObjective)
    cost += mIK->mNullSpaceObjective->eval(_x);

  return cost;
}

//==============================================================================
void InverseKinematics::Objective::evalGradient(
    const Eigen::VectorXd& _x, Eigen::Map<Eigen::VectorXd> _grad)
{
  if(nullptr == mIK)
  {
    dterr << "[InverseKinematics::Objective::evalGradient] Attempting to use "
          << "an Objective function of an expired InverseKinematics module!\n";
    assert(false);
    return;
  }

  if(mIK->mObjective)
    mIK->mObjective->evalGradient(_x, _grad);
  else
    _grad.setZero();

  if(mIK->mNullSpaceObjective)
  {
    mGradCache.resize(_grad.size());
    Eigen::Map<Eigen::VectorXd> gradMap(mGradCache.data(), _grad.size());
    mIK->mNullSpaceObjective->evalGradient(_x, gradMap);

    mIK->setPositions(_x);

    const math::Jacobian& J = mIK->computeJacobian();
    mSVDCache.compute(J, Eigen::ComputeFullV);
    math::extractNullSpace(mSVDCache, mNullSpaceCache);
    _grad += mNullSpaceCache*mNullSpaceCache.transpose() * mGradCache;
  }
}

//==============================================================================
InverseKinematics::Constraint::Constraint(InverseKinematics* _ik)
  : mIK(_ik)
{
  // Do nothing
}

//==============================================================================
optimizer::FunctionPtr InverseKinematics::Constraint::clone(
    InverseKinematics* _newIK) const
{
  return std::make_shared<Constraint>(_newIK);
}

//==============================================================================
double InverseKinematics::Constraint::eval(const Eigen::VectorXd& _x)
{
  if(nullptr == mIK)
  {
    dterr << "[InverseKinematics::Constraint::eval] Attempting to use a "
          << "Constraint function of an expired InverseKinematics module!\n";
    assert(false);
    return 0;
  }

  return mIK->getErrorMethod().evalError(_x).norm();
}

//==============================================================================
void InverseKinematics::Constraint::evalGradient(
    const Eigen::VectorXd& _x, Eigen::Map<Eigen::VectorXd> _grad)
{
  if(nullptr == mIK)
  {
    dterr << "[InverseKinematics::Constraint::evalGradient] Attempting to use "
          << "a Constraint function of an expired InverseKinematics module!\n";
    assert(false);
    return;
  }

  mIK->getGradientMethod().evalGradient(_x, _grad);
}

//==============================================================================
InverseKinematics::InverseKinematics(JacobianNode* _node)
  : mActive(true),
    mHierarchyLevel(0),
    mOffset(Eigen::Vector3d::Zero()),
    mHasOffset(false),
    mNode(_node)
{
  initialize();
}

//==============================================================================
void InverseKinematics::initialize()
{
  // Default to having no objectives
  setObjective(nullptr);
  setNullSpaceObjective(nullptr);

  mProblem = std::make_shared<optimizer::Problem>();
  resetProblem();

  // The default error method is the one based on Task Space Regions
  setErrorMethod<TaskSpaceRegion>();

  // The default gradient method is damped least squares Jacobian
  setGradientMethod<JacobianDLS>();

  // Create the default target for this IK module
  setTarget(nullptr);

  // Set up the cache clearing connections. These connections will ensure that
  // any time an outsider alters the transform of the object's transform or the
  // target's transform, our saved caches will be cleared so that the error and
  // gradient will be recomputed the next time they are polled.
  resetTargetConnection();
  resetNodeConnection();

  // By default, we use the linkage when performing IK
  useChain();

  // Default to the native DART gradient descent solver
  std::shared_ptr<optimizer::GradientDescentSolver> solver =
      std::make_shared<optimizer::GradientDescentSolver>(mProblem);
  solver->setStepSize(1.0);
  mSolver = solver;
}

//==============================================================================
void InverseKinematics::resetTargetConnection()
{
  mTargetConnection.disconnect();
  mTargetConnection = mTarget->onTransformUpdated.connect(
        [=](const Entity*)
        { this->clearCaches(); } );
  clearCaches();
}

//==============================================================================
void InverseKinematics::resetNodeConnection()
{
  mNodeConnection.disconnect();
  mNodeConnection = mNode->onTransformUpdated.connect(
        [=](const Entity*)
        { this->clearCaches(); } );
  clearCaches();
}

} // namespace dynamics
} // namespace dart
