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
#include "DegreeOfFreedom.h"

namespace dart {
namespace dynamics {

//===============================================================================
InverseKinematics::InverseKinematics(JacobianEntity* _entity)
  : mActive(false),
    mHierarchyLevel(0),
    mEntity(_entity)
{
  initialize();
}

//==============================================================================
InverseKinematics::~InverseKinematics()
{
  mTargetConnection.disconnect();
  mEntityConnection.disconnect();

  mOverallObjective->clearCostFunction(true);
  mOverallObjective->clearGradientFunction();
  mOverallObjective->clearHessianFunction();

  mConstraint->clearCostFunction(true);
  mConstraint->clearGradientFunction();
  mConstraint->clearHessianFunction();
}

//==============================================================================
InverseKinematics::ErrorMethod::ErrorMethod(InverseKinematics* _ik,
    const std::string& _methodName)
  : mIK(_ik),
    mMethodName(_methodName),
    mLastError(Eigen::Vector6d::Constant(std::nan(""))),
    mBounds(Eigen::Vector6d::Constant(-DefaultIKTolerance),
            Eigen::Vector6d::Constant( DefaultIKTolerance)),
    mErrorLengthClamp(DefaultIKErrorClamp)
{
  setAngularErrorWeights();
  setLinearErrorWeights();
}

//==============================================================================
const Eigen::Vector6d& InverseKinematics::ErrorMethod::computeError(
    const Eigen::VectorXd& _q)
{
  if(_q.size() != static_cast<int>(mIK->getDofs().size()))
  {
    dterr << "[InverseKinematics::ErrorMethod::computeError] Mismatch between "
          << "configuration size [" << _q.size() << "] and the available "
          << "degrees of freedom [" << mIK->getDofs().size() <<"]."
          << "\nSkeleton name: " << mIK->getEntity()->getSkeleton()->getName()
          << "\nBody name: " << mIK->getEntity()->getName()
          << "\nMethod name: " << mMethodName << "\n";
    mLastError.setZero();
    return mLastError;
  }

  if(_q.size() == mLastConfig.size())
  {
    bool repeat = true;
    for(int i=0; i<mLastConfig.size(); ++i)
    {
      if(_q[i] != mLastConfig[i])
      {
        repeat = false;
        break;
      }
    }

    if(repeat)
      return mLastError;
  }

  mIK->setConfiguration(_q);
  mLastConfig = _q;

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
  mBounds.first = _lower;
  mBounds.second = _upper;
}

//==============================================================================
void InverseKinematics::ErrorMethod::setBounds(
    const std::pair<Eigen::Vector6d, Eigen::Vector6d>& _bounds)
{
  mBounds = _bounds;
}

//==============================================================================
const std::pair<Eigen::Vector6d, Eigen::Vector6d>&
InverseKinematics::ErrorMethod::getBounds() const
{
  return mBounds;
}

//==============================================================================
void InverseKinematics::ErrorMethod::setAngularBounds(
    const Eigen::Vector3d& _lower, const Eigen::Vector3d& _upper)
{
  mBounds.first.head<3>() = _lower;
  mBounds.second.head<3>() = _upper;
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
        mBounds.first.head<3>(), mBounds.second.head<3>());
}

//==============================================================================
void InverseKinematics::ErrorMethod::setLinearBounds(
    const Eigen::Vector3d& _lower, const Eigen::Vector3d& _upper)
{
  mBounds.first.tail<3>() = _lower;
  mBounds.second.tail<3>() = _upper;
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
        mBounds.first.tail<3>(), mBounds.second.tail<3>());
}

//==============================================================================
void InverseKinematics::ErrorMethod::setErrorLengthClamp(double _clampSize)
{
  mErrorLengthClamp = _clampSize;
}

//==============================================================================
double InverseKinematics::ErrorMethod::getErrorLengthClamp() const
{
  return mErrorLengthClamp;
}

//==============================================================================
void InverseKinematics::ErrorMethod::setErrorWeights(
    const Eigen::Vector6d& _weights)
{
  mErrorWeights = _weights;
}

//==============================================================================
const Eigen::Vector6d& InverseKinematics::ErrorMethod::getErrorWeights() const
{
  return mErrorWeights;
}

//==============================================================================
void InverseKinematics::ErrorMethod::setAngularErrorWeights(
    const Eigen::Vector3d& _weights)
{
  mErrorWeights.head<3>() = _weights;
}

//==============================================================================
Eigen::Vector3d InverseKinematics::ErrorMethod::getAngularErrorWeights() const
{
  return mErrorWeights.head<3>();
}

//==============================================================================
void InverseKinematics::ErrorMethod::setLinearErrorWeights(
    const Eigen::Vector3d& _weights)
{
  mErrorWeights.tail<3>() = _weights;
}

//==============================================================================
Eigen::Vector3d InverseKinematics::ErrorMethod::getLinearErrorWeights() const
{
  return mErrorWeights.tail<3>();
}

//==============================================================================
void InverseKinematics::ErrorMethod::clearCache()
{
  // This will force the error to be recomputed the next time computeError is
  // called
  if(mLastConfig.size() > 0)
    mLastConfig[0] = std::nan("");
}

//==============================================================================
InverseKinematics::TaskSpaceRegion::TaskSpaceRegion(
    InverseKinematics* _ik)
  : ErrorMethod(_ik, "EulerAngleXYZ"),
    mComputeErrorFromCenter(true)
{
  // Do nothing
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
  // paths while performing gradient descent, because there is a pseudo-error in
  // translation whenever rotational error exists.
  //
  // Also, we offer the option of always computing the error from the center of
  // the Task Space Region, which will often allow the end effector to enter the
  // valid constraint manifold faster than computing it from the edge of the
  // Task Space Region.

  // Use the target's transform with respect to its reference frame
  const Eigen::Isometry3d& targetTf =
      this->mIK->getTarget()->getRelativeTransform();
  // Use the actual transform with respect to the target's reference frame
  const Eigen::Isometry3d& actualTf =
      this->mIK->getEntity()->getTransform(
        this->mIK->getTarget()->getParentFrame());

  // ^ This scheme makes it so that the bounds are expressed in the reference
  // frame of the target

  Eigen::Vector3d p_error = actualTf.translation() - targetTf.translation();
  Eigen::Matrix3d R_error = actualTf.linear() * targetTf.linear().transpose();

  Eigen::Vector6d displacement;
  displacement.head<3>() = math::matrixToEulerXYZ(R_error);
  displacement.tail<3>() = p_error;

  Eigen::Vector6d error;
  const Eigen::Vector6d& min = this->mBounds.first;
  const Eigen::Vector6d& max = this->mBounds.second;
  double tolerance = mIK->getSolver()->getTolerance();
  for(int i=0; i<6; ++i)
  {
    if( displacement[i] < min[i] )
    {
      if(mComputeErrorFromCenter)
      {
        if(!std::isinf(max[i]))
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
      if(mComputeErrorFromCenter && !std::isinf(min[i]))
      {
        if(!std::isinf(min[i]))
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

  error = error.cwiseProduct(this->mErrorWeights);

  if(error.norm() > this->mErrorLengthClamp)
    error = error.normalized()*this->mErrorLengthClamp;

  if(!this->mIK->getTarget()->getParentFrame()->isWorld())
  {
    // Transform the error term into the world frame if it's not already
    const Eigen::Isometry3d& R =
        this->mIK->getTarget()->getParentFrame()->getWorldTransform();
    error.head<3>() = R.linear()*error.head<3>();
    error.tail<3>() = R.linear()*error.tail<3>();
  }

  return error;
}

//==============================================================================
InverseKinematics::GradientMethod::GradientMethod(
    InverseKinematics* _ik, const std::string& _methodName)
  : mIK(_ik),
    mMethodName(_methodName)
{
  setComponentWiseClamp();
}

//==============================================================================
void InverseKinematics::GradientMethod::computeGradient(
    const Eigen::VectorXd& _q,
    Eigen::Map<Eigen::VectorXd> _grad)
{
  if(_q.size() != static_cast<int>(mIK->getDofs().size()))
  {
    dterr << "[InverseKinematics::GradientMethod::computeGradient] Mismatch "
          << "between configuration size [" << _q.size() << "] and the "
          << "available degrees of freedom [" << mIK->getDofs().size() << "]."
          << "\nSkeleton name: " << mIK->getEntity()->getSkeleton()->getName()
          << "\nBody name: " << mIK->getEntity()->getName()
          << "\nMethod name: " << mMethodName << "\n";
    mLastGradient.resize(_q.size());
    mLastGradient.setZero();
    _grad = mLastGradient;
    return;
  }

  if(_q.size() == mLastConfig.size())
  {
    bool repeat = true;
    for(int i=0; i<mLastConfig.size(); ++i)
    {
      if(_q[i] != mLastConfig[i])
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

  Eigen::Vector6d error = mIK->getErrorMethod()->computeError(_q);
  mIK->setConfiguration(_q);
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
    if(std::abs(_grad[i]) > this->mComponentWiseClamp)
      _grad[i] = _grad[i] > 0 ?  this->mComponentWiseClamp :
                                -this->mComponentWiseClamp;
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
void InverseKinematics::GradientMethod::clearCache()
{
  // This will force the gradient to be recomputed the next time computeGradient
  // is called
  if(mLastConfig.size() > 0)
    mLastConfig[0] = std::nan("");
}

//==============================================================================
InverseKinematics::JacobianDLS::JacobianDLS(
    InverseKinematics* _ik)
  : GradientMethod(_ik, "JacobianDLS")
{
  setDampingCoefficient();
}

//==============================================================================
void InverseKinematics::JacobianDLS::computeGradient(
    const Eigen::Vector6d& _error,
    Eigen::VectorXd& _grad)
{
  const math::Jacobian& J = this->mIK->computeJacobian();

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

  this->clampGradient(_grad);
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
InverseKinematics::JacobianTranspose::JacobianTranspose(InverseKinematics* _ik)
  : GradientMethod(_ik, "JacobianTranspose")
{
  // Do nothing
}

//==============================================================================
void InverseKinematics::JacobianTranspose::computeGradient(
    const Eigen::Vector6d& _error,
    Eigen::VectorXd& _grad)
{
  const math::Jacobian& J = this->mIK->computeJacobian();
  _grad = J.transpose() * _error;
  this->clampGradient(_grad);
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

  if(mEntity->getNumDependentGenCoords() == 0)
  {
    useDofs(mDofs);
    return;
  }

  useDofs(mEntity->getChainDofs());
}

//==============================================================================
void InverseKinematics::useWholeBody()
{
  useDofs(mEntity->getDependentGenCoordIndices());
}

//==============================================================================
void InverseKinematics::useDofs(const std::vector<size_t>& _dofs)
{
  mDofs = _dofs;
  const std::vector<size_t>& entityDependencies =
      mEntity->getDependentGenCoordIndices();

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
    std::shared_ptr<optimizer::Function> _objective)
{
  if(nullptr == _objective)
    _objective = optimizer::FunctionPtr(new optimizer::NullFunction);

  mObjective = _objective;
}

//==============================================================================
std::shared_ptr<optimizer::Function> InverseKinematics::getObjective()
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
    std::shared_ptr<optimizer::Function> _nsObjective)
{
  mUseNullSpace = true;
  if(nullptr == _nsObjective)
  {
    mUseNullSpace = false;
    _nsObjective = optimizer::FunctionPtr(new optimizer::NullFunction);
  }

  mNullSpaceObjective = _nsObjective;
}

//==============================================================================
std::shared_ptr<optimizer::Function> InverseKinematics::getNullSpaceObjective()
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
double InverseKinematics::evalObjective(
    const Eigen::VectorXd& _q)
{
  return mObjective->eval(_q) + mNullSpaceObjective->eval(_q);
}

//==============================================================================
void InverseKinematics::evalObjectiveGradient(
    const Eigen::VectorXd& _q, Eigen::Map<Eigen::VectorXd> _grad)
{
  mObjective->evalGradient(_q, _grad);

  if(mUseNullSpace)
  {
    mGradCache.resize(_grad.size());
    Eigen::Map<Eigen::VectorXd> grad_map(mGradCache.data(), _grad.size());
    mNullSpaceObjective->evalGradient(_q, grad_map);

    setConfiguration(_q);

    // Find a nullspace projection to apply to mGradCache
    computeJacobian();
    mSVDCache.compute(mJacobian, Eigen::ComputeFullV);
    math::extractNullSpace(mSVDCache, mNullspaceCache);
    _grad += mNullspaceCache*mNullspaceCache.transpose()*mGradCache;
  }
}

//==============================================================================
InverseKinematics::ErrorMethod* InverseKinematics::getErrorMethod()
{
  return mErrorMethod.get();
}

//==============================================================================
const InverseKinematics::ErrorMethod* InverseKinematics::getErrorMethod() const
{
  return mErrorMethod.get();
}

//==============================================================================
InverseKinematics::GradientMethod* InverseKinematics::getGradientMethod()
{
  return mGradientMethod.get();
}

//==============================================================================
const InverseKinematics::GradientMethod*
InverseKinematics::getGradientMethod() const
{
  return mGradientMethod.get();
}

//==============================================================================
std::shared_ptr<optimizer::Problem> InverseKinematics::getProblem()
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
  mOverallObjective->setCostFunction(
        [=](const Eigen::VectorXd& _q)
        { return this->evalObjective(_q); } );
  mOverallObjective->setGradientFunction(
        [=](const Eigen::VectorXd& _q,
            Eigen::Map<Eigen::VectorXd> _grad)
        { this->evalObjectiveGradient(_q, _grad); } );
  mOverallObjective->clearHessianFunction();

  mConstraint->setCostFunction(
        [=](const Eigen::VectorXd& _q)
        { return this->mErrorMethod->computeError(_q).norm(); } );
  mConstraint->setGradientFunction(
        [=](const Eigen::VectorXd& _q,
            Eigen::Map<Eigen::VectorXd> _grad)
        { this->mGradientMethod->computeGradient(_q, _grad); } );
  mConstraint->clearHessianFunction();

  mProblem->removeAllEqConstraints();
  mProblem->removeAllIneqConstraints();

  if(_clearSeeds)
    mProblem->clearAllSeeds();

  mProblem->setObjective(mOverallObjective);
  mProblem->addEqConstraint(mConstraint);

  mProblem->setDimension(mDofs.size());
}

//==============================================================================
void InverseKinematics::setSolver(std::shared_ptr<optimizer::Solver> _newSolver)
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
void InverseKinematics::setTarget(std::shared_ptr<SimpleFrame> _newTarget)
{
  if(nullptr == _newTarget)
  {
    _newTarget = SimpleFramePtr(
          new SimpleFrame(Frame::World(), mEntity->getName()+"_target",
                          mEntity->getWorldTransform()));
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
JacobianEntity* InverseKinematics::getEntity()
{
  return mEntity;
}

//==============================================================================
const JacobianEntity* InverseKinematics::getEntity() const
{
  return mEntity;
}

//==============================================================================
const math::Jacobian& InverseKinematics::computeJacobian() const
{
  const math::Jacobian& fullJacobian =
      this->getEntity()->getWorldJacobian();
  mJacobian.resize(6, this->getDofs().size());

  for(int i=0; i< static_cast<int>(this->getDofMap().size()); ++i)
  {
    int j = this->getDofMap()[i];
    if(j >= 0)
      mJacobian.block<6,1>(0,i) = fullJacobian.block<6,1>(0,j);
    else
      mJacobian.block<6,1>(0,i).setZero();
  }

  return mJacobian;
}

//==============================================================================
void InverseKinematics::setConfiguration(const Eigen::VectorXd& _q)
{
  if(_q.size() != static_cast<int>(mDofs.size()))
  {
    dterr << "[InverseKinematics::setConfiguration] Mismatch between config "
          << "size [" << _q.size() << "] and number of available degrees of "
          << "freedom [" << mDofs.size() << "]\n";
    return;
  }

  dart::dynamics::SkeletonPtr skel = this->getEntity()->getSkeleton();
  skel->setPositions(mDofs, _q);
}

//==============================================================================
void InverseKinematics::clearCaches()
{
  mErrorMethod->clearCache();
  mGradientMethod->clearCache();
}

//==============================================================================
void InverseKinematics::initialize()
{
  // Default to having no objectives
  setObjective(nullptr);
  setNullSpaceObjective(nullptr);

  mOverallObjective = std::shared_ptr<optimizer::ModularFunction>(
        new optimizer::ModularFunction(mEntity->getName()+"_objective"));

  mConstraint = std::shared_ptr<optimizer::ModularFunction>(
        new optimizer::ModularFunction(mEntity->getName()+"_constraint"));

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
  resetEntityConnection();

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
void InverseKinematics::resetEntityConnection()
{
  mEntityConnection.disconnect();
  mEntityConnection = mEntity->onTransformUpdated.connect(
        [=](const Entity*)
        { this->clearCaches(); } );
  clearCaches();
}

} // namespace dynamics
} // namespace dart
