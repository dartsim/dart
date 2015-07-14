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

#include "dart/dynamics/DegreeOfFreedom.h"
#include "dart/dynamics/HierarchicalIK.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/EndEffector.h"
#include "dart/dynamics/Skeleton.h"

namespace dart {
namespace dynamics {

//==============================================================================
bool HierarchicalIK::solve(bool _resetConfiguration)
{
  if(nullptr == mSolver)
  {
    dtwarn << "[HierarchicalIK::solve] The Solver for a HierarchicalIK module "
           << "associated with [" << mSkeleton.lock()->getName() << "] is a "
           << "nullptr. You must reset the module's Solver before you can use "
           << "it.\n";
    return false;
  }

  if(nullptr == mProblem)
  {
    dtwarn << "[HierarchicalIK::solve] The Problem for a HierarchicalIK module "
           << "associated with [" << mSkeleton.lock()->getName() << "] is a "
           << "nullptr. You must reset the module's Problem before you can use "
           << "it.\n";
    return false;
  }

  const SkeletonPtr& skel = getSkeleton();

  if(nullptr == skel)
  {
    dtwarn << "[HierarchicalIK::solve] Calling a HierarchicalIK module which "
           << "is associated with a Skeleton that no longer exists.\n";
    return false;
  }

  const size_t nDofs = skel->getNumDofs();
  mProblem->setDimension(nDofs);

  mProblem->setInitialGuess(skel->getPositions());

  Eigen::VectorXd bounds(nDofs);
  for(size_t i=0; i < nDofs; ++i)
    bounds[i] = skel->getDof(i)->getPositionLowerLimit();
  mProblem->setLowerBounds(bounds);

  for(size_t i=0; i < nDofs; ++i)
    bounds[i] = skel->getDof(i)->getPositionUpperLimit();
  mProblem->setUpperBounds(bounds);

  refreshIKHierarchy();

  if(!_resetConfiguration)
    return mSolver->solve();

  Eigen::VectorXd config = mSkeleton.lock()->getPositions();
  bool wasSolved = mSolver->solve();
  setConfiguration(config);
  return wasSolved;
}

//==============================================================================
bool HierarchicalIK::solve(Eigen::VectorXd& config, bool _resetConfiguration)
{
  bool wasSolved = solve(_resetConfiguration);
  config = mProblem->getOptimalSolution();
  return wasSolved;
}

//==============================================================================
void HierarchicalIK::setObjective(
    const std::shared_ptr<optimizer::Function>& _objective)
{
  mObjective = _objective;
}

//==============================================================================
const std::shared_ptr<optimizer::Function>& HierarchicalIK::getObjective()
{
  return mObjective;
}

//==============================================================================
std::shared_ptr<const optimizer::Function> HierarchicalIK::getObjective() const
{
  return mObjective;
}

//==============================================================================
void HierarchicalIK::setNullSpaceObjective(
    const std::shared_ptr<optimizer::Function>& _nsObjective)
{
  mNullSpaceObjective = _nsObjective;
}

//==============================================================================
const std::shared_ptr<optimizer::Function>&
HierarchicalIK::getNullSpaceObjective()
{
  return mNullSpaceObjective;
}

//==============================================================================
std::shared_ptr<const optimizer::Function>
HierarchicalIK::getNullSpaceObjective() const
{
  return mNullSpaceObjective;
}

//==============================================================================
bool HierarchicalIK::hasNullSpaceObjective() const
{
  return (nullptr != mNullSpaceObjective);
}

//==============================================================================
const std::shared_ptr<optimizer::Problem>& HierarchicalIK::getProblem()
{
  return mProblem;
}

//==============================================================================
std::shared_ptr<const optimizer::Problem> HierarchicalIK::getProblem() const
{
  return mProblem;
}

//==============================================================================
void HierarchicalIK::resetProblem(bool _clearSeeds)
{
  mProblem->removeAllEqConstraints();
  mProblem->removeAllIneqConstraints();

  if(_clearSeeds)
    mProblem->clearAllSeeds();

  mProblem->setObjective(std::make_shared<Objective>(this));
  mProblem->setObjective(std::make_shared<Constraint>(this));

  mProblem->setDimension(mSkeleton.lock()->getNumDofs());
}

//==============================================================================
void HierarchicalIK::setSolver(
    const std::shared_ptr<optimizer::Solver>& _newSolver)
{
  mSolver = _newSolver;
  if(nullptr == mSolver)
    return;

  mSolver->setProblem(mProblem);
}

//==============================================================================
const std::shared_ptr<optimizer::Solver>& HierarchicalIK::getSolver()
{
  return mSolver;
}

//==============================================================================
std::shared_ptr<const optimizer::Solver> HierarchicalIK::getSolver() const
{
  return mSolver;
}

//==============================================================================
const IKHierarchy& HierarchicalIK::getIKHierarchy() const
{
  return mHierarchy;
}

//==============================================================================
const std::vector<Eigen::MatrixXd>& HierarchicalIK::computeNullSpaces() const
{
  bool recompute = false;
  const ConstSkeletonPtr& skel = getSkeleton();
  const size_t nDofs = skel->getNumDofs();
  if(static_cast<size_t>(mLastConfig.size()) != nDofs)
  {
    recompute = true;
  }
  else
  {
    for(size_t i=0; i < nDofs; ++i)
    {
      if(mLastConfig[i] != skel->getDof(i)->getPosition())
      {
        recompute = true;
        break;
      }
    }
  }

  // TODO(MXG): When deciding whether we need to recompute, we should also check
  // the "version" of the Skeleton, as soon as the Skeleton versioning features
  // are available. The version should account for information about changes in
  // indexing and changes in Joint / BodyNode properties.

  if(!recompute)
    return mNullSpaceCache;

  const IKHierarchy& hierarchy = getIKHierarchy();

  mNullSpaceCache.resize(hierarchy.size());
  bool zeroedNullSpace = false;
  for(size_t i=0; i < hierarchy.size(); ++i)
  {
    const std::vector< std::shared_ptr<InverseKinematics> >& level =
        hierarchy[i];

    Eigen::MatrixXd& NS = mNullSpaceCache[i];
    if(i == 0)
    {
      // Start with an identity null space
      NS = Eigen::MatrixXd::Identity(nDofs, nDofs);
    }
    else if(zeroedNullSpace)
    {
      // If the null space has been zeroed out, just keep propogating the zeroes
      NS.setZero(nDofs, nDofs);
      continue;
    }
    else
    {
      // Otherwise, we will just build on the last level's null space
      NS = mNullSpaceCache[i-1];
    }

    mJacCache.resize(6, nDofs);
    for(size_t j=0; j < level.size(); ++j)
    {
      const std::shared_ptr<InverseKinematics>& ik = level[j];

      if(!ik->isActive())
        continue;

      const math::Jacobian& J = ik->computeJacobian();
      const std::vector<size_t>& dofs = ik->getDofs();

      mJacCache.setZero();
      for(size_t d=0; d < dofs.size(); ++d)
      {
        size_t k = dofs[d];
        mJacCache.block<6,1>(0,k) = J.block<6,1>(0,d);
      }

      mSVDCache.compute(mJacCache, Eigen::ComputeFullV);
      math::extractNullSpace(mSVDCache, mPartialNullspaceCache);

      if(mPartialNullspaceCache.rows() > 0 && mPartialNullspaceCache.cols() > 0)
      {
        NS *= mPartialNullspaceCache * mPartialNullspaceCache.transpose();
      }
      else
      {
        // There no longer exists a null space for this or any lower level
        NS.setZero();
        zeroedNullSpace = true;
        break;
      }
    }
  }

  return mNullSpaceCache;
}

//==============================================================================
void HierarchicalIK::setConfiguration(const Eigen::VectorXd& _q)
{
  const SkeletonPtr skel = mSkeleton.lock();
  if(skel)
    skel->setPositions(_q);
}

//==============================================================================
SkeletonPtr HierarchicalIK::getSkeleton()
{
  return getAffiliation();
}

//==============================================================================
ConstSkeletonPtr HierarchicalIK::getSkeleton() const
{
  return getAffiliation();
}

//==============================================================================
SkeletonPtr HierarchicalIK::getAffiliation()
{
  return mSkeleton.lock();
}

//==============================================================================
ConstSkeletonPtr HierarchicalIK::getAffiliation() const
{
  return mSkeleton.lock();
}

//==============================================================================
void HierarchicalIK::clearCaches()
{
  mLastConfig.resize(0);
}

//==============================================================================
HierarchicalIK::Objective::Objective(HierarchicalIK* _ik)
  : mIK(_ik)
{
  // Do nothing
}

//==============================================================================
optimizer::FunctionPtr HierarchicalIK::Objective::clone(
    HierarchicalIK* _newIK) const
{
  return std::make_shared<Objective>(_newIK);
}

//==============================================================================
double HierarchicalIK::Objective::eval(const Eigen::VectorXd& _x)
{
  if(nullptr == mIK)
  {
    dterr << "[HierarchicalIK::Objective::eval] Attempting to use an Objective "
          << "function of an expired HierarchicalIK module!\n";
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
void HierarchicalIK::Objective::evalGradient(
    const Eigen::VectorXd& _x, Eigen::Map<Eigen::VectorXd> _grad)
{
  if(nullptr == mIK)
  {
    dterr << "[HierarchicalIK::Objective::evalGradient] Attempting to use an "
          << "Objective function of an expired HierarchicalIK module!\n";
    assert(false);
    return;
  }

  if(mIK->mObjective)
    mIK->mObjective->evalGradient(_x, _grad);

  if(mIK->mNullSpaceObjective)
  {
    mGradCache.resize(_grad.size());
    Eigen::Map<Eigen::VectorXd> gradMap(mGradCache.data(), _grad.size());
    mIK->mNullSpaceObjective->evalGradient(_x, gradMap);

    mIK->setConfiguration(_x);

    const std::vector<Eigen::MatrixXd>& nullspaces = mIK->computeNullSpaces();
    if(nullspaces.size() > 0)
    {
      // Project through the deepest null space
      mGradCache = nullspaces.back() * mGradCache;
    }

    _grad += mGradCache;
  }
}

//==============================================================================
HierarchicalIK::Constraint::Constraint(HierarchicalIK* _ik)
  : mIK(_ik)
{
  // Do nothing
}

//==============================================================================
optimizer::FunctionPtr HierarchicalIK::Constraint::clone(
    HierarchicalIK* _newIK) const
{
  return std::make_shared<Constraint>(_newIK);
}

//==============================================================================
double HierarchicalIK::Constraint::eval(const Eigen::VectorXd& _x)
{
  if(nullptr == mIK)
  {
    dterr << "[HierarchicalIK::Constraint::eval] Attempting to use a "
          << "Constraint function of an expired HierarchicalIK module!\n";
    assert(false);
    return 0.0;
  }

  const IKHierarchy& hierarchy = mIK->getIKHierarchy();

  double cost = 0.0;
  for(size_t i=0; i < hierarchy.size(); ++i)
  {
    const std::vector< std::shared_ptr<InverseKinematics> >& level =
        hierarchy[i];

    for(size_t j=0; j < level.size(); ++j)
    {
      const std::shared_ptr<InverseKinematics>& ik = level[j];

      if(!ik->isActive())
        continue;

      const std::vector<size_t>& dofs = ik->getDofs();
      Eigen::VectorXd q(dofs.size());
      for(size_t k=0; k < dofs.size(); ++k)
        q[k] = _x[dofs[k]];

      InverseKinematics::ErrorMethod& method = ik->getErrorMethod();
      const Eigen::Vector6d& error = method.evalError(q);

      cost += error.dot(error);
    }
  }

  return std::sqrt(cost);
}

//==============================================================================
void HierarchicalIK::Constraint::evalGradient(
    const Eigen::VectorXd& _x, Eigen::Map<Eigen::VectorXd> _grad)
{
  const IKHierarchy& hierarchy = mIK->getIKHierarchy();
  const SkeletonPtr& skel = mIK->getSkeleton();
  const size_t nDofs = skel->getNumDofs();
  const std::vector<Eigen::MatrixXd>& nullspaces = mIK->computeNullSpaces();

  _grad.setZero();
  for(size_t i=0; i < hierarchy.size(); ++i)
  {
    const std::vector< std::shared_ptr<InverseKinematics> >& level =
        hierarchy[i];

    mLevelGradCache.setZero(nDofs);
    for(size_t j=0; j < level.size(); ++j)
    {
      const std::shared_ptr<InverseKinematics>& ik = level[j];

      if(!ik->isActive())
        continue;

      // Grab only the dependent coordinates from q
      const std::vector<size_t>& dofs = ik->getDofs();
      Eigen::VectorXd q(dofs.size());
      for(size_t k=0; k < dofs.size(); ++k)
        q[k] = _x[dofs[k]];

      // Compute the gradient of this specific error term
      mTempGradCache.setZero(dofs.size());
      Eigen::Map<Eigen::VectorXd> gradMap(mTempGradCache.data(),
                                          mTempGradCache.size());

      InverseKinematics::GradientMethod& method = ik->getGradientMethod();
      method.evalGradient(q, gradMap);

      // Add the components of this gradient into the gradient of this level
      for(size_t k=0; k < dofs.size(); ++k)
        mLevelGradCache[dofs[k]] += mTempGradCache[k];
    }

    // Project this level's gradient through the null spaces of the levels with
    // higher precedence, then add it to the overall gradient
    if(i > 0)
      _grad += nullspaces[i-1] * mLevelGradCache;
    else
      _grad += mLevelGradCache;
  }
}

//==============================================================================
HierarchicalIK::HierarchicalIK(const SkeletonPtr& _skeleton)
  : mSkeleton(_skeleton)
{
  initialize();
}

//==============================================================================
void HierarchicalIK::initialize()
{
  setObjective(nullptr);
  setNullSpaceObjective(nullptr);

  mProblem = std::make_shared<optimizer::Problem>();
  resetProblem();

  std::shared_ptr<optimizer::GradientDescentSolver> solver =
      std::make_shared<optimizer::GradientDescentSolver>(mProblem);
  solver->setStepSize(1.0);
  mSolver = solver;
}

//==============================================================================
std::shared_ptr<CompositeIK> CompositeIK::create(const SkeletonPtr& _skel)
{
  return std::shared_ptr<CompositeIK>(new CompositeIK(_skel));
}

//==============================================================================
std::shared_ptr<HierarchicalIK> CompositeIK::clone(
    const SkeletonPtr& _newSkel) const
{
  return cloneCompositeIK(_newSkel);
}

//==============================================================================
std::shared_ptr<CompositeIK> CompositeIK::cloneCompositeIK(
    const SkeletonPtr& _newSkel) const
{
  std::shared_ptr<CompositeIK> newComposite = create(_newSkel);

  for( const std::shared_ptr<InverseKinematics>& ik : mModuleSet )
  {
    JacobianNode* node = nullptr;
    JacobianNode* oldNode = ik->getNode();

    if(dynamic_cast<BodyNode*>(oldNode))
    {
      node = _newSkel->getBodyNode(oldNode->getName());
    }
    else if(dynamic_cast<EndEffector*>(oldNode))
    {
      node = _newSkel->getEndEffector(oldNode->getName());
    }

    if(node)
    {
      newComposite->addModule(ik->clone(node));
    }
  }

  return newComposite;
}

//==============================================================================
bool CompositeIK::addModule(const std::shared_ptr<InverseKinematics>& _ik)
{
  if(_ik->getNode()->getSkeleton() != mSkeleton.lock())
    return false; // Should we print a warning message here, or is the return
                  // value sufficient?

  ModuleSet::iterator it = mModuleSet.find(_ik);

  // We already have this module
  if(it != mModuleSet.end())
    return true;

  mModuleSet.insert(_ik);

  return true;
}

//==============================================================================
const CompositeIK::ModuleSet& CompositeIK::getModuleSet()
{
  return mModuleSet;
}

//==============================================================================
CompositeIK::ConstModuleSet CompositeIK::getModuleSet() const
{
  ConstModuleSet modules;
  for(const std::shared_ptr<InverseKinematics>& module : mModuleSet)
    modules.insert(module);

  return modules;
}

//==============================================================================
void CompositeIK::refreshIKHierarchy()
{
  if(mModuleSet.size() == 0)
  {
    mHierarchy.clear();
    return;
  }

  int highestLevel = -1;
  for(const std::shared_ptr<InverseKinematics>& module : mModuleSet)
  {
    highestLevel = std::max(static_cast<int>(module->getHierarchyLevel()),
                            highestLevel);
  }

  assert(highestLevel >= 0);

  mHierarchy.resize(highestLevel+1);
  for(auto& level : mHierarchy)
    level.clear();

  for(const std::shared_ptr<InverseKinematics>& module : mModuleSet)
    mHierarchy[module->getHierarchyLevel()].push_back(module);
}

//==============================================================================
CompositeIK::CompositeIK(const SkeletonPtr& _skel)
  : HierarchicalIK(_skel)
{
  // Do nothing
}

//==============================================================================
std::shared_ptr<WholeBodyIK> WholeBodyIK::create(const SkeletonPtr& _skel)
{
  return std::shared_ptr<WholeBodyIK>(new WholeBodyIK(_skel));
}

//==============================================================================
std::shared_ptr<HierarchicalIK> WholeBodyIK::clone(
    const SkeletonPtr& _newSkel) const
{
  return cloneWholeBodyIK(_newSkel);
}

//==============================================================================
std::shared_ptr<WholeBodyIK> WholeBodyIK::cloneWholeBodyIK(
    const SkeletonPtr& _newSkel) const
{
  return create(_newSkel);
}

//==============================================================================
void WholeBodyIK::refreshIKHierarchy()
{
  const SkeletonPtr& skel = mSkeleton.lock();

  // TODO(MXG): Consider giving Skeletons a list of all the JacobianNodes that
  // they contain, so that we can make this extensible to user-defined
  // JacobianNode types, and also make the code more DRY.

  int highestLevel = -1;
  for(size_t i=0; i < skel->getNumBodyNodes(); ++i)
  {
    BodyNode* bn = skel->getBodyNode(i);
    const std::shared_ptr<InverseKinematics>& ik = bn->getIK();

    if(ik)
    {
      highestLevel = std::max(static_cast<int>(ik->getHierarchyLevel()),
                              highestLevel);
    }
  }

  for(size_t i=0; i < skel->getNumEndEffectors(); ++i)
  {
    EndEffector* ee = skel->getEndEffector(i);
    const std::shared_ptr<InverseKinematics>& ik = ee->getIK();

    if(ik)
    {
      highestLevel = std::max(static_cast<int>(ik->getHierarchyLevel()),
                              highestLevel);
    }
  }

  if(-1 == highestLevel)
  {
    // There were no IK modules present in this Skeleton
    mHierarchy.clear();
    return;
  }

  mHierarchy.resize(highestLevel+1);
  for(auto& level : mHierarchy)
    level.clear();

  for(size_t i=0; i < skel->getNumBodyNodes(); ++i)
  {
    BodyNode* bn = skel->getBodyNode(i);
    const std::shared_ptr<InverseKinematics>& ik = bn->getIK();

    if(ik)
      mHierarchy[ik->getHierarchyLevel()].push_back(ik);
  }

  for(size_t i=0; i < skel->getNumEndEffectors(); ++i)
  {
    EndEffector* ee = skel->getEndEffector(i);
    const std::shared_ptr<InverseKinematics>& ik = ee->getIK();

    if(ik)
      mHierarchy[ik->getHierarchyLevel()].push_back(ik);
  }
}

//==============================================================================
WholeBodyIK::WholeBodyIK(const SkeletonPtr& _skel)
  : HierarchicalIK(_skel)
{
  // Do nothing
}

} // namespace dynamics
} // namespace dart

