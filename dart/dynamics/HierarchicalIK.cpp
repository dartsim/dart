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

#include "dart/dynamics/DegreeOfFreedom.hpp"
#include "dart/dynamics/HierarchicalIK.hpp"
#include "dart/dynamics/BodyNode.hpp"
#include "dart/dynamics/EndEffector.hpp"
#include "dart/dynamics/Skeleton.hpp"
#include "dart/optimizer/GradientDescentSolver.hpp"

namespace dart {
namespace dynamics {

//==============================================================================
bool HierarchicalIK::solve(bool _applySolution)
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

  const std::size_t nDofs = skel->getNumDofs();
  mProblem->setDimension(nDofs);

  mProblem->setInitialGuess(skel->getPositions());

  Eigen::VectorXd bounds(nDofs);
  for(std::size_t i=0; i < nDofs; ++i)
    bounds[i] = skel->getDof(i)->getPositionLowerLimit();
  mProblem->setLowerBounds(bounds);

  for(std::size_t i=0; i < nDofs; ++i)
    bounds[i] = skel->getDof(i)->getPositionUpperLimit();
  mProblem->setUpperBounds(bounds);

  refreshIKHierarchy();

  if(_applySolution)
  {
    bool wasSolved = mSolver->solve();
    setPositions(mProblem->getOptimalSolution());
    return wasSolved;
  }

  Eigen::VectorXd originalPositions = skel->getPositions();
  bool wasSolved = mSolver->solve();
  setPositions(originalPositions);
  return wasSolved;
}

//==============================================================================
bool HierarchicalIK::solve(Eigen::VectorXd& positions, bool _applySolution)
{
  bool wasSolved = solve(_applySolution);
  positions = mProblem->getOptimalSolution();
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

  mProblem->setObjective(std::make_shared<Objective>(mPtr.lock()));
  mProblem->addEqConstraint(std::make_shared<Constraint>(mPtr.lock()));

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
  const std::size_t nDofs = skel->getNumDofs();
  if(static_cast<std::size_t>(mLastPositions.size()) != nDofs)
  {
    recompute = true;
  }
  else
  {
    for(std::size_t i=0; i < nDofs; ++i)
    {
      if(mLastPositions[i] != skel->getDof(i)->getPosition())
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
  for(std::size_t i=0; i < hierarchy.size(); ++i)
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
    for(std::size_t j=0; j < level.size(); ++j)
    {
      const std::shared_ptr<InverseKinematics>& ik = level[j];

      if(!ik->isActive())
        continue;

      const math::Jacobian& J = ik->computeJacobian();
      const std::vector<std::size_t>& dofs = ik->getDofs();

      mJacCache.setZero();
      for(std::size_t d=0; d < dofs.size(); ++d)
      {
        std::size_t k = dofs[d];
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
Eigen::VectorXd HierarchicalIK::getPositions() const
{
  const SkeletonPtr& skel = mSkeleton.lock();
  if(skel)
    return skel->getPositions();

  return Eigen::VectorXd();
}

//==============================================================================
void HierarchicalIK::setPositions(const Eigen::VectorXd& _q)
{
  const SkeletonPtr& skel = mSkeleton.lock();
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
  mLastPositions.resize(0);
}

//==============================================================================
HierarchicalIK::Objective::Objective(const std::shared_ptr<HierarchicalIK>& _ik)
  : mIK(_ik)
{
  // Do nothing
}

//==============================================================================
optimizer::FunctionPtr HierarchicalIK::Objective::clone(
    const std::shared_ptr<HierarchicalIK>& _newIK) const
{
  return std::make_shared<Objective>(_newIK);
}

//==============================================================================
double HierarchicalIK::Objective::eval(const Eigen::VectorXd& _x)
{
  const std::shared_ptr<HierarchicalIK>& hik = mIK.lock();

  if(nullptr == hik)
  {
    dterr << "[HierarchicalIK::Objective::eval] Attempting to use an Objective "
          << "function of an expired HierarchicalIK module!\n";
    assert(false);
    return 0;
  }

  double cost = 0.0;

  if(hik->mObjective)
    cost += hik->mObjective->eval(_x);

  if(hik->mNullSpaceObjective)
    cost += hik->mNullSpaceObjective->eval(_x);

  return cost;
}

//==============================================================================
void HierarchicalIK::Objective::evalGradient(
    const Eigen::VectorXd& _x, Eigen::Map<Eigen::VectorXd> _grad)
{
  const std::shared_ptr<HierarchicalIK>& hik = mIK.lock();

  if(nullptr == hik)
  {
    dterr << "[HierarchicalIK::Objective::evalGradient] Attempting to use an "
          << "Objective function of an expired HierarchicalIK module!\n";
    assert(false);
    return;
  }

  if(hik->mObjective)
    hik->mObjective->evalGradient(_x, _grad);
  else
    _grad.setZero();

  if(hik->mNullSpaceObjective)
  {
    mGradCache.resize(_grad.size());
    Eigen::Map<Eigen::VectorXd> gradMap(mGradCache.data(), _grad.size());
    hik->mNullSpaceObjective->evalGradient(_x, gradMap);

    hik->setPositions(_x);

    const std::vector<Eigen::MatrixXd>& nullspaces = hik->computeNullSpaces();
    if(nullspaces.size() > 0)
    {
      // Project through the deepest null space
      mGradCache = nullspaces.back() * mGradCache;
    }

    _grad += mGradCache;
  }
}

//==============================================================================
HierarchicalIK::Constraint::Constraint(const std::shared_ptr<HierarchicalIK>& _ik)
  : mIK(_ik)
{
  // Do nothing
}

//==============================================================================
optimizer::FunctionPtr HierarchicalIK::Constraint::clone(const std::shared_ptr<HierarchicalIK>& _newIK) const
{
  return std::make_shared<Constraint>(_newIK);
}

//==============================================================================
double HierarchicalIK::Constraint::eval(const Eigen::VectorXd& _x)
{
  const std::shared_ptr<HierarchicalIK>& hik = mIK.lock();
  if(nullptr == hik)
  {
    dterr << "[HierarchicalIK::Constraint::eval] Attempting to use a "
          << "Constraint function of an expired HierarchicalIK module!\n";
    assert(false);
    return 0.0;
  }

  const IKHierarchy& hierarchy = hik->getIKHierarchy();

  double cost = 0.0;
  for(std::size_t i=0; i < hierarchy.size(); ++i)
  {
    const std::vector< std::shared_ptr<InverseKinematics> >& level =
        hierarchy[i];

    for(std::size_t j=0; j < level.size(); ++j)
    {
      const std::shared_ptr<InverseKinematics>& ik = level[j];

      if(!ik->isActive())
        continue;

      const std::vector<std::size_t>& dofs = ik->getDofs();
      Eigen::VectorXd q(dofs.size());
      for(std::size_t k=0; k < dofs.size(); ++k)
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
  const std::shared_ptr<HierarchicalIK>& hik = mIK.lock();

  const IKHierarchy& hierarchy = hik->getIKHierarchy();
  const SkeletonPtr& skel = hik->getSkeleton();
  const std::size_t nDofs = skel->getNumDofs();
  const std::vector<Eigen::MatrixXd>& nullspaces = hik->computeNullSpaces();

  _grad.setZero();
  for(std::size_t i=0; i < hierarchy.size(); ++i)
  {
    const std::vector< std::shared_ptr<InverseKinematics> >& level =
        hierarchy[i];

    mLevelGradCache.setZero(nDofs);
    for(std::size_t j=0; j < level.size(); ++j)
    {
      const std::shared_ptr<InverseKinematics>& ik = level[j];

      if(!ik->isActive())
        continue;

      // Grab only the dependent coordinates from q
      const std::vector<std::size_t>& dofs = ik->getDofs();
      Eigen::VectorXd q(dofs.size());
      for(std::size_t k=0; k < dofs.size(); ++k)
        q[k] = _x[dofs[k]];

      // Compute the gradient of this specific error term
      mTempGradCache.setZero(dofs.size());
      Eigen::Map<Eigen::VectorXd> gradMap(mTempGradCache.data(),
                                          mTempGradCache.size());

      InverseKinematics::GradientMethod& method = ik->getGradientMethod();
      method.evalGradient(q, gradMap);

      // Add the components of this gradient into the gradient of this level
      for(std::size_t k=0; k < dofs.size(); ++k)
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
  // initialize MUST be called immediately after the construction of any
  // directly inheriting classes.
}

//==============================================================================
void HierarchicalIK::initialize(const std::shared_ptr<HierarchicalIK> my_ptr)
{
  mPtr = my_ptr;

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
static std::shared_ptr<optimizer::Function> cloneIkFunc(
    const std::shared_ptr<optimizer::Function>& _function,
    const std::shared_ptr<HierarchicalIK>& _ik)
{
  std::shared_ptr<HierarchicalIK::Function> ikFunc =
      std::dynamic_pointer_cast<HierarchicalIK::Function>(_function);

  if(ikFunc)
    return ikFunc->clone(_ik);

  return _function;
}

//==============================================================================
void HierarchicalIK::copyOverSetup(
    const std::shared_ptr<HierarchicalIK>& _otherIK) const
{
  _otherIK->setSolver(mSolver->clone());

  const std::shared_ptr<optimizer::Problem>& newProblem =
      _otherIK->getProblem();
  newProblem->setObjective( cloneIkFunc(mProblem->getObjective(), _otherIK) );

  newProblem->removeAllEqConstraints();
  for(std::size_t i=0; i < mProblem->getNumEqConstraints(); ++i)
    newProblem->addEqConstraint(
          cloneIkFunc(mProblem->getEqConstraint(i), _otherIK));

  newProblem->removeAllIneqConstraints();
  for(std::size_t i=0; i < mProblem->getNumIneqConstraints(); ++i)
    newProblem->addIneqConstraint(
          cloneIkFunc(mProblem->getIneqConstraint(i), _otherIK));

  newProblem->getSeeds() = mProblem->getSeeds();
}

//==============================================================================
std::shared_ptr<CompositeIK> CompositeIK::create(const SkeletonPtr& _skel)
{
  std::shared_ptr<CompositeIK> ik(new CompositeIK(_skel));
  ik->initialize(ik);
  return ik;
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
  copyOverSetup(newComposite);

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
  std::shared_ptr<WholeBodyIK> ik(new WholeBodyIK(_skel));
  ik->initialize(ik);
  return ik;
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
  std::shared_ptr<WholeBodyIK> newIK = create(_newSkel);
  copyOverSetup(newIK);
  return newIK;
}

//==============================================================================
void WholeBodyIK::refreshIKHierarchy()
{
  const SkeletonPtr& skel = mSkeleton.lock();

  // TODO(MXG): Consider giving Skeletons a list of all the JacobianNodes that
  // they contain, so that we can make this extensible to user-defined
  // JacobianNode types, and also make the code more DRY.

  int highestLevel = -1;
  for(std::size_t i=0; i < skel->getNumBodyNodes(); ++i)
  {
    BodyNode* bn = skel->getBodyNode(i);
    const std::shared_ptr<InverseKinematics>& ik = bn->getIK();

    if(ik)
    {
      highestLevel = std::max(static_cast<int>(ik->getHierarchyLevel()),
                              highestLevel);
    }
  }

  for(std::size_t i=0; i < skel->getNumEndEffectors(); ++i)
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

  for(std::size_t i=0; i < skel->getNumBodyNodes(); ++i)
  {
    BodyNode* bn = skel->getBodyNode(i);
    const std::shared_ptr<InverseKinematics>& ik = bn->getIK();

    if(ik)
      mHierarchy[ik->getHierarchyLevel()].push_back(ik);
  }

  for(std::size_t i=0; i < skel->getNumEndEffectors(); ++i)
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

