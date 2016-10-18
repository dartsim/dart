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

#include "dart/dynamics/ReferentialSkeleton.hpp"

#include "dart/common/Deprecated.hpp"
#include "dart/dynamics/BodyNode.hpp"
#include "dart/dynamics/SoftBodyNode.hpp"
#include "dart/dynamics/Joint.hpp"
#include "dart/dynamics/DegreeOfFreedom.hpp"

namespace dart {
namespace dynamics {

//==============================================================================
const std::string& ReferentialSkeleton::setName(const std::string& _name)
{
  const std::string oldName = mName;
  mName = _name;

  const MetaSkeletonPtr& me = mPtr.lock();
  mNameChangedSignal.raise(me, oldName, mName);

  return mName;
}

//==============================================================================
const std::string& ReferentialSkeleton::getName() const
{
  return mName;
}

//==============================================================================
std::size_t ReferentialSkeleton::getNumBodyNodes() const
{
  return mBodyNodes.size();
}

//==============================================================================
BodyNode* ReferentialSkeleton::getBodyNode(std::size_t _idx)
{
  return common::getVectorObjectIfAvailable<BodyNodePtr>(_idx, mBodyNodes);
}

//==============================================================================
const BodyNode* ReferentialSkeleton::getBodyNode(std::size_t _idx) const
{
  return common::getVectorObjectIfAvailable<BodyNodePtr>(_idx, mBodyNodes);
}

//==============================================================================
template <class T1, class T2>
static std::vector<T2>& convertVector(const std::vector<T1>& t1_vec,
                                      std::vector<T2>& t2_vec)
{
  t2_vec.resize(t1_vec.size());
  for(std::size_t i = 0; i < t1_vec.size(); ++i)
    t2_vec[i] = t1_vec[i];
  return t2_vec;
}

//==============================================================================
const std::vector<BodyNode*>& ReferentialSkeleton::getBodyNodes()
{
  // TODO(MXG): This might not be necessary, since there should never be a
  // discrepancy between the raw BodyNodes and the BodyNodePtrs
  return convertVector<BodyNodePtr, BodyNode*>(
        mBodyNodes, mRawBodyNodes);
}

//==============================================================================
const std::vector<const BodyNode*>& ReferentialSkeleton::getBodyNodes() const
{
  return convertVector<BodyNodePtr, const BodyNode*>(
        mBodyNodes, mRawConstBodyNodes);
}

//==============================================================================
std::size_t ReferentialSkeleton::getIndexOf(const BodyNode* _bn, bool _warning) const
{
  if(nullptr == _bn)
  {
    if(_warning)
    {
      dterr << "[ReferentialSkeleton::getIndexOf] Requesting index of a "
            << "nullptr BodyNode!\n";
      assert(false);
    }
    return INVALID_INDEX;
  }

  std::unordered_map<const BodyNode*, IndexMap>::const_iterator it =
      mIndexMap.find(_bn);
  if( it == mIndexMap.end() )
  {
    if(_warning)
    {
      dterr << "[ReferentialSkeleton::getIndexOf] Requesting index of a "
            << "BodyNode [" << _bn->getName() << "] (" << _bn << ") that is "
            << "not in this ReferentialSkeleton [" << getName() << "] ("
            << this << ").\n";
      assert(false);
    }
    return INVALID_INDEX;
  }

  return it->second.mBodyNodeIndex;
}

//==============================================================================
std::size_t ReferentialSkeleton::getNumJoints() const
{
  return mJoints.size();
}

//==============================================================================
Joint* ReferentialSkeleton::getJoint(std::size_t _idx)
{
  return common::getVectorObjectIfAvailable<JointPtr>(_idx, mJoints);
}

//==============================================================================
const Joint* ReferentialSkeleton::getJoint(std::size_t _idx) const
{
  return common::getVectorObjectIfAvailable<JointPtr>(_idx, mJoints);
}

//==============================================================================
std::size_t ReferentialSkeleton::getIndexOf(const Joint* _joint, bool _warning) const
{
  if(nullptr == _joint)
  {
    if(_warning)
    {
      dterr << "[ReferentialSkeleton::getIndexOf] Requesting index of a nullptr "
            << "Joint!\n";
      assert(false);
    }
    return INVALID_INDEX;
  }

  std::unordered_map<const BodyNode*, IndexMap>::const_iterator it =
      mIndexMap.find(_joint->getChildBodyNode());
  if( it == mIndexMap.end() )
  {
    if(_warning)
    {
      dterr << "[ReferentialSkeleton::getIndexOf] Requesting index of a Joint ["
            << _joint->getName() << "] (" << _joint << ") that is not in this "
            << "ReferentialSkeleton [" << getName() << "] (" << this << ").\n";
      assert(false);
    }
    return INVALID_INDEX;
  }

  return it->second.mJointIndex;
}

//==============================================================================
std::size_t ReferentialSkeleton::getNumDofs() const
{
  return mDofs.size();
}

//==============================================================================
DegreeOfFreedom* ReferentialSkeleton::getDof(std::size_t _idx)
{
  return common::getVectorObjectIfAvailable<DegreeOfFreedomPtr>(_idx, mDofs);
}

//==============================================================================
const DegreeOfFreedom* ReferentialSkeleton::getDof(std::size_t _idx) const
{
  return common::getVectorObjectIfAvailable<DegreeOfFreedomPtr>(_idx, mDofs);
}

//==============================================================================
const std::vector<DegreeOfFreedom*>& ReferentialSkeleton::getDofs()
{
  // We want to refill the raw DegreeOfFreedom vector, because the pointers will
  // change any time a BodyNode's parent Joint gets changed, and we have no way
  // of knowing when that might happen.
  return convertVector<DegreeOfFreedomPtr, DegreeOfFreedom*>(
        mDofs, mRawDofs);
}

//==============================================================================
std::vector<const DegreeOfFreedom*> ReferentialSkeleton::getDofs() const
{
  // We want to refill the raw DegreeOfFreedom vector, because the pointers will
  // change any time a BodyNode's parent Joint gets changed, and we have no way
  // of knowing when that might happen.
  return convertVector<DegreeOfFreedomPtr, const DegreeOfFreedom*>(
        mDofs, mRawConstDofs);
}

//==============================================================================
std::size_t ReferentialSkeleton::getIndexOf(
    const DegreeOfFreedom* _dof, bool _warning) const
{
  if(nullptr == _dof)
  {
    if(_warning)
    {
      dterr << "[ReferentialSkeleton::getIndexOf] Requesting index of a "
            << "nullptr DegreeOfFreedom!\n";
      assert(false);
    }
    return INVALID_INDEX;
  }

  const BodyNode* bn = _dof->getChildBodyNode();
  std::unordered_map<const BodyNode*, IndexMap>::const_iterator it =
      mIndexMap.find(bn);
  if( it == mIndexMap.end() )
  {
    if(_warning)
    {
      dterr << "[ReferentialSkeleton::getIndexOf] Requesting index of a "
            << "DegreeOfFreedom [" << _dof->getName() << "] (" << _dof
            << ") that is not in this ReferentialSkeleton [" << getName()
            << "] (" << this << ").\n";
      assert(false);
    }
    return INVALID_INDEX;
  }

  std::size_t localIndex = _dof->getIndexInJoint();
  if(it->second.mDofIndices.size() <= localIndex ||
     it->second.mDofIndices[localIndex] == INVALID_INDEX )
  {
    if(_warning)
    {
      dterr << "[ReferentialSkeleton::getIndexOf] BodyNode named ["
            << bn->getName() << "] (" << bn << ") is referenced by the "
            << "ReferentialSkeleton named [" << getName() << "] (" << this
            << "), but it does not include the DegreeOfFreedom #"
            << localIndex << " of its parent Joint!\n";
      assert(false);
    }
    return INVALID_INDEX;
  }

  return it->second.mDofIndices[localIndex];
}

//==============================================================================
static bool isValidBodyNode(const ReferentialSkeleton* /*_refSkel*/,
                            const JacobianNode* _node,
                            const std::string& _fname)
{
  if(nullptr == _node)
  {
    dtwarn << "[ReferentialSkeleton::" << _fname << "] Invalid BodyNode "
           << "pointer: nullptr. Returning zero Jacobian.\n";
    assert(false);
    return false;
  }

  // We should not test whether the BodyNode is in this ReferentialSkeleton,
  // because even if it is not in this ReferentialSkeleton, it might have
  // dependent degrees of freedom which *are* in this ReferentialSkeleton.

  return true;
}

//==============================================================================
template <typename JacobianType>
void assignJacobian(JacobianType& _J,
                    const ReferentialSkeleton* _refSkel,
                    const JacobianNode* _node,
                    const JacobianType& _JBodyNode)
{
  const std::vector<const DegreeOfFreedom*>& bn_dofs =
      _node->getDependentDofs();
  std::size_t nDofs = bn_dofs.size();
  for(std::size_t i=0; i<nDofs; ++i)
  {
    std::size_t refIndex = _refSkel->getIndexOf(bn_dofs[i], false);
    if(INVALID_INDEX == refIndex)
      continue;

    _J.col(refIndex) = _JBodyNode.col(i);
  }
}

//==============================================================================
template <typename ...Args>
math::Jacobian variadicGetJacobian(
    const ReferentialSkeleton* _refSkel,
    const JacobianNode* _node,
    Args... args)
{
  math::Jacobian J = math::Jacobian::Zero(6, _refSkel->getNumDofs());

  if( !isValidBodyNode(_refSkel, _node, "getJacobian") )
    return J;

  const math::Jacobian JBodyNode = _node->getJacobian(args...);

  assignJacobian(J, _refSkel, _node, JBodyNode);

  return J;
}

//==============================================================================
math::Jacobian ReferentialSkeleton::getJacobian(
    const JacobianNode* _node) const
{
  return variadicGetJacobian(this, _node);
}

//==============================================================================
math::Jacobian ReferentialSkeleton::getJacobian(
    const JacobianNode* _node, const Frame* _inCoordinatesOf) const
{
  return variadicGetJacobian(this, _node, _inCoordinatesOf);
}

//==============================================================================
math::Jacobian ReferentialSkeleton::getJacobian(
    const JacobianNode* _node, const Eigen::Vector3d& _localOffset) const
{
  return variadicGetJacobian(this, _node, _localOffset);
}

//==============================================================================
math::Jacobian ReferentialSkeleton::getJacobian(const JacobianNode* _node,
    const Eigen::Vector3d& _localOffset,
    const Frame* _inCoordinatesOf) const
{
  return variadicGetJacobian(this, _node, _localOffset, _inCoordinatesOf);
}

//==============================================================================
template <typename ...Args>
math::Jacobian variadicGetWorldJacobian(const ReferentialSkeleton* _refSkel,
                                        const JacobianNode* _node, Args... args)
{
  math::Jacobian J = math::Jacobian::Zero(6, _refSkel->getNumDofs());

  if( !isValidBodyNode(_refSkel, _node, "getWorldJacobian") )
    return J;

  const math::Jacobian JBodyNode = _node->getWorldJacobian(args...);

  assignJacobian(J, _refSkel, _node, JBodyNode);

  return J;
}

//==============================================================================
math::Jacobian ReferentialSkeleton::getWorldJacobian(
    const JacobianNode* _node) const
{
  return variadicGetWorldJacobian(this, _node);
}

//==============================================================================
math::Jacobian ReferentialSkeleton::getWorldJacobian(
    const JacobianNode* _node, const Eigen::Vector3d& _localOffset) const
{
  return variadicGetWorldJacobian(this, _node, _localOffset);
}

//==============================================================================
template <typename ...Args>
math::LinearJacobian variadicGetLinearJacobian(
    const ReferentialSkeleton* _refSkel,
    const JacobianNode* _node, Args... args)
{
  math::LinearJacobian J =
      math::LinearJacobian::Zero(3, _refSkel->getNumDofs());

  if( !isValidBodyNode(_refSkel, _node, "getLinearJacobian") )
    return J;

  const math::LinearJacobian JBodyNode = _node->getLinearJacobian(args...);

  assignJacobian(J, _refSkel, _node, JBodyNode);

  return J;
}

//==============================================================================
math::LinearJacobian ReferentialSkeleton::getLinearJacobian(
    const JacobianNode* _node, const Frame* _inCoordinatesOf) const
{
  return variadicGetLinearJacobian(this, _node, _inCoordinatesOf);
}

//==============================================================================
math::LinearJacobian ReferentialSkeleton::getLinearJacobian(
    const JacobianNode* _node, const Eigen::Vector3d& _localOffset,
    const Frame* _inCoordinatesOf) const
{
  return variadicGetLinearJacobian(this, _node, _localOffset,
                                   _inCoordinatesOf);
}

//==============================================================================
template <typename ...Args>
math::AngularJacobian variadicGetAngularJacobian(
    const ReferentialSkeleton* _refSkel,
    const JacobianNode* _node, Args... args)
{
  math::AngularJacobian J =
      math::AngularJacobian::Zero(3, _refSkel->getNumDofs());

  if( !isValidBodyNode(_refSkel, _node, "getAngularJacobian") )
    return J;

  const math::AngularJacobian JBodyNode =
      _node->getAngularJacobian(args...);

  assignJacobian(J, _refSkel, _node, JBodyNode);

  return J;
}

//==============================================================================
math::AngularJacobian ReferentialSkeleton::getAngularJacobian(
    const JacobianNode* _node, const Frame* _inCoordinatesOf) const
{
  return variadicGetAngularJacobian(this, _node, _inCoordinatesOf);
}

//==============================================================================
template <typename ...Args>
math::Jacobian variadicGetJacobianSpatialDeriv(
    const ReferentialSkeleton* _refSkel,
    const JacobianNode* _node, Args... args)
{
  math::Jacobian dJ = math::Jacobian::Zero(6, _refSkel->getNumDofs());

  if( !isValidBodyNode(_refSkel, _node, "getJacobianSpatialDeriv") )
    return dJ;

  const math::Jacobian dJBodyNode = _node->getJacobianSpatialDeriv(args...);

  assignJacobian(dJ, _refSkel, _node, dJBodyNode);

  return dJ;
}

//==============================================================================
math::Jacobian ReferentialSkeleton::getJacobianSpatialDeriv(
    const JacobianNode* _node) const
{
  return variadicGetJacobianSpatialDeriv(this, _node);
}

//==============================================================================
math::Jacobian ReferentialSkeleton::getJacobianSpatialDeriv(
    const JacobianNode* _node, const Frame* _inCoordinatesOf) const
{
  return variadicGetJacobianSpatialDeriv(this, _node, _inCoordinatesOf);
}

//==============================================================================
math::Jacobian ReferentialSkeleton::getJacobianSpatialDeriv(
    const JacobianNode* _node, const Eigen::Vector3d& _localOffset) const
{
  return variadicGetJacobianSpatialDeriv(this, _node, _localOffset);
}

//==============================================================================
math::Jacobian ReferentialSkeleton::getJacobianSpatialDeriv(
    const JacobianNode* _node, const Eigen::Vector3d& _localOffset,
    const Frame* _inCoordinatesOf) const
{
  return variadicGetJacobianSpatialDeriv(this, _node, _localOffset,
                                         _inCoordinatesOf);
}

//==============================================================================
template <typename ...Args>
math::Jacobian variadicGetJacobianClassicDeriv(
    const ReferentialSkeleton* _refSkel,
    const JacobianNode* _node, Args... args)
{
  math::Jacobian dJ = math::Jacobian::Zero(6, _refSkel->getNumDofs());

  if( !isValidBodyNode(_refSkel, _node, "getJacobianClassicDeriv") )
    return dJ;

  const math::Jacobian dJBodyNode = _node->getJacobianClassicDeriv(args...);

  assignJacobian(dJ, _refSkel, _node, dJBodyNode);

  return dJ;
}

//==============================================================================
math::Jacobian ReferentialSkeleton::getJacobianClassicDeriv(
    const JacobianNode* _node) const
{
  return variadicGetJacobianClassicDeriv(this, _node);
}

//==============================================================================
math::Jacobian ReferentialSkeleton::getJacobianClassicDeriv(
    const JacobianNode* _node, const Frame* _inCoordinatesOf) const
{
  return variadicGetJacobianClassicDeriv(this, _node, _inCoordinatesOf);
}

//==============================================================================
math::Jacobian ReferentialSkeleton::getJacobianClassicDeriv(
    const JacobianNode* _node, const Eigen::Vector3d& _localOffset,
    const Frame* _inCoordinatesOf) const
{
  return variadicGetJacobianClassicDeriv(this, _node, _localOffset,
                                         _inCoordinatesOf);
}

//==============================================================================
template <typename ...Args>
math::LinearJacobian variadicGetLinearJacobianDeriv(
    const ReferentialSkeleton* _refSkel,
    const JacobianNode* _node, Args... args)
{
  math::LinearJacobian dJv =
      math::LinearJacobian::Zero(3, _refSkel->getNumDofs());

  if( !isValidBodyNode(_refSkel, _node, "getLinearJacobianDeriv") )
    return dJv;

  const math::LinearJacobian dJvBodyNode =
      _node->getLinearJacobianDeriv(args...);

  assignJacobian(dJv, _refSkel, _node, dJvBodyNode);

  return dJv;
}

//==============================================================================
math::LinearJacobian ReferentialSkeleton::getLinearJacobianDeriv(
    const JacobianNode* _node, const Frame* _inCoordinatesOf) const
{
  return variadicGetLinearJacobianDeriv(this, _node, _inCoordinatesOf);
}

//==============================================================================
math::LinearJacobian ReferentialSkeleton::getLinearJacobianDeriv(
    const JacobianNode* _node, const Eigen::Vector3d& _localOffset,
    const Frame* _inCoordinatesOf) const
{
  return variadicGetLinearJacobianDeriv(this, _node, _localOffset,
                                        _inCoordinatesOf);
}

//==============================================================================
template <typename ...Args>
math::AngularJacobian variadicGetAngularJacobianDeriv(
    const ReferentialSkeleton* _refSkel,
    const JacobianNode* _node, Args... args)
{
  math::AngularJacobian dJw =
      math::AngularJacobian::Zero(3, _refSkel->getNumDofs());

  if( !isValidBodyNode(_refSkel, _node, "getAngularJacobianDeriv") )
    return dJw;

  const math::AngularJacobian dJwBodyNode =
      _node->getAngularJacobianDeriv(args...);

  assignJacobian(dJw, _refSkel, _node, dJwBodyNode);

  return dJw;
}

//==============================================================================
math::AngularJacobian ReferentialSkeleton::getAngularJacobianDeriv(
    const JacobianNode* _node, const Frame* _inCoordinatesOf) const
{
  return variadicGetAngularJacobianDeriv(this, _node, _inCoordinatesOf);
}

//==============================================================================
double ReferentialSkeleton::getMass() const
{
  double mass = 0;
  for(const BodyNode* bn : mRawBodyNodes)
    mass += bn->getMass();

  return mass;
}

//==============================================================================
template <const Eigen::MatrixXd& (Skeleton::*getMatrix)(std::size_t) const>
const Eigen::MatrixXd& setMatrixFromSkeletonData(
    Eigen::MatrixXd& M, const std::vector<const DegreeOfFreedom*>& dofs)
{
  const std::size_t nDofs = dofs.size();

  M.setZero();

  for(std::size_t i=0; i<nDofs; ++i)
  {
    const DegreeOfFreedom* dof_i = dofs[i];
    const std::size_t tree_i = dof_i->getTreeIndex();
    const ConstSkeletonPtr& skel_i = dof_i->getSkeleton();

    const std::size_t index_i = dof_i->getIndexInTree();
    const Eigen::MatrixXd& treeMatrix = (skel_i.get()->*getMatrix)(tree_i);

    M(i,i) = treeMatrix(index_i, index_i);

    for(std::size_t j=i+1; j<nDofs; ++j)
    {
      const DegreeOfFreedom* dof_j = dofs[j];
      const std::size_t tree_j = dof_j->getTreeIndex();
      const ConstSkeletonPtr& skel_j = dof_j->getSkeleton();

      // If the DegreesOfFreedom are in the same tree within the same
      // Skeleton, then set their entries in the referential matrix.
      // Otherwise, leave the entry as zero.
      if(skel_i == skel_j && tree_i == tree_j)
      {
        const std::size_t index_j = dof_j->getIndexInTree();

        M(i,j) = treeMatrix(index_i, index_j);
        M(j,i) = M(i,j);
      }
    }
  }

  return M;
}

//==============================================================================
const Eigen::MatrixXd& ReferentialSkeleton::getMassMatrix() const
{
  return setMatrixFromSkeletonData<&Skeleton::getMassMatrix>(mM, mRawConstDofs);
}

//==============================================================================
const Eigen::MatrixXd& ReferentialSkeleton::getAugMassMatrix() const
{
  return setMatrixFromSkeletonData<&Skeleton::getAugMassMatrix>(
        mAugM, mRawConstDofs);
}

//==============================================================================
const Eigen::MatrixXd& ReferentialSkeleton::getInvMassMatrix() const
{
  return setMatrixFromSkeletonData<&Skeleton::getInvMassMatrix>(
        mInvM, mRawConstDofs);
}

//==============================================================================
const Eigen::MatrixXd& ReferentialSkeleton::getInvAugMassMatrix() const
{
  return setMatrixFromSkeletonData<&Skeleton::getInvAugMassMatrix>(
        mInvAugM, mRawConstDofs);
}

//==============================================================================
template <const Eigen::VectorXd& (Skeleton::*getVector)(std::size_t) const>
const Eigen::VectorXd& setVectorFromSkeletonData(
    Eigen::VectorXd& V, const std::vector<const DegreeOfFreedom*>& dofs)
{
  const std::size_t nDofs = dofs.size();

  V.setZero();

  for(std::size_t i=0; i<nDofs; ++i)
  {
    const DegreeOfFreedom* dof_i = dofs[i];
    const std::size_t tree_i = dof_i->getTreeIndex();
    const ConstSkeletonPtr& skel_i = dof_i->getSkeleton();

    const std::size_t index_i = dof_i->getIndexInTree();
    const Eigen::VectorXd& treeVector = (skel_i.get()->*getVector)(tree_i);

    V[i] = treeVector[index_i];
  }

  return V;
}

//==============================================================================
const Eigen::VectorXd& ReferentialSkeleton::getCoriolisForces() const
{
  return setVectorFromSkeletonData<&Skeleton::getCoriolisForces>(
        mCvec, mRawConstDofs);
}

//==============================================================================
const Eigen::VectorXd& ReferentialSkeleton::getGravityForces() const
{
  return setVectorFromSkeletonData<&Skeleton::getGravityForces>(
        mG, mRawConstDofs);
}

//==============================================================================
const Eigen::VectorXd& ReferentialSkeleton::getCoriolisAndGravityForces() const
{
  return setVectorFromSkeletonData<&Skeleton::getCoriolisAndGravityForces>(
        mCg, mRawConstDofs);
}

//==============================================================================
const Eigen::VectorXd& ReferentialSkeleton::getExternalForces() const
{
  return setVectorFromSkeletonData<&Skeleton::getExternalForces>(
        mFext, mRawConstDofs);
}

//==============================================================================
const Eigen::VectorXd& ReferentialSkeleton::getConstraintForces() const
{
  return setVectorFromSkeletonData<&Skeleton::getConstraintForces>(
        mFc, mRawConstDofs);
}

//==============================================================================
void ReferentialSkeleton::clearExternalForces()
{
  for(BodyNode* bn : mRawBodyNodes)
    bn->clearExternalForces();
}

//==============================================================================
void ReferentialSkeleton::clearInternalForces()
{
  for(BodyNode* bn : mRawBodyNodes)
    bn->clearInternalForces();
}

//==============================================================================
double ReferentialSkeleton::computeKineticEnergy() const
{
  double KE = 0.0;

  for(const BodyNode* bn : mRawBodyNodes)
    KE += bn->computeKineticEnergy();

  assert( KE >= 0.0 && "Kinetic Energy should always be zero or greater");
  return KE;
}

//==============================================================================
double ReferentialSkeleton::computePotentialEnergy() const
{
  double PE = 0.0;

  for(const BodyNode* bn : mRawBodyNodes)
  {
    PE += bn->computePotentialEnergy(bn->getSkeleton()->getGravity());
    PE += bn->getParentJoint()->computePotentialEnergy();
  }

  return PE;
}

//==============================================================================
void ReferentialSkeleton::clearCollidingBodies()
{
  for (auto i = 0u; i < getNumBodyNodes(); ++i)
  {
    auto bodyNode = getBodyNode(i);
DART_SUPPRESS_DEPRECATED_BEGIN
    bodyNode->setColliding(false);
DART_SUPPRESS_DEPRECATED_END

    auto softBodyNode = bodyNode->asSoftBodyNode();
    if (softBodyNode)
    {
      auto& pointMasses = softBodyNode->getPointMasses();

      for (auto pointMass : pointMasses)
        pointMass->setColliding(false);
    }
  }
}

//==============================================================================
Eigen::Vector3d ReferentialSkeleton::getCOM(const Frame* _withRespectTo) const
{
  Eigen::Vector3d com = Eigen::Vector3d::Zero();
  double totalMass = 0.0;

  for(const BodyNode* bn : mRawConstBodyNodes)
  {
    com += bn->getMass() * bn->getCOM(_withRespectTo);
    totalMass += bn->getMass();
  }

  assert(totalMass != 0.0);
  return com / totalMass;
}

//==============================================================================
// Templated function for computing different kinds of COM properties, like
// velocities and accelerations
template <
    typename PropertyType,
    PropertyType (BodyNode::*getProperty)(const Frame*, const Frame*) const>
PropertyType getCOMPropertyTemplate(const ReferentialSkeleton* _refSkel,
                                    const Frame* _relativeTo,
                                    const Frame* _inCoordinatesOf)
{
  PropertyType result = PropertyType::Zero();
  double totalMass = 0.0;

  const std::vector<const BodyNode*>& bodyNodes = _refSkel->getBodyNodes();
  for(const BodyNode* bn : bodyNodes)
  {
    result += bn->getMass() * (bn->*getProperty)(_relativeTo, _inCoordinatesOf);
    totalMass += bn->getMass();
  }

  assert(totalMass != 0.0);
  return result / totalMass;
}

//==============================================================================
Eigen::Vector6d ReferentialSkeleton::getCOMSpatialVelocity(
    const Frame* _relativeTo, const Frame* _inCoordinatesOf) const
{
  return getCOMPropertyTemplate<Eigen::Vector6d,
      &BodyNode::getCOMSpatialVelocity>(this, _relativeTo, _inCoordinatesOf);
}

//==============================================================================
Eigen::Vector3d ReferentialSkeleton::getCOMLinearVelocity(
    const Frame* _relativeTo, const Frame* _inCoordinatesOf) const
{
  return getCOMPropertyTemplate<Eigen::Vector3d,
      &BodyNode::getCOMLinearVelocity>(this, _relativeTo, _inCoordinatesOf);
}

//==============================================================================
Eigen::Vector6d ReferentialSkeleton::getCOMSpatialAcceleration(
    const Frame* _relativeTo, const Frame* _inCoordinatesOf) const
{
  return getCOMPropertyTemplate<Eigen::Vector6d,
      &BodyNode::getCOMSpatialAcceleration>(
        this, _relativeTo, _inCoordinatesOf);
}

//==============================================================================
Eigen::Vector3d ReferentialSkeleton::getCOMLinearAcceleration(
    const Frame* _relativeTo, const Frame* _inCoordinatesOf) const
{
  return getCOMPropertyTemplate<Eigen::Vector3d,
      &BodyNode::getCOMLinearAcceleration>(this, _relativeTo, _inCoordinatesOf);
}

//==============================================================================
// Templated function for computing different kinds of COM Jacobians and their
// derivatives
template <
    typename JacType, // JacType is the type of Jacobian we're computing
    JacType (TemplatedJacobianNode<BodyNode>::*getJacFn)(
        const Eigen::Vector3d&, const Frame*) const>
JacType getCOMJacobianTemplate(const ReferentialSkeleton* _refSkel,
                               const Frame* _inCoordinatesOf)
{
  // Initialize the Jacobian to zero
  JacType J = JacType::Zero(JacType::RowsAtCompileTime, _refSkel->getNumDofs());
  double totalMass = 0.0;

  // Iterate through each of the BodyNodes
  const std::vector<const BodyNode*>& bodyNodes = _refSkel->getBodyNodes();
  for(const BodyNode* bn : bodyNodes)
  {
    JacType bnJ = bn->getMass() * (bn->*getJacFn)(bn->getLocalCOM(),
                                                  _inCoordinatesOf);
    totalMass += bn->getMass();

    const std::vector<const DegreeOfFreedom*>& dofs = bn->getDependentDofs();
    std::size_t nDofs = dofs.size();
    for(std::size_t i=0; i<nDofs; ++i)
    {
      const DegreeOfFreedom* dof = dofs[i];
      std::size_t index = _refSkel->getIndexOf(dof, false);
      if(INVALID_INDEX == index)
        continue;

      J.col(index) += bnJ.col(i);
    }
  }

  assert(totalMass != 0.0);
  return J / totalMass;
}

//==============================================================================
math::Jacobian ReferentialSkeleton::getCOMJacobian(
    const Frame* _inCoordinatesOf) const
{
  return getCOMJacobianTemplate<
      math::Jacobian, &TemplatedJacobianNode<BodyNode>::getJacobian>(
        this, _inCoordinatesOf);
}

//==============================================================================
math::LinearJacobian ReferentialSkeleton::getCOMLinearJacobian(
    const Frame* _inCoordinatesOf) const
{
  return getCOMJacobianTemplate<
      math::LinearJacobian, &TemplatedJacobianNode<BodyNode>::getLinearJacobian>(
        this, _inCoordinatesOf);
}

//==============================================================================
math::Jacobian ReferentialSkeleton::getCOMJacobianSpatialDeriv(
    const Frame* _inCoordinatesOf) const
{
  return getCOMJacobianTemplate<
      math::Jacobian,
          &TemplatedJacobianNode<BodyNode>::getJacobianSpatialDeriv>(
        this, _inCoordinatesOf);
}

//==============================================================================
math::LinearJacobian ReferentialSkeleton::getCOMLinearJacobianDeriv(
    const Frame* _inCoordinatesOf) const
{
  return getCOMJacobianTemplate<
      math::LinearJacobian,
          &TemplatedJacobianNode<BodyNode>::getLinearJacobianDeriv>(
        this, _inCoordinatesOf);
}

//==============================================================================
void ReferentialSkeleton::registerComponent(BodyNode* _bn)
{
  registerBodyNode(_bn);
  registerJoint(_bn->getParentJoint());

  std::size_t nDofs = _bn->getParentJoint()->getNumDofs();
  for(std::size_t i=0; i < nDofs; ++i)
    registerDegreeOfFreedom(_bn->getParentJoint()->getDof(i));
}

//==============================================================================
void ReferentialSkeleton::registerBodyNode(BodyNode* _bn)
{
  std::unordered_map<const BodyNode*, IndexMap>::iterator it =
      mIndexMap.find(_bn);

  if( it == mIndexMap.end() )
  {
    // Create an index map entry for this BodyNode, and only add the BodyNode's
    // index to it.
    IndexMap indexing;

    mBodyNodes.push_back(_bn);
    indexing.mBodyNodeIndex = mBodyNodes.size()-1;

    mIndexMap[_bn] = indexing;
  }
  else
  {
    IndexMap& indexing = it->second;

    if(INVALID_INDEX == indexing.mBodyNodeIndex)
    {
      mBodyNodes.push_back(_bn);
      indexing.mBodyNodeIndex = mBodyNodes.size()-1;
    }
  }

  updateCaches();
}

//==============================================================================
void ReferentialSkeleton::registerJoint(Joint* _joint)
{
  BodyNode* bn = _joint->getChildBodyNode();

  std::unordered_map<const BodyNode*, IndexMap>::iterator it =
      mIndexMap.find(bn);

  if( it == mIndexMap.end() )
  {
    // Create an index map entry for this Joint, and only add the Joint's index
    // to it
    IndexMap indexing;

    mJoints.push_back(_joint);
    indexing.mJointIndex = mJoints.size()-1;

    mIndexMap[bn] = indexing;
  }
  else
  {
    IndexMap& indexing = it->second;

    if(INVALID_INDEX == indexing.mJointIndex)
    {
      mJoints.push_back(_joint);
      indexing.mJointIndex = mJoints.size()-1;
    }
  }

  // Updating the caches isn't necessary after registering a joint right now,
  // but it might matter in the future, so it might be better to be safe than
  // sorry.
  updateCaches();
}

//==============================================================================
void ReferentialSkeleton::registerDegreeOfFreedom(DegreeOfFreedom* _dof)
{
  BodyNode* bn = _dof->getChildBodyNode();
  std::size_t localIndex = _dof->getIndexInJoint();

  std::unordered_map<const BodyNode*, IndexMap>::iterator it =
      mIndexMap.find(bn);

  if( it == mIndexMap.end() )
  {
    // Create an index map entry for this DegreeOfFreedom, and only add the
    // DegreeOfFreedom's index to it
    IndexMap indexing;

    indexing.mDofIndices.resize(localIndex+1, INVALID_INDEX);
    mDofs.push_back(_dof);
    indexing.mDofIndices[localIndex] = mDofs.size()-1;

    mIndexMap[bn] = indexing;
  }
  else
  {
    IndexMap& indexing = it->second;

    if(indexing.mDofIndices.size() < localIndex+1)
      indexing.mDofIndices.resize(localIndex+1, INVALID_INDEX);

    if(INVALID_INDEX == indexing.mDofIndices[localIndex])
    {
      mDofs.push_back(_dof);
      indexing.mDofIndices[localIndex] = mDofs.size()-1;
    }
  }

  updateCaches();
}

//==============================================================================
void ReferentialSkeleton::unregisterComponent(BodyNode* _bn)
{
  unregisterBodyNode(_bn, true);
  unregisterJoint(_bn);
}

//==============================================================================
void ReferentialSkeleton::unregisterBodyNode(
    BodyNode* _bn, bool _unregisterDofs)
{
  if(nullptr == _bn)
  {
    dterr << "[ReferentialSkeleton::unregisterBodyNode] Attempting to "
          << "unregister a nullptr BodyNode. This is most likely a bug. Please "
          << "report this!\n";
    assert(false);
    return;
  }

  std::unordered_map<const BodyNode*, IndexMap>::iterator it =
      mIndexMap.find(_bn);

  if( it == mIndexMap.end() )
  {
    dterr << "[ReferentialSkeleton::unregisterBodyNode] Attempting to "
          << "unregister a BodyNode that is not referred to by this "
          << "ReferentialSkeleton. This is most likely a bug. Please report "
          << "this!\n";
    assert(false);
    return;
  }

  IndexMap& indexing = it->second;
  std::size_t bnIndex = indexing.mBodyNodeIndex;
  mBodyNodes.erase(mBodyNodes.begin() + bnIndex);
  indexing.mBodyNodeIndex = INVALID_INDEX;

  for(std::size_t i=bnIndex; i < mBodyNodes.size(); ++i)
  {
    // Re-index all the BodyNodes in this ReferentialSkeleton which came after
    // the one that was removed.
    IndexMap& alteredIndexing = mIndexMap[mBodyNodes[i]];
    alteredIndexing.mBodyNodeIndex = i;
  }

  if(_unregisterDofs)
  {
    for(std::size_t i=0; i < indexing.mDofIndices.size(); ++i)
    {
      if(indexing.mDofIndices[i] != INVALID_INDEX)
        unregisterDegreeOfFreedom(_bn, i);
    }
  }

  if(indexing.isExpired())
    mIndexMap.erase(it);

  updateCaches();
}

//==============================================================================
void ReferentialSkeleton::unregisterJoint(BodyNode* _child)
{
  if(nullptr == _child)
  {
    dterr << "[ReferentialSkeleton::unregisterJoint] Attempting to unregister "
          << "a Joint from a nullptr BodyNode. This is most likely a bug. "
          << "Please report this!\n";
    assert(false);
    return;
  }

  Joint* joint = _child->getParentJoint();

  std::unordered_map<const BodyNode*, IndexMap>::iterator it =
      mIndexMap.find(_child);

  if( it == mIndexMap.end() || INVALID_INDEX == it->second.mJointIndex)
  {
    dterr << "[ReferentialSkeleton::unregisterJoint] Attempting to unregister "
          << "a Joint named [" << joint->getName() << "] (" << joint << "), "
          << "which is the parent Joint of BodyNode [" << _child->getName()
          << "] (" << _child << "), but the Joint is not currently in this "
          << "ReferentialSkeleton! This is most likely a bug. Please report "
          << "this!\n";
    assert(false);
    return;
  }

  std::size_t jointIndex = it->second.mJointIndex;
  mJoints.erase(mJoints.begin() + jointIndex);
  it->second.mJointIndex = INVALID_INDEX;

  for(std::size_t i = jointIndex; i < mJoints.size(); ++i)
  {
    // Re-index all of the Joints in this ReferentialSkeleton which came after
    // the Joint that was removed.
    JointPtr alteredJoint = mJoints[i];
    IndexMap& indexing = mIndexMap[alteredJoint.getBodyNodePtr()];
    indexing.mJointIndex = i;
  }

  if(it->second.isExpired())
    mIndexMap.erase(it);

  // Updating the caches isn't necessary after unregistering a joint right now,
  // but it might matter in the future, so it might be better to be safe than
  // sorry.
  updateCaches();
}

//==============================================================================
void ReferentialSkeleton::unregisterDegreeOfFreedom(
    BodyNode* _bn, std::size_t _localIndex)
{
  if(nullptr == _bn)
  {
    dterr << "[ReferentialSkeleton::unregisterDegreeOfFreedom] Attempting to "
          << "unregister a DegreeOfFreedom from a nullptr BodyNode. This is "
          << "most likely a bug. Please report this!\n";
    assert(false);
    return;
  }

  std::unordered_map<const BodyNode*, IndexMap>::iterator it =
      mIndexMap.find(_bn);

  if( it == mIndexMap.end() ||
      it->second.mDofIndices.size() <= _localIndex ||
      it->second.mDofIndices[_localIndex] == INVALID_INDEX)
  {
    dterr << "[ReferentialSkeleton::unregisterDegreeOfFreedom] Attempting to "
          << "unregister DegreeOfFreedom #" << _localIndex << " of a BodyNode "
          << "named [" << _bn->getName() << "] (" << _bn << "), but it is not "
          << "currently in the ReferentialSkeleton! This is most likely a bug. "
          << "Please report this!\n";
    assert(false);
    return;
  }

  std::size_t dofIndex = it->second.mDofIndices[_localIndex];
  mDofs.erase(mDofs.begin() + dofIndex);
  it->second.mDofIndices[_localIndex] = INVALID_INDEX;

  for(std::size_t i = dofIndex; i < mDofs.size(); ++i)
  {
    // Re-index all the DOFs in this ReferentialSkeleton which came after the
    // DOF that was removed.
    DegreeOfFreedomPtr dof = mDofs[i];
    IndexMap& indexing = mIndexMap[dof.getBodyNodePtr()];
    indexing.mDofIndices[dof.getLocalIndex()] = i;
  }

  if(it->second.isExpired())
    mIndexMap.erase(it);

  updateCaches();
}

//==============================================================================
void ReferentialSkeleton::updateCaches()
{
  if(mBodyNodes.size() != mRawBodyNodes.size())
  {
    mRawBodyNodes.clear();
    mRawBodyNodes.reserve(mBodyNodes.size());
    mRawConstBodyNodes.clear();
    mRawConstBodyNodes.reserve(mBodyNodes.size());

    for(const BodyNodePtr& bn : mBodyNodes)
    {
      mRawBodyNodes.push_back(bn);
      mRawConstBodyNodes.push_back(bn);
    }
  }

  mRawDofs.clear();
  mRawDofs.reserve(mDofs.size());
  mRawConstDofs.clear();
  mRawConstDofs.reserve(mDofs.size());

  // TODO(MXG): This shouldn't actually be necessary, because this always gets
  // refilled whenever getDofs() is called.
  for(const DegreeOfFreedomPtr& dof : mDofs)
  {
    mRawDofs.push_back(dof);
    mRawConstDofs.push_back(dof);
  }

  std::size_t nDofs = mDofs.size();
  mM        = Eigen::MatrixXd::Zero(nDofs, nDofs);
  mAugM     = Eigen::MatrixXd::Zero(nDofs, nDofs);
  mInvM     = Eigen::MatrixXd::Zero(nDofs, nDofs);
  mInvAugM  = Eigen::MatrixXd::Zero(nDofs, nDofs);
  mCvec     = Eigen::VectorXd::Zero(nDofs);
  mG        = Eigen::VectorXd::Zero(nDofs);
  mCg       = Eigen::VectorXd::Zero(nDofs);
  mFext     = Eigen::VectorXd::Zero(nDofs);
  mFc       = Eigen::VectorXd::Zero(nDofs);
}

//==============================================================================
ReferentialSkeleton::IndexMap::IndexMap()
  : mBodyNodeIndex(INVALID_INDEX),
    mJointIndex(INVALID_INDEX)
{
  // Do nothing
}

//==============================================================================
bool ReferentialSkeleton::IndexMap::isExpired() const
{
  if(INVALID_INDEX != mBodyNodeIndex)
    return false;

  if(INVALID_INDEX != mJointIndex)
    return false;

  for(std::size_t i=0; i < mDofIndices.size(); ++i)
  {
    if(mDofIndices[i] != INVALID_INDEX)
      return false;
  }

  return true;
}

} // namespace dynamics
} // namespace dart
