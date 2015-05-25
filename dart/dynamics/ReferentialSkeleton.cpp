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

#include "dart/dynamics/ReferentialSkeleton.h"
#include "dart/dynamics/Joint.h"
#include "dart/dynamics/DegreeOfFreedom.h"

namespace dart {
namespace dynamics {

//==============================================================================
const std::string& ReferentialSkeleton::setName(const std::string& _name)
{
  const std::string oldName = mName;
  mName = _name;

  MetaSkeletonPtr me = mPtr.lock();
  mNameChangedSignal.raise(me, oldName, mName);

  return mName;
}

//==============================================================================
const std::string& ReferentialSkeleton::getName() const
{
  return mName;
}

//==============================================================================
size_t ReferentialSkeleton::getNumBodyNodes() const
{
  return mBodyNodes.size();
}

//==============================================================================
template<typename T>
static T getVectorObjectIfAvailable(size_t _idx, const std::vector<T>& _vec)
{
  if (_idx < _vec.size())
    return _vec[_idx];

  assert( _idx < _vec.size() );
  return nullptr;
}

//==============================================================================
BodyNode* ReferentialSkeleton::getBodyNode(size_t _idx)
{
  return getVectorObjectIfAvailable<BodyNodePtr>(_idx, mBodyNodes);
}

//==============================================================================
const BodyNode* ReferentialSkeleton::getBodyNode(size_t _idx) const
{
  return getVectorObjectIfAvailable<BodyNodePtr>(_idx, mBodyNodes);
}

//==============================================================================
template <class T1, class T2>
static std::vector<T2>& convertVector(const std::vector<T1>& t1_vec,
                                      std::vector<T2>& t2_vec)
{
  t2_vec.resize(t1_vec.size());
  for(size_t i = 0; i < t1_vec.size(); ++i)
    t2_vec[i] = t1_vec[i];
  return t2_vec;
}

//==============================================================================
const std::vector<BodyNode*>& ReferentialSkeleton::getBodyNodes()
{
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
size_t ReferentialSkeleton::getIndexOf(const BodyNode* _bn, bool _warning) const
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
    return INVALID_INDEX;

  return it->second.mBodyNodeIndex;
}

//==============================================================================
size_t ReferentialSkeleton::getNumJoints() const
{
  return mBodyNodes.size();
}

//==============================================================================
Joint* ReferentialSkeleton::getJoint(size_t _idx)
{
  BodyNode* bn = getVectorObjectIfAvailable<BodyNodePtr>(_idx, mBodyNodes);
  if(nullptr == bn)
    return nullptr;

  return bn->getParentJoint();
}

//==============================================================================
const Joint* ReferentialSkeleton::getJoint(size_t _idx) const
{
  const BodyNode* bn = getVectorObjectIfAvailable<BodyNodePtr>(_idx, mBodyNodes);
  if(nullptr == bn)
    return nullptr;

  return bn->getParentJoint();
}

//==============================================================================
size_t ReferentialSkeleton::getIndexOf(const Joint* _joint, bool _warning) const
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
    return INVALID_INDEX;

  return it->second.mBodyNodeIndex;
}

//==============================================================================
size_t ReferentialSkeleton::getNumDofs() const
{
  return mDofs.size();
}

//==============================================================================
DegreeOfFreedom* ReferentialSkeleton::getDof(size_t _idx)
{
  return getVectorObjectIfAvailable<DegreeOfFreedomPtr>(_idx, mDofs);
}

//==============================================================================
const DegreeOfFreedom* ReferentialSkeleton::getDof(size_t _idx) const
{
  return getVectorObjectIfAvailable<DegreeOfFreedomPtr>(_idx, mDofs);
}

//==============================================================================
const std::vector<DegreeOfFreedom*>& ReferentialSkeleton::getDofs()
{
  return convertVector<DegreeOfFreedomPtr, DegreeOfFreedom*>(
        mDofs, mRawDofs);
}

//==============================================================================
std::vector<const DegreeOfFreedom*> ReferentialSkeleton::getDofs() const
{
  return convertVector<DegreeOfFreedomPtr, const DegreeOfFreedom*>(
        mDofs, mRawConstDofs);
}

//==============================================================================
size_t ReferentialSkeleton::getIndexOf(
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
    return INVALID_INDEX;

  size_t localIndex = _dof->getIndexInJoint();
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


//==============================================================================
void ReferentialSkeleton::registerBodyNode(BodyNode* _bn)
{
  size_t nDofs = _bn->getParentJoint()->getNumDofs();
  for(size_t i=0; i < nDofs; ++i)
  {
    registerDegreeOfFreedom(_bn->getParentJoint()->getDof(i));
  }
}

//==============================================================================
void ReferentialSkeleton::registerDegreeOfFreedom(DegreeOfFreedom* _dof)
{
  BodyNode* bn = _dof->getChildBodyNode();
  size_t localIndex = _dof->getIndexInJoint();

  std::unordered_map<const BodyNode*, IndexMap>::iterator it =
      mIndexMap.find(bn);

  if( it == mIndexMap.end() )
  {
    IndexMap indexing;
    mBodyNodes.push_back(bn);
    indexing.mBodyNodeIndex = mBodyNodes.size()-1;
    mBodyNodes.push_back(bn);

    indexing.mDofIndices.resize(localIndex+1, INVALID_INDEX);
    mDofs.push_back(_dof);
    indexing.mDofIndices[localIndex] = mDofs.size()-1;
  }
  else
  {
    IndexMap& indexing = it->second;
    if(indexing.mDofIndices.size() < localIndex+1)
      indexing.mDofIndices.resize(localIndex+1, INVALID_INDEX);
    mDofs.push_back(_dof);
    indexing.mDofIndices[localIndex] = mDofs.size()-1;
  }
}

//==============================================================================
void ReferentialSkeleton::unregisterBodyNode(BodyNode* _bn)
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

  const IndexMap& indexing = it->second;

  for(size_t i=0; i<indexing.mDofIndices.size(); ++i)
  {
    if(indexing.mDofIndices[i] != INVALID_INDEX)
      unregisterDegreeOfFreedom(_bn, i, false);
  }

  size_t bnIndex = indexing.mBodyNodeIndex;
  mBodyNodes.erase(mBodyNodes.begin() + indexing.mBodyNodeIndex);
  for(size_t i = bnIndex; i < mBodyNodes.size(); ++i)
  {
    IndexMap& indexing = mIndexMap[mBodyNodes[i]];
    indexing.mBodyNodeIndex = i;
  }
  mIndexMap.erase(it);
}

//==============================================================================
void ReferentialSkeleton::unregisterDegreeOfFreedom(
    BodyNode* _bn, size_t _localIndex, bool removeBnIfEmpty)
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
          << "unregister a DegreeOfFreedom from a BodyNode named ["
          << _bn->getName() << "] (" << _bn << ") that is not currently in the "
          << "ReferentialSkeleton! This is most likely a bug. Please report "
          << "this!\n";
    assert(false);
    return;
  }

  size_t dofIndex = it->second.mDofIndices[_localIndex];
  mDofs.erase(mDofs.begin() + dofIndex);
  it->second.mDofIndices[_localIndex] = INVALID_INDEX;

  for(size_t i = dofIndex; i < mDofs.size(); ++i)
  {
    DegreeOfFreedomPtr dof = mDofs[i];
    IndexMap& indexing = mIndexMap[dof.getBodyNodePtr()];
    indexing.mDofIndices[dof.getLocalIndex()] = i;
  }

  if(removeBnIfEmpty)
  {
    const std::vector<size_t>& dofIndices = it->second.mDofIndices;
    bool removeBn = true;
    for(size_t i=0; i<dofIndices.size(); ++i)
    {
      if(dofIndices[i] != INVALID_INDEX)
      {
        removeBn = false;
        break;
      }
    }

    if(removeBn)
      unregisterBodyNode(_bn);
  }
}

} // namespace dynamics
} // namespace dart
