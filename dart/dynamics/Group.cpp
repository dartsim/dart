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

#include "dart/common/Console.h"
#include "dart/dynamics/Group.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/Joint.h"
#include "dart/dynamics/DegreeOfFreedom.h"

namespace dart {
namespace dynamics {

//==============================================================================
GroupPtr Group::create(const std::string& _name,
                       const std::vector<BodyNode*>& _bodyNodes)
{
  GroupPtr group(new Group(_name, _bodyNodes));
  group->mPtr = group;
  return group;
}

//==============================================================================
GroupPtr Group::create(const std::string& _name,
                       const std::vector<DegreeOfFreedom*>& _dofs)
{
  GroupPtr group(new Group(_name, _dofs));
  group->mPtr = group;
  return group;
}

//==============================================================================
void Group::swapBodyNodeIndices(size_t _index1, size_t _index2)
{
  if(_index1 >= mBodyNodes.size() || _index2 >= mBodyNodes.size())
  {
    dterr << "[Group::swapBodyNodeIndices] Trying to swap out-of-bound indices "
          << "for the Group named [" << getName() << "] (" << this << ")! "
          << "_index1:" << _index1 << " | _index2:" << _index2 << " | Both "
          << "values must be less than " << mBodyNodes.size() << "\n";
    assert(false);
    return;
  }

  BodyNode* bn1 = mBodyNodes[_index1];
  BodyNode* bn2 = mBodyNodes[_index2];

  std::unordered_map<const BodyNode*, IndexMap>::iterator it1 =
      mIndexMap.find(bn1);

  if(it1 == mIndexMap.end())
  {
    dterr << "[Group::swapBodyNodeIndices] Unable to find BodyNode ["
          << bn1->getName() << "] (" << bn1 << ") in the index map of the "
          << "Group [" << getName() << "] (" << this << ")! Please report this "
          << "as a bug!\n";
    assert(false);
    return;
  }

  std::unordered_map<const BodyNode*, IndexMap>::iterator it2 =
      mIndexMap.find(bn2);

  if(it2 == mIndexMap.end())
  {
    dterr << "[Group::swapBodyNodeIndices] Unable to find BodyNode ["
          << bn2->getName() << "] (" << bn2 << ") in the index map of the "
          << "Group [" << getName() << "] (" << this << ")! Please report this "
          << "as a bug!\n";
    assert(false);
    return;
  }

  it1->second.mBodyNodeIndex = _index2;
  it2->second.mBodyNodeIndex = _index1;

  std::swap(mBodyNodes[_index1], mBodyNodes[_index2]);
  std::swap(mRawBodyNodes[_index1], mRawBodyNodes[_index2]);
  std::swap(mRawConstBodyNodes[_index1], mRawConstBodyNodes[_index2]);
}

//==============================================================================
void Group::swapDofIndices(size_t _index1, size_t _index2)
{
  if(_index1 >= mDofs.size() || _index2 >= mDofs.size())
  {
    dterr << "[Group::swapDofIndices] Trying to swap out-of-bound indices for "
          << "the Group named [" << getName() << "] (" << this << ")! "
          << "_index1:" << _index1 << " | _index2:" << _index2 << " | Both "
          << "values must be less than " << mDofs.size() << "\n";
    assert(false);
    return;
  }

  DegreeOfFreedom* dof1 = mDofs[_index1];
  DegreeOfFreedom* dof2 = mDofs[_index2];

  std::unordered_map<const BodyNode*, IndexMap>::iterator it1 =
      mIndexMap.find(dof1->getChildBodyNode());

  if(it1 == mIndexMap.end())
  {
    dterr << "[Group::swapDofIndices] Unable to find DegreeOfFreedom ["
          << dof1->getName() << "] (" << dof1 << ") in the index map of the "
          << "Group [" << getName() << "] (" << this << ")! Please report this "
          << "as a bug!\n";
    assert(false);
    return;
  }

  std::unordered_map<const BodyNode*, IndexMap>::iterator it2 =
      mIndexMap.find(dof2->getChildBodyNode());

  if(it2 == mIndexMap.end())
  {
    dterr << "[Group::swapDofIndices] Unable to find DegreeOfFreedom ["
          << dof2->getName() << "] (" << dof2 << ") in the index map of the "
          << "Group [" << getName() << "] (" << this << ")! Please report this "
          << "as a bug!\n";
    assert(false);
    return;
  }

  size_t localIndex1 = dof1->getIndexInJoint();
  size_t localIndex2 = dof2->getIndexInJoint();

  it1->second.mDofIndices[localIndex1] = _index2;
  it2->second.mDofIndices[localIndex2] = _index1;

  std::swap(mDofs[_index1], mDofs[_index2]);
  // TODO(MXG): The following two swaps should not be necessary
  std::swap(mRawDofs[_index1], mRawDofs[_index2]);
  std::swap(mRawConstDofs[_index1], mRawConstDofs[_index2]);
}

//==============================================================================
void Group::addBodyNode(BodyNode* _bn, bool _warning)
{
  if(INVALID_INDEX != getIndexOf(_bn, false))
  {
    if(_warning)
    {
      dtwarn << "[Group::addBodyNode] The BodyNode named [" << _bn->getName()
             << "] (" << _bn << ") is already in the Group [" << getName()
             << "] (" << this << ")\n";
      assert(false);
      return;
    }

    return;
  }

  registerBodyNode(_bn);
}

//==============================================================================
void Group::addBodyNodes(const std::vector<BodyNode*>& _bodyNodes,
                         bool _warning)
{
  for(BodyNode* bn : _bodyNodes)
    addBodyNode(bn, _warning);
}

//==============================================================================
bool Group::removeBodyNode(BodyNode* _bn, bool _warning)
{
  if(INVALID_INDEX == getIndexOf(_bn, false))
  {
    if(_warning)
    {
      dtwarn << "[Group::removeBodyNode] The BodyNode named [" << _bn->getName()
             << "] (" << _bn << ") is NOT in the Group [" << getName() << "] ("
             << this << ")\n";
      assert(false);
    }

    return false;
  }

  unregisterBodyNode(_bn);

  return true;
}

//==============================================================================
bool Group::removeBodyNodes(const std::vector<BodyNode*>& _bodyNodes,
                            bool _warning)
{
  bool allGood = true;
  for(BodyNode* bn : _bodyNodes)
    allGood &= removeBodyNode(bn, _warning);

  return allGood;
}

//==============================================================================
void Group::addDof(DegreeOfFreedom* _dof, bool _warning)
{
  if(INVALID_INDEX != getIndexOf(_dof, false))
  {
    if(_warning)
    {
      dtwarn << "[Group::addDof] The DegreeOfFreedom named [" << _dof->getName()
             << "] (" << _dof << ") is already in the Group [" << getName()
             << "] (" << this << ")\n";
      assert(false);
      return;
    }

    return;
  }

  registerDegreeOfFreedom(_dof);
}

//==============================================================================
void Group::addDofs(const std::vector<DegreeOfFreedom*>& _dofs, bool _warning)
{
  for(DegreeOfFreedom* dof : _dofs)
    addDof(dof, _warning);
}

//==============================================================================
bool Group::removeDof(DegreeOfFreedom* _dof, bool _warning)
{
  if(INVALID_INDEX == getIndexOf(_dof, false))
  {
    if(_warning)
    {
      dtwarn << "[Group::removeDof] The DegreeOfFreedom named ["
             << _dof->getName() << "] (" << _dof << ") is NOT in the Group ["
             << getName() << "] (" << this << ")\n";
      assert(false);
    }

    return false;
  }

  unregisterDegreeOfFreedom(_dof->getChildBodyNode(), _dof->getIndexInJoint());

  return true;
}

//==============================================================================
bool Group::removeDofs(const std::vector<DegreeOfFreedom*>& _dofs,
                       bool _warning)
{
  bool allGood = true;
  for(DegreeOfFreedom* dof : _dofs)
    allGood &= removeDof(dof, _warning);

  return allGood;
}

//==============================================================================
Group::Group(const std::string& _name,
             const std::vector<BodyNode*>& _bodyNodes)
{
  setName(_name);
  addBodyNodes(_bodyNodes);
}

//==============================================================================
Group::Group(const std::string& _name,
             const std::vector<DegreeOfFreedom*>& _dofs)
{
  setName(_name);
  addDofs(_dofs);
}

} // namespace dynamics
} // namespace dart
