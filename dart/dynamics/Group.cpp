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

#include "dart/common/Console.hpp"
#include "dart/dynamics/Group.hpp"
#include "dart/dynamics/BodyNode.hpp"
#include "dart/dynamics/Joint.hpp"
#include "dart/dynamics/DegreeOfFreedom.hpp"

namespace dart {
namespace dynamics {

//==============================================================================
GroupPtr Group::create(const std::string& _name,
                       const std::vector<BodyNode*>& _bodyNodes,
                       bool _includeJoints, bool _includeDofs)
{
  GroupPtr group(new Group(_name, _bodyNodes, _includeJoints, _includeDofs));
  group->mPtr = group;
  return group;
}

//==============================================================================
GroupPtr Group::create(const std::string& _name,
                       const std::vector<DegreeOfFreedom*>& _dofs,
                       bool _includeBodyNodes, bool _includeJoints)
{
  GroupPtr group(new Group(_name, _dofs, _includeBodyNodes, _includeJoints));
  group->mPtr = group;
  return group;
}

//==============================================================================
GroupPtr Group::create(const std::string& _name,
                       const MetaSkeletonPtr& _metaSkeleton)
{
  GroupPtr group(new Group(_name, _metaSkeleton));
  group->mPtr = group;
  return group;
}

//==============================================================================
void Group::swapBodyNodeIndices(std::size_t _index1, std::size_t _index2)
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
void Group::swapDofIndices(std::size_t _index1, std::size_t _index2)
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

  std::size_t localIndex1 = dof1->getIndexInJoint();
  std::size_t localIndex2 = dof2->getIndexInJoint();

  it1->second.mDofIndices[localIndex1] = _index2;
  it2->second.mDofIndices[localIndex2] = _index1;

  std::swap(mDofs[_index1], mDofs[_index2]);
  // TODO(MXG): The following two swaps should not be necessary
  std::swap(mRawDofs[_index1], mRawDofs[_index2]);
  std::swap(mRawConstDofs[_index1], mRawConstDofs[_index2]);
}

//==============================================================================
bool Group::addComponent(BodyNode* _bn, bool _warning)
{
  if(nullptr == _bn)
  {
    if(_warning)
    {
      dtwarn << "[Group::addComponent] Attempting to add a nullptr component "
             << "to the Group [" << getName() << "] (" << this << ")\n";
      assert(false);
    }

    return false;
  }

  bool added = false;

  added |= addBodyNode(_bn, false);

  for(std::size_t i=0; i < _bn->getParentJoint()->getNumDofs(); ++i)
    added |= addDof(_bn->getParentJoint()->getDof(i), false);

  if(_warning && !added)
  {
    dtwarn << "[Group::addComponent] The BodyNode named [" << _bn->getName()
           << "] (" << _bn << ") and all of its parent DegreesOfFreedom are "
           << "already in the Group [" << getName() << "] (" << this << ")\n";
    assert(false);
  }

  return added;
}

//==============================================================================
bool Group::addComponents(const std::vector<BodyNode*>& _bodyNodes,
                          bool _warning)
{
  bool added = false;
  for(BodyNode* bn : _bodyNodes)
    added |= addComponent(bn, _warning);

  return added;
}

//==============================================================================
bool Group::removeComponent(BodyNode* _bn, bool _warning)
{
  if(nullptr == _bn)
  {
    if(_warning)
    {
      dtwarn << "[Group::removeComponent] Attempting to remove a nullptr "
             << "component from the Group [" << getName() << "] (" << this
             << ")\n";
      assert(false);
    }

    return false;
  }

  bool removed = false;

  removed |= removeBodyNode(_bn, false);

  for(std::size_t i=0; i < _bn->getParentJoint()->getNumDofs(); ++i)
    removed |= removeDof(_bn->getParentJoint()->getDof(i), false);

  if(_warning && !removed)
  {
    dtwarn << "[Group::removeComponent] The BodyNode named [" << _bn->getName()
           << "] (" << _bn << ") and its parent DegreesOfFreedom were not in "
           << "the Group [" << getName() << "] (" << this << ")\n";
    assert(false);
  }

  return removed;
}

//==============================================================================
bool Group::removeComponents(const std::vector<BodyNode*>& _bodyNodes,
                             bool _warning)
{
  bool removed = false;
  for(BodyNode* bn : _bodyNodes)
    removed |= removeComponent(bn, _warning);

  return removed;
}

//==============================================================================
bool Group::addBodyNode(BodyNode* _bn, bool _warning)
{
  if(nullptr == _bn)
  {
    if(_warning)
    {
      dtwarn << "[Group::addBodyNode] Attempting to add a nullptr BodyNode "
             << "to the Group [" << getName() << "] (" << this << ")\n";
      assert(false);
    }

    return false;
  }

  if(INVALID_INDEX != getIndexOf(_bn, false))
  {
    if(_warning)
    {
      dtwarn << "[Group::addBodyNode] The BodyNode named [" << _bn->getName()
             << "] (" << _bn << ") is already in the Group [" << getName()
             << "] (" << this << ")\n";
      assert(false);
    }

    return false;
  }

  registerBodyNode(_bn);
  return true;
}

//==============================================================================
bool Group::addBodyNodes(const std::vector<BodyNode*>& _bodyNodes,
                         bool _warning)
{
  bool added = false;
  for(BodyNode* bn : _bodyNodes)
    added |= addBodyNode(bn, _warning);

  return added;
}

//==============================================================================
bool Group::removeBodyNode(BodyNode* _bn, bool _warning)
{
  if(nullptr == _bn)
  {
    if(_warning)
    {
      dtwarn << "[Group::removeBodyNode] Attempting to remove a nullptr "
             << "BodyNode from the Group [" << getName() << "] (" << this
             << ")\n";
      assert(false);
    }

    return false;
  }

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

  unregisterBodyNode(_bn, false);
  return true;
}

//==============================================================================
bool Group::removeBodyNodes(const std::vector<BodyNode*>& _bodyNodes,
                            bool _warning)
{
  bool removed = false;
  for(BodyNode* bn : _bodyNodes)
    removed |= removeBodyNode(bn, _warning);

  return removed;
}

//==============================================================================
bool Group::addJoint(Joint* _joint, bool _addDofs, bool _warning)
{
  if(nullptr == _joint)
  {
    if(_warning)
    {
      dtwarn << "[Group::addJoint] Attempting to add a nullptr Joint to the "
             << "Group [" << getName() << "] (" << this << ")\n";
      assert(false);
    }

    return false;
  }

  bool added = false;
  if(INVALID_INDEX == getIndexOf(_joint, false))
  {
    registerJoint(_joint);
    added = true;
  }

  if(_addDofs)
  {
    for(std::size_t i=0; i < _joint->getNumDofs(); ++i)
      added |= addDof(_joint->getDof(i), false, false);
  }

  if(!added && _warning)
  {
    if(_addDofs)
    {
      dtwarn << "[Group::addJoint] The Joint named [" << _joint->getName()
             << "] (" << _joint << ") and all its DOFs are already in the "
             << "Group [" << getName() << "] (" << this << ")\n";
      assert(false);
    }
    else
    {
      dtwarn << "[Group::addJoint] The Joint named [" << _joint->getName()
             << "] (" << _joint << ") is already in the Group [" << getName()
             << "] (" << this << ")\n";
      assert(false);
    }
  }

  return added;
}

//==============================================================================
bool Group::addJoints(const std::vector<Joint*>& _joints,
                      bool _addDofs, bool _warning)
{
  bool added = false;
  for(Joint* joint : _joints)
    added |= addJoint(joint, _addDofs, _warning);

  return added;
}

//==============================================================================
bool Group::removeJoint(Joint* _joint, bool _removeDofs, bool _warning)
{
  if(nullptr == _joint)
  {
    if(_warning)
    {
      dtwarn << "[Group::removeJoint] Attempting to remove a nullptr Joint "
             << "from the Group [" << getName() << "] (" << this << ")\n";
      assert(false);
    }

    return false;
  }

  // Make sure the Joint continues to exist for the duration of this scope
  JointPtr hold(_joint);

  bool removed = false;
  if(INVALID_INDEX != getIndexOf(_joint, false))
  {
    unregisterJoint(_joint->getChildBodyNode());
    removed = true;
  }

  if(_removeDofs)
  {
    for(std::size_t i=0; i < _joint->getNumDofs(); ++i)
      removed |= removeDof(_joint->getDof(i), false, false);
  }

  if(!removed && _warning)
  {
    if(_removeDofs)
    {
      dtwarn << "[Group::removeJoint] The Joint named [" << _joint->getName()
             << "] (" << _joint << ") and its DOFs were NOT in the Group ["
             << getName() << "] (" << this << ")\n";
      assert(false);
    }
    else
    {
      dtwarn << "[Group::removeJoint] The Joint named [" << _joint->getName()
             << "] (" << _joint << ") was NOT in the Group [" << getName()
             << "] (" << this << ")\n";
      assert(false);
    }
  }

  return removed;
}

//==============================================================================
bool Group::addDof(DegreeOfFreedom* _dof, bool _addJoint, bool _warning)
{
  if(nullptr == _dof)
  {
    if(_warning)
    {
      dtwarn << "[Group::addDof] Attempting to add a nullptr DegreeOfFreedom "
             << "to the Group [" << getName() << "] (" << this << ")\n";
      assert(false);
    }

    return false;
  }

  bool added = false;
  if(INVALID_INDEX == getIndexOf(_dof, false))
  {
    registerDegreeOfFreedom(_dof);
    added = true;
  }

  if(_addJoint)
    added |= addJoint(_dof->getJoint(), false, false);

  if(!added && _warning)
  {
    if(_addJoint)
    {
      dtwarn << "[Group::addDof] The DegreeOfFreedom named [" << _dof->getName()
             << "] (" << _dof << ") and its Joint are already in the Group ["
             << getName() << "] (" << this << ")\n";
      assert(false);
    }
    else
    {
      dtwarn << "[Group::addDof] The DegreeOfFreedom named [" << _dof->getName()
             << "] (" << _dof << ") is already in the Group [" << getName()
             << "] (" << this << ")\n";
      assert(false);
    }
  }

  return added;
}

//==============================================================================
bool Group::addDofs(const std::vector<DegreeOfFreedom*>& _dofs,
                    bool _addJoint, bool _warning)
{
  bool added = false;
  for(DegreeOfFreedom* dof : _dofs)
    added |= addDof(dof, _addJoint, _warning);

  return added;
}

//==============================================================================
bool Group::removeDof(DegreeOfFreedom* _dof, bool _cleanupJoint, bool _warning)
{
  if(nullptr == _dof)
  {
    if(_warning)
    {
      dtwarn << "[Group::removeDof] Attempting to remove a nullptr "
             << "DegreeOfFreedom from the Group [" << getName() << "] ("
             << this << ")\n";
      assert(false);
    }

    return false;
  }

  // Make sure the DegreeOfFreedom continues to exist for the duration of this
  // scope
  DegreeOfFreedomPtr hold(_dof);

  bool removed = false;
  if(INVALID_INDEX != getIndexOf(_dof, false))
  {
    unregisterDegreeOfFreedom(_dof->getChildBodyNode(), _dof->getIndexInJoint());
    removed = true;
  }

  if(_cleanupJoint)
  {
    // Check whether any DOFs belonging to the Joint are remaining in the Group
    bool performCleanup = true;
    Joint* joint = _dof->getJoint();
    for(std::size_t i=0; i < joint->getNumDofs(); ++i)
    {
      if(getIndexOf(joint->getDof(i), false) == INVALID_INDEX)
      {
        performCleanup = false;
        break;
      }
    }

    // Remove the Joint if none of its DOFs remain
    if(performCleanup)
      removed |= removeJoint(joint, false, false);
  }

  if(!removed && _warning)
  {
    if(_cleanupJoint)
    {
      dtwarn << "[Group::removeDof] The DegreeOfFreedom named ["
             << _dof->getName() << "] (" << _dof << ") and its Joint were NOT "
             << "in the Group [" << getName() << "] (" << this << ")\n";
      assert(false);
    }
    else
    {
      dtwarn << "[Group::removeDof] The DegreeOfFreedom named ["
             << _dof->getName() << "] (" << _dof << ") was NOT in the Group ["
             << getName() << "] (" << this << ")\n";
      assert(false);
    }
  }

  return removed;
}

//==============================================================================
bool Group::removeDofs(const std::vector<DegreeOfFreedom*>& _dofs,
                       bool _cleanupJoint, bool _warning)
{
  bool removed = false;
  for(DegreeOfFreedom* dof : _dofs)
    removed |= removeDof(dof, _cleanupJoint, _warning);

  return removed;
}

//==============================================================================
Group::Group(const std::string& _name,
             const std::vector<BodyNode*>& _bodyNodes,
             bool _includeJoints, bool _includeDofs)
{
  setName(_name);
  addBodyNodes(_bodyNodes);

  if(_includeDofs || _includeJoints)
  {
    for(std::size_t i=0; i < _bodyNodes.size(); ++i)
    {
      Joint* joint = _bodyNodes[i]->getParentJoint();

      if(_includeJoints)
        addJoint(joint, false);

      if(_includeDofs)
      {
        for(std::size_t j=0; j < joint->getNumDofs(); ++j)
          addDof(joint->getDof(j));
      }
    }
  }
}

//==============================================================================
Group::Group(const std::string& _name,
             const std::vector<DegreeOfFreedom*>& _dofs,
             bool _includeBodyNodes, bool _includeJoints)
{
  setName(_name);
  addDofs(_dofs, _includeJoints);

  if(_includeBodyNodes)
  {
    for(std::size_t i=0; i < _dofs.size(); ++i)
    {
      DegreeOfFreedom* dof = _dofs[i];
      addBodyNode(dof->getChildBodyNode(), false);
    }
  }
}

//==============================================================================
Group::Group(const std::string& _name,
             const MetaSkeletonPtr& _metaSkeleton)
{
  setName(_name);

  if(_metaSkeleton)
  {
    for(std::size_t i=0; i < _metaSkeleton->getNumBodyNodes(); ++i)
      addBodyNode(_metaSkeleton->getBodyNode(i));

    for(std::size_t i=0; i < _metaSkeleton->getNumJoints(); ++i)
      addJoint(_metaSkeleton->getJoint(i), false);

    for(std::size_t i=0; i < _metaSkeleton->getNumDofs(); ++i)
      addDof(_metaSkeleton->getDof(i), false);
  }
}

} // namespace dynamics
} // namespace dart
