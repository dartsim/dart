/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
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

#include "dart/dynamics/group.hpp"

#include "dart/common/logging.hpp"
#include "dart/common/macros.hpp"
#include "dart/dynamics/body_node.hpp"
#include "dart/dynamics/degree_of_freedom.hpp"
#include "dart/dynamics/joint.hpp"

namespace dart {
namespace dynamics {

//==============================================================================
GroupPtr Group::create(
    const std::string& name,
    std::span<BodyNode* const> bodyNodes,
    bool includeJoints,
    bool includeDofs)
{
  GroupPtr group(new Group(name, bodyNodes, includeJoints, includeDofs));
  group->mPtr = group;
  return group;
}

//==============================================================================
GroupPtr Group::create(
    const std::string& name,
    std::span<DegreeOfFreedom* const> dofs,
    bool includeBodyNodes,
    bool includeJoints)
{
  GroupPtr group(new Group(name, dofs, includeBodyNodes, includeJoints));
  group->mPtr = group;
  return group;
}

//==============================================================================
GroupPtr Group::create(
    const std::string& _name, const MetaSkeletonPtr& _metaSkeleton)
{
  GroupPtr group(new Group(_name, _metaSkeleton));
  group->mPtr = group;
  return group;
}

//==============================================================================
GroupPtr Group::cloneGroup() const
{
  return cloneGroup(getName());
}

//==============================================================================
GroupPtr Group::cloneGroup(const std::string& cloneName) const
{
  // Create an empty Group
  GroupPtr newGroup = create(cloneName);

  DART_WARN_IF(
      getNumBodyNodes() == 0u && (getNumJoints() != 0u || getNumDofs() != 0u),
      "Attempting to clone a ReferentialSkeleton that doesn't include any "
      "BodyNodes but including some Joints or DegreeOfFreedoms. This will lead "
      "to dangling Joints or DegreeOfFreedoms in the cloned "
      "ReferentialSkeleton because it only holds the strong reference to the "
      "BodyNodes but not others.");

  // Array for Skeleton clones that will be collected during cloning BodyNodes,
  // Joints, DegreeOfFreedoms.
  //
  // The clones will not be destroyed even after the map is destroyed because
  // the new Linkage will hold the skeleton by holding the strong references of
  // the body nodes.
  std::unordered_map<const Skeleton*, SkeletonPtr> mapToSkeletonClones;
  mapToSkeletonClones.reserve(mSkeletons.size());
  for (const Skeleton* skel : mSkeletons) {
    SkeletonPtr skelClone = skel->cloneSkeleton();
    mapToSkeletonClones.insert(std::make_pair(skel, skelClone));
  }

  // Add BodyNodes
  for (const BodyNodePtr& bodyNode : mBodyNodes) {
    SkeletonPtr skelClone = mapToSkeletonClones[bodyNode->getSkeleton().get()];
    DART_ASSERT(skelClone);
    BodyNode* bodyNodeClone = skelClone->getBodyNode(bodyNode->getName());
    DART_ASSERT(bodyNodeClone);
    newGroup->addBodyNode(bodyNodeClone);
  }

  // Add Joints
  for (const JointPtr& joint : mJoints) {
    SkeletonPtr skelClone = mapToSkeletonClones[joint->getSkeleton().get()];
    DART_ASSERT(skelClone);
    Joint* jointClone = skelClone->getJoint(joint->getName());
    DART_ASSERT(jointClone);
    newGroup->addJoint(jointClone, false);
  }

  // Add DegreeOfFreedoms
  for (const DegreeOfFreedomPtr& dof : mDofs) {
    SkeletonPtr skelClone = mapToSkeletonClones[dof->getSkeleton().get()];
    DART_ASSERT(skelClone);
    DegreeOfFreedom* dofClone = skelClone->getDof(dof->getName());
    DART_ASSERT(dofClone);
    newGroup->addDof(dofClone, false);
  }

  return newGroup;
}

//==============================================================================
MetaSkeletonPtr Group::cloneMetaSkeleton(const std::string& cloneName) const
{
  return cloneGroup(cloneName);
}

//==============================================================================
void Group::swapBodyNodeIndices(std::size_t _index1, std::size_t _index2)
{
  if (_index1 >= mBodyNodes.size() || _index2 >= mBodyNodes.size()) {
    DART_ERROR(
        "Trying to swap out-of-bound indices for the Group named [{}] ({})! "
        "_index1:{} | _index2:{} | Both values must be less than {}",
        getName(),
        static_cast<const void*>(this),
        _index1,
        _index2,
        mBodyNodes.size());
    DART_ASSERT(false);
    return;
  }

  BodyNode* bn1 = mBodyNodes[_index1];
  BodyNode* bn2 = mBodyNodes[_index2];

  std::unordered_map<const BodyNode*, IndexMap>::iterator it1
      = mIndexMap.find(bn1);

  if (it1 == mIndexMap.end()) {
    DART_ERROR(
        "Unable to find BodyNode [{}] ({}) in the index map of the Group [{}] "
        "({})! Please report this as a bug!",
        bn1->getName(),
        static_cast<const void*>(bn1),
        getName(),
        static_cast<const void*>(this));
    DART_ASSERT(false);
    return;
  }

  std::unordered_map<const BodyNode*, IndexMap>::iterator it2
      = mIndexMap.find(bn2);

  if (it2 == mIndexMap.end()) {
    DART_ERROR(
        "Unable to find BodyNode [{}] ({}) in the index map of the Group [{}] "
        "({})! Please report this as a bug!",
        bn2->getName(),
        static_cast<const void*>(bn2),
        getName(),
        static_cast<const void*>(this));
    DART_ASSERT(false);
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
  if (_index1 >= mDofs.size() || _index2 >= mDofs.size()) {
    DART_ERROR(
        "Trying to swap out-of-bound indices for the Group named [{}] ({})! "
        "_index1:{} | _index2:{} | Both values must be less than {}",
        getName(),
        static_cast<const void*>(this),
        _index1,
        _index2,
        mDofs.size());
    DART_ASSERT(false);
    return;
  }

  DegreeOfFreedom* dof1 = mDofs[_index1];
  DegreeOfFreedom* dof2 = mDofs[_index2];

  std::unordered_map<const BodyNode*, IndexMap>::iterator it1
      = mIndexMap.find(dof1->getChildBodyNode());

  if (it1 == mIndexMap.end()) {
    DART_ERROR(
        "Unable to find DegreeOfFreedom [{}] ({}) in the index map of the "
        "Group [{}] ({})! Please report this as a bug!",
        dof1->getName(),
        static_cast<const void*>(dof1),
        getName(),
        static_cast<const void*>(this));
    DART_ASSERT(false);
    return;
  }

  std::unordered_map<const BodyNode*, IndexMap>::iterator it2
      = mIndexMap.find(dof2->getChildBodyNode());

  if (it2 == mIndexMap.end()) {
    DART_ERROR(
        "Unable to find DegreeOfFreedom [{}] ({}) in the index map of the "
        "Group [{}] ({})! Please report this as a bug!",
        dof2->getName(),
        static_cast<const void*>(dof2),
        getName(),
        static_cast<const void*>(this));
    DART_ASSERT(false);
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
  if (nullptr == _bn) {
    if (_warning) {
      DART_WARN(
          "Attempting to add a nullptr component to the Group [{}] ({})",
          getName(),
          static_cast<const void*>(this));
      DART_ASSERT(false);
    }

    return false;
  }

  bool added = false;

  added |= addBodyNode(_bn, false);

  for (std::size_t i = 0; i < _bn->getParentJoint()->getNumDofs(); ++i) {
    added |= addDof(_bn->getParentJoint()->getDof(i), false);
  }

  if (_warning && !added) {
    DART_WARN(
        "The BodyNode named [{}] ({}) and all of its parent DegreesOfFreedom "
        "are already in the Group [{}] ({})",
        _bn->getName(),
        static_cast<const void*>(_bn),
        getName(),
        static_cast<const void*>(this));
    DART_ASSERT(false);
  }

  return added;
}

//==============================================================================
bool Group::addComponents(std::span<BodyNode* const> bodyNodes, bool warning)
{
  bool added = false;
  for (BodyNode* bn : bodyNodes) {
    added |= addComponent(bn, warning);
  }

  return added;
}

//==============================================================================
bool Group::removeComponent(BodyNode* _bn, bool _warning)
{
  if (nullptr == _bn) {
    if (_warning) {
      DART_WARN(
          "Attempting to remove a nullptr component from the Group [{}] ({})",
          getName(),
          static_cast<const void*>(this));
      DART_ASSERT(false);
    }

    return false;
  }

  bool removed = false;

  removed |= removeBodyNode(_bn, false);

  for (std::size_t i = 0; i < _bn->getParentJoint()->getNumDofs(); ++i) {
    removed |= removeDof(_bn->getParentJoint()->getDof(i), false);
  }

  if (_warning && !removed) {
    DART_WARN(
        "The BodyNode named [{}] ({}) and its parent DegreesOfFreedom were not "
        "in the Group [{}] ({})",
        _bn->getName(),
        static_cast<const void*>(_bn),
        getName(),
        static_cast<const void*>(this));
    DART_ASSERT(false);
  }

  return removed;
}

//==============================================================================
bool Group::removeComponents(std::span<BodyNode* const> bodyNodes, bool warning)
{
  bool removed = false;
  for (BodyNode* bn : bodyNodes) {
    removed |= removeComponent(bn, warning);
  }

  return removed;
}

//==============================================================================
bool Group::addBodyNode(BodyNode* _bn, bool _warning)
{
  if (nullptr == _bn) {
    if (_warning) {
      DART_WARN(
          "Attempting to add a nullptr BodyNode to the Group [{}] ({})",
          getName(),
          static_cast<const void*>(this));
      DART_ASSERT(false);
    }

    return false;
  }

  if (INVALID_INDEX != getIndexOf(_bn, false)) {
    if (_warning) {
      DART_WARN(
          "The BodyNode named [{}] ({}) is already in the Group [{}] ({})",
          _bn->getName(),
          static_cast<const void*>(_bn),
          getName(),
          static_cast<const void*>(this));
      DART_ASSERT(false);
    }

    return false;
  }

  registerBodyNode(_bn);
  return true;
}

//==============================================================================
bool Group::addBodyNodes(std::span<BodyNode* const> bodyNodes, bool warning)
{
  bool added = false;
  for (BodyNode* bn : bodyNodes) {
    added |= addBodyNode(bn, warning);
  }

  return added;
}

//==============================================================================
bool Group::removeBodyNode(BodyNode* _bn, bool _warning)
{
  if (nullptr == _bn) {
    if (_warning) {
      DART_WARN(
          "Attempting to remove a nullptr BodyNode from the Group [{}] ({})",
          getName(),
          static_cast<const void*>(this));
      DART_ASSERT(false);
    }

    return false;
  }

  if (INVALID_INDEX == getIndexOf(_bn, false)) {
    if (_warning) {
      DART_WARN(
          "The BodyNode named [{}] ({}) is NOT in the Group [{}] ({})",
          _bn->getName(),
          static_cast<const void*>(_bn),
          getName(),
          static_cast<const void*>(this));
      DART_ASSERT(false);
    }

    return false;
  }

  unregisterBodyNode(_bn, false);
  return true;
}

//==============================================================================
bool Group::removeBodyNodes(std::span<BodyNode* const> bodyNodes, bool warning)
{
  bool removed = false;
  for (BodyNode* bn : bodyNodes) {
    removed |= removeBodyNode(bn, warning);
  }

  return removed;
}

//==============================================================================
bool Group::addJoint(Joint* _joint, bool _addDofs, bool _warning)
{
  if (nullptr == _joint) {
    if (_warning) {
      DART_WARN(
          "Attempting to add a nullptr Joint to the Group [{}] ({})",
          getName(),
          static_cast<const void*>(this));
      DART_ASSERT(false);
    }

    return false;
  }

  bool added = false;
  if (INVALID_INDEX == getIndexOf(_joint, false)) {
    registerJoint(_joint);
    added = true;
  }

  if (_addDofs) {
    for (std::size_t i = 0; i < _joint->getNumDofs(); ++i) {
      added |= addDof(_joint->getDof(i), false, false);
    }
  }

  if (!added && _warning) {
    if (_addDofs) {
      DART_WARN(
          "The Joint named [{}] ({}) and all its DOFs are already in the Group "
          "[{}] ({})",
          _joint->getName(),
          static_cast<const void*>(_joint),
          getName(),
          static_cast<const void*>(this));
      DART_ASSERT(false);
    } else {
      DART_WARN(
          "The Joint named [{}] ({}) is already in the Group [{}] ({})",
          _joint->getName(),
          static_cast<const void*>(_joint),
          getName(),
          static_cast<const void*>(this));
      DART_ASSERT(false);
    }
  }

  return added;
}

//==============================================================================
bool Group::addJoints(
    std::span<Joint* const> joints, bool addDofs, bool warning)
{
  bool added = false;
  for (Joint* joint : joints) {
    added |= addJoint(joint, addDofs, warning);
  }

  return added;
}

//==============================================================================
bool Group::removeJoint(Joint* _joint, bool _removeDofs, bool _warning)
{
  if (nullptr == _joint) {
    if (_warning) {
      DART_WARN(
          "Attempting to remove a nullptr Joint from the Group [{}] ({})",
          getName(),
          static_cast<const void*>(this));
      DART_ASSERT(false);
    }

    return false;
  }

  // Make sure the Joint continues to exist for the duration of this scope
  JointPtr hold(_joint);

  bool removed = false;
  if (INVALID_INDEX != getIndexOf(_joint, false)) {
    unregisterJoint(_joint->getChildBodyNode());
    removed = true;
  }

  if (_removeDofs) {
    for (std::size_t i = 0; i < _joint->getNumDofs(); ++i) {
      removed |= removeDof(_joint->getDof(i), false, false);
    }
  }

  if (!removed && _warning) {
    if (_removeDofs) {
      DART_WARN(
          "The Joint named [{}] ({}) and its DOFs were NOT in the Group [{}] "
          "({})",
          _joint->getName(),
          static_cast<const void*>(_joint),
          getName(),
          static_cast<const void*>(this));
      DART_ASSERT(false);
    } else {
      DART_WARN(
          "The Joint named [{}] ({}) was NOT in the Group [{}] ({})",
          _joint->getName(),
          static_cast<const void*>(_joint),
          getName(),
          static_cast<const void*>(this));
      DART_ASSERT(false);
    }
  }

  return removed;
}

//==============================================================================
bool Group::removeJoints(
    std::span<Joint* const> joints, bool removeDofs, bool warning)
{
  bool removed = false;
  for (Joint* joint : joints) {
    removed |= removeJoint(joint, removeDofs, warning);
  }

  return removed;
}

//==============================================================================
bool Group::addDof(DegreeOfFreedom* _dof, bool _addJoint, bool _warning)
{
  if (nullptr == _dof) {
    if (_warning) {
      DART_WARN(
          "Attempting to add a nullptr DegreeOfFreedom to the Group [{}] ({})",
          getName(),
          static_cast<const void*>(this));
      DART_ASSERT(false);
    }

    return false;
  }

  bool added = false;
  if (INVALID_INDEX == getIndexOf(_dof, false)) {
    registerDegreeOfFreedom(_dof);
    added = true;
  }

  if (_addJoint) {
    added |= addJoint(_dof->getJoint(), false, false);
  }

  if (!added && _warning) {
    if (_addJoint) {
      DART_WARN(
          "The DegreeOfFreedom named [{}] ({}) and its Joint are already in "
          "the Group [{}] ({})",
          _dof->getName(),
          static_cast<const void*>(_dof),
          getName(),
          static_cast<const void*>(this));
      DART_ASSERT(false);
    } else {
      DART_WARN(
          "The DegreeOfFreedom named [{}] ({}) is already in the Group [{}] "
          "({})",
          _dof->getName(),
          static_cast<const void*>(_dof),
          getName(),
          static_cast<const void*>(this));
      DART_ASSERT(false);
    }
  }

  return added;
}

//==============================================================================
bool Group::addDofs(
    std::span<DegreeOfFreedom* const> dofs, bool addJoint, bool warning)
{
  bool added = false;
  for (DegreeOfFreedom* dof : dofs) {
    added |= addDof(dof, addJoint, warning);
  }

  return added;
}

//==============================================================================
bool Group::removeDof(DegreeOfFreedom* _dof, bool _cleanupJoint, bool _warning)
{
  if (nullptr == _dof) {
    if (_warning) {
      DART_WARN(
          "Attempting to remove a nullptr DegreeOfFreedom from the Group [{}] "
          "({})",
          getName(),
          static_cast<const void*>(this));
      DART_ASSERT(false);
    }

    return false;
  }

  // Make sure the DegreeOfFreedom continues to exist for the duration of this
  // scope
  DegreeOfFreedomPtr hold(_dof);

  bool removed = false;
  if (INVALID_INDEX != getIndexOf(_dof, false)) {
    unregisterDegreeOfFreedom(
        _dof->getChildBodyNode(), _dof->getIndexInJoint());
    removed = true;
  }

  if (_cleanupJoint) {
    // Check whether any DOFs belonging to the Joint are remaining in the Group
    bool performCleanup = true;
    Joint* joint = _dof->getJoint();
    for (std::size_t i = 0; i < joint->getNumDofs(); ++i) {
      if (getIndexOf(joint->getDof(i), false) == INVALID_INDEX) {
        performCleanup = false;
        break;
      }
    }

    // Remove the Joint if none of its DOFs remain
    if (performCleanup) {
      removed |= removeJoint(joint, false, false);
    }
  }

  if (!removed && _warning) {
    if (_cleanupJoint) {
      DART_WARN(
          "The DegreeOfFreedom named [{}] ({}) and its Joint were NOT in the "
          "Group [{}] ({})",
          _dof->getName(),
          static_cast<const void*>(_dof),
          getName(),
          static_cast<const void*>(this));
      DART_ASSERT(false);
    } else {
      DART_WARN(
          "The DegreeOfFreedom named [{}] ({}) was NOT in the Group [{}] ({})",
          _dof->getName(),
          static_cast<const void*>(_dof),
          getName(),
          static_cast<const void*>(this));
      DART_ASSERT(false);
    }
  }

  return removed;
}

//==============================================================================
bool Group::removeDofs(
    std::span<DegreeOfFreedom* const> dofs, bool cleanupJoint, bool warning)
{
  bool removed = false;
  for (DegreeOfFreedom* dof : dofs) {
    removed |= removeDof(dof, cleanupJoint, warning);
  }

  return removed;
}

//==============================================================================
Group::Group(
    const std::string& name,
    std::span<BodyNode* const> bodyNodes,
    bool includeJoints,
    bool includeDofs)
{
  setName(name);
  addBodyNodes(bodyNodes);

  if (includeDofs || includeJoints) {
    for (std::size_t i = 0; i < bodyNodes.size(); ++i) {
      Joint* joint = bodyNodes[i]->getParentJoint();

      if (includeJoints) {
        addJoint(joint, false);
      }

      if (includeDofs) {
        for (std::size_t j = 0; j < joint->getNumDofs(); ++j) {
          addDof(joint->getDof(j));
        }
      }
    }
  }
}

//==============================================================================
Group::Group(
    const std::string& name,
    std::span<DegreeOfFreedom* const> dofs,
    bool includeBodyNodes,
    bool includeJoints)
{
  setName(name);
  addDofs(dofs, includeJoints);

  if (includeBodyNodes) {
    for (std::size_t i = 0; i < dofs.size(); ++i) {
      DegreeOfFreedom* dof = dofs[i];
      addBodyNode(dof->getChildBodyNode(), false);
    }
  }
}

//==============================================================================
Group::Group(const std::string& _name, const MetaSkeletonPtr& _metaSkeleton)
{
  setName(_name);

  if (_metaSkeleton) {
    for (std::size_t i = 0; i < _metaSkeleton->getNumBodyNodes(); ++i) {
      addBodyNode(_metaSkeleton->getBodyNode(i));
    }

    for (std::size_t i = 0; i < _metaSkeleton->getNumJoints(); ++i) {
      addJoint(_metaSkeleton->getJoint(i), false);
    }

    for (std::size_t i = 0; i < _metaSkeleton->getNumDofs(); ++i) {
      addDof(_metaSkeleton->getDof(i), false);
    }
  }
}

} // namespace dynamics
} // namespace dart
