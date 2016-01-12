/*
 * Copyright (c) 2013-2015, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>,
 *            Tobias Kunz <tobias@gatech.edu>
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

#include "kido/collision/CollisionDetector.h"

#include <algorithm>
#include <iostream>
#include <vector>

#include "kido/common/Console.h"
#include "kido/dynamics/BodyNode.h"
#include "kido/dynamics/Skeleton.h"
#include "kido/collision/CollisionNode.h"

namespace kido {
namespace collision {

CollisionDetector::CollisionDetector()
  : mNumMaxContacts(100) {
}

CollisionDetector::~CollisionDetector() {
  for (size_t i = 0; i < mCollisionNodes.size(); i++)
    delete mCollisionNodes[i];
}

//==============================================================================
void CollisionDetector::addSkeleton(const dynamics::SkeletonPtr& _skeleton)
{
  assert(_skeleton != nullptr
      && "Null pointer skeleton is not allowed to add to CollisionDetector.");

  if (containSkeleton(_skeleton) == false)
  {
    mSkeletons.push_back(_skeleton);
    for (size_t i = 0; i < _skeleton->getNumBodyNodes(); ++i)
      addCollisionSkeletonNode(_skeleton->getBodyNode(i));
  }
  else
  {
    dtwarn << "Skeleton [" << _skeleton->getName()
           << "] is already in CollisionDetector." << std::endl;
  }
}

//==============================================================================
void CollisionDetector::removeSkeleton(const dynamics::SkeletonPtr& _skeleton)
{
  assert(_skeleton != nullptr
      && "Null pointer skeleton is not allowed to add to CollisionDetector.");

  if (containSkeleton(_skeleton))
  {
    mSkeletons.erase(remove(mSkeletons.begin(), mSkeletons.end(), _skeleton),
                     mSkeletons.end());
    for (size_t i = 0; i < _skeleton->getNumBodyNodes(); ++i)
      removeCollisionSkeletonNode(_skeleton->getBodyNode(i));
  }
  else
  {
    dtwarn << "Skeleton [" << _skeleton->getName()
           << "] is not in CollisionDetector." << std::endl;
  }
}

//==============================================================================
void CollisionDetector::removeAllSkeletons()
{
  for (size_t i = 0; i < mSkeletons.size(); ++i)
    removeSkeleton(mSkeletons[i]);

  mSkeletons.clear();
}

void CollisionDetector::addCollisionSkeletonNode(dynamics::BodyNode* _bodyNode,
                                                 bool _isRecursive) {
  assert(_bodyNode != nullptr && "Invalid body node.");

  // If this collision detector already has collision node for _bodyNode, then
  // we do nothing.
  if (getCollisionNode(_bodyNode) != nullptr) {
    std::cout << "The collision detector already has a collision node for "
              << "body node [" << _bodyNode->getName() << "]." << std::endl;
    return;
  }

  // Create collision node and set index
  CollisionNode* collNode = createCollisionNode(_bodyNode);
  collNode->setIndex(mCollisionNodes.size());

  // Add the collision node to collision node list
  mCollisionNodes.push_back(collNode);

  // Add the collision node to map (BodyNode -> CollisionNode)
  mBodyCollisionMap[_bodyNode] = collNode;

  // Add collidable pairs for the collision node
  mCollidablePairs.push_back(
        std::vector<bool>(mCollisionNodes.size() - 1, true));

  if (_isRecursive) {
    for (size_t i = 0; i < _bodyNode->getNumChildBodyNodes(); i++)
      addCollisionSkeletonNode(_bodyNode->getChildBodyNode(i), true);
  }
}

void CollisionDetector::removeCollisionSkeletonNode(
    dynamics::BodyNode* _bodyNode, bool _isRecursive) {
  assert(_bodyNode != nullptr && "Invalid body node.");

  // If a collision node is already created for _bodyNode, then we just return
  CollisionNode* collNode = getCollisionNode(_bodyNode);
  if (collNode == nullptr) {
    std::cout << "The collision detector does not have any collision node "
              << "for body node [" << _bodyNode->getName() << "]."
              << std::endl;
    return;
  }

  // Update index of collision nodes.
  size_t iCollNode = collNode->getIndex();
  for (size_t i = iCollNode + 1; i < mCollisionNodes.size(); ++i)
    mCollisionNodes[i]->setIndex(mCollisionNodes[i]->getIndex() - 1);

  // Remove collNode from mCollisionNodes
  mCollisionNodes.erase(remove(mCollisionNodes.begin(), mCollisionNodes.end(),
                               collNode),
                        mCollisionNodes.end());

  // Remove collNode-_bodyNode pair from mBodyCollisionMap
  mBodyCollisionMap.erase(_bodyNode);

  // Delete collNode
  delete collNode;

  // Update mCollidablePairs
  for (size_t i = iCollNode + 1; i < mCollidablePairs.size(); ++i) {
    for (size_t j = 0; j < iCollNode; ++j)
      mCollidablePairs[i-1][j] = mCollidablePairs[i][j];
    for (size_t j = iCollNode + 1; j < mCollidablePairs[i].size(); ++j)
      mCollidablePairs[i-1][j-1] = mCollidablePairs[i][j];
  }
  mCollidablePairs.pop_back();

  if (_isRecursive) {
    for (size_t i = 0; i < _bodyNode->getNumChildBodyNodes(); i++)
      removeCollisionSkeletonNode(_bodyNode->getChildBodyNode(i), true);
  }
}

bool CollisionDetector::detectCollision(dynamics::BodyNode* _node1,
                                        dynamics::BodyNode* _node2,
                                        bool _calculateContactPoints) {
  return detectCollision(getCollisionNode(_node1),
                         getCollisionNode(_node2),
                         _calculateContactPoints);
}

size_t CollisionDetector::getNumContacts() {
  return mContacts.size();
}

Contact& CollisionDetector::getContact(int _idx) {
  return mContacts[_idx];
}

void CollisionDetector::clearAllContacts() {
  mContacts.clear();
}

int CollisionDetector::getNumMaxContacts() const {
  return mNumMaxContacts;
}

void CollisionDetector::setNumMaxContacs(int _num) {
  mNumMaxContacts = _num;
}

void CollisionDetector::enablePair(dynamics::BodyNode* _node1,
                                   dynamics::BodyNode* _node2) {
  CollisionNode* collisionNode1 = getCollisionNode(_node1);
  CollisionNode* collisionNode2 = getCollisionNode(_node2);
  if (collisionNode1 && collisionNode2)
    setPairCollidable(collisionNode1, collisionNode2, true);
}

void CollisionDetector::disablePair(dynamics::BodyNode* _node1,
                                    dynamics::BodyNode* _node2) {
  CollisionNode* collisionNode1 = getCollisionNode(_node1);
  CollisionNode* collisionNode2 = getCollisionNode(_node2);
  if (collisionNode1 && collisionNode2)
    setPairCollidable(collisionNode1, collisionNode2, false);
}

//==============================================================================
bool CollisionDetector::isCollidable(const CollisionNode* _node1,
                                     const CollisionNode* _node2)
{
  dynamics::BodyNode* bn1 = _node1->getBodyNode();
  dynamics::BodyNode* bn2 = _node2->getBodyNode();

  if (!getPairCollidable(_node1, _node2))
    return false;

  if (!bn1->isCollidable() || !bn2->isCollidable())
    return false;

  if (bn1->getSkeleton() == bn2->getSkeleton())
  {
    if (bn1->getSkeleton()->isEnabledSelfCollisionCheck())
    {
      if (isAdjacentBodies(bn1, bn2))
      {
        if (!bn1->getSkeleton()->isEnabledAdjacentBodyCheck())
          return false;
      }
    }
    else
    {
      return false;
    }
  }

  return true;
}

//==============================================================================
bool CollisionDetector::containSkeleton(const dynamics::SkeletonPtr& _skeleton)
{
  for (std::vector<dynamics::SkeletonPtr>::const_iterator it = mSkeletons.begin();
       it != mSkeletons.end(); ++it)
  {
    if ((*it) == _skeleton)
      return true;
  }

  return false;
}

bool isValidIndex(const std::vector<std::vector<bool>>& _collidablePairs,
                  const std::size_t _index1,
                  const std::size_t _index2)
{
  assert(_index1 >= _index2);

  if (_collidablePairs.size() > _index1
      && _collidablePairs[_index1].size() > _index2)
    return true;

  return false;
}

bool CollisionDetector::getPairCollidable(const CollisionNode* _node1,
                                          const CollisionNode* _node2)
{
  assert(_node1 != _node2);

  size_t index1 = _node1->getIndex();
  size_t index2 = _node2->getIndex();

  if (index1 < index2)
    std::swap(index1, index2);

  // Index validity check. The indices are not valid if the body nodes are not
  // completely added to the collision detector yet.
  if (!isValidIndex(mCollidablePairs, index1, index2))
    return false;

  if (index1 == index2)
    return false;
  
  return mCollidablePairs[index1][index2];
}

void CollisionDetector::setPairCollidable(const CollisionNode* _node1,
                                          const CollisionNode* _node2,
                                          bool _val)
{
  assert(_node1 != _node2);

  size_t index1 = _node1->getIndex();
  size_t index2 = _node2->getIndex();

  if (index1 < index2)
    std::swap(index1, index2);

  // Index validity check. The indices are not valid if the body nodes are not
  // completely added to the collision detector yet.
  if (!isValidIndex(mCollidablePairs, index1, index2))
    return;

  mCollidablePairs[index1][index2] = _val;
}

bool CollisionDetector::isAdjacentBodies(
    const dynamics::BodyNode* _bodyNode1,
    const dynamics::BodyNode* _bodyNode2) const
{
  if ((_bodyNode1->getParentBodyNode() == _bodyNode2)
      || (_bodyNode2->getParentBodyNode() == _bodyNode1))
  {
    assert(_bodyNode1->getSkeleton() == _bodyNode2->getSkeleton());
    return true;
  }

  return false;
}

CollisionNode* CollisionDetector::getCollisionNode(
    const dynamics::BodyNode* _bodyNode) {
  if (mBodyCollisionMap.find(_bodyNode) != mBodyCollisionMap.end())
    return mBodyCollisionMap[_bodyNode];
  else
    return nullptr;
}

}  // namespace collision
}  // namespace kido
