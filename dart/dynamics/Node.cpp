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

#include "dart/dynamics/Node.hpp"
#include "dart/dynamics/BodyNode.hpp"

#define REPORT_INVALID_NODE( func )                                          \
  dterr << "[Node::" #func "] This Node was not constructed correctly. It "  \
        << "needs to specify a valid BodyNode pointer during construction. " \
        << "Please report this as a bug if it is not a custom node type!\n"; \
  assert(false);

namespace dart {
namespace dynamics {

//==============================================================================
NodeDestructor::NodeDestructor(Node* _node)
  : mNode(_node)
{
  // Do nothing
}

//==============================================================================
NodeDestructor::~NodeDestructor()
{
  delete mNode;
}

//==============================================================================
Node* NodeDestructor::getNode() const
{
  return mNode;
}

//==============================================================================
void Node::setNodeState(const State& /*otherState*/)
{
  // Do nothing
}

//==============================================================================
std::unique_ptr<Node::State> Node::getNodeState() const
{
  return nullptr;
}

//==============================================================================
void Node::copyNodeStateTo(std::unique_ptr<State>& outputState) const
{
  outputState = getNodeState();
}

//==============================================================================
void Node::setNodeProperties(const Properties& /*properties*/)
{
  // Do nothing
}

//==============================================================================
std::unique_ptr<Node::Properties> Node::getNodeProperties() const
{
  return nullptr;
}

//==============================================================================
void Node::copyNodePropertiesTo(
    std::unique_ptr<Properties>& outputProperties) const
{
  outputProperties = getNodeProperties();
}

//==============================================================================
BodyNodePtr Node::getBodyNodePtr()
{
  return mBodyNode;
}

//==============================================================================
ConstBodyNodePtr Node::getBodyNodePtr() const
{
  return mBodyNode;
}

//==============================================================================
bool Node::isRemoved() const
{
  if(nullptr == mBodyNode)
  {
    REPORT_INVALID_NODE(isRemoved);
    return true;
  }

  return !mAmAttached;
}

//==============================================================================
std::shared_ptr<Skeleton> Node::getSkeleton()
{
  return mBodyNode->getSkeleton();
}

//==============================================================================
std::shared_ptr<const Skeleton> Node::getSkeleton() const
{
  return mBodyNode->getSkeleton();
}

//==============================================================================
std::shared_ptr<NodeDestructor> Node::getOrCreateDestructor()
{
  std::shared_ptr<NodeDestructor> destructor = mDestructor.lock();
  if(nullptr == destructor)
  {
    destructor = std::shared_ptr<NodeDestructor>(new NodeDestructor(this));
    mDestructor = destructor;
  }

  return destructor;
}

//==============================================================================
Node::Node(BodyNode* _bn)
  : mBodyNode(_bn),
    mAmAttached(false),
    mIndexInBodyNode(INVALID_INDEX),
    mIndexInSkeleton(INVALID_INDEX),
    mIndexInTree(INVALID_INDEX)
{
  if(nullptr == mBodyNode)
  {
    REPORT_INVALID_NODE(Node);
    return;
  }

  if(mBodyNode != this)
    setVersionDependentObject(mBodyNode);
}

//==============================================================================
std::string Node::registerNameChange(const std::string& newName)
{
  const SkeletonPtr& skel = mBodyNode->getSkeleton();
  if(nullptr == skel)
    return newName;

  Skeleton::NodeNameMgrMap& nodeNameMgrMap = skel->mNodeNameMgrMap;
  Skeleton::NodeNameMgrMap::iterator it = nodeNameMgrMap.find(typeid(*this));

  if(nodeNameMgrMap.end() == it)
    return newName;

  common::NameManager<Node*>& mgr = it->second;
  return mgr.changeObjectName(this, newName);
}

//==============================================================================
void Node::attach()
{
  if(nullptr == mBodyNode)
  {
    REPORT_INVALID_NODE(attach);
    return;
  }

  // If we are in release mode, and the Node believes it is attached, then we
  // can shortcut this procedure
#ifdef NDEBUG
  if(mAmAttached)
    return;
#endif

  using NodeMapPair = std::pair<std::type_index, std::vector<Node*>>;

  // Add empty list of Node pointers only when typeid(*this) doesn't exist.
  BodyNode::NodeMap::iterator it = mBodyNode->mNodeMap.insert(
      NodeMapPair(typeid(*this), std::vector<Node*>())).first;

  std::vector<Node*>& nodes = it->second;
  BodyNode::NodeDestructorSet& destructors = mBodyNode->mNodeDestructors;

  NodeDestructorPtr destructor = getOrCreateDestructor();
  if(INVALID_INDEX == mIndexInBodyNode)
  {
    // If the Node was not in the map, then its destructor should not be in the set
    assert(destructors.find(destructor) == destructors.end());

    // If this Node believes its index is invalid, then it should not exist
    // anywhere in the vector
    assert(std::find(nodes.begin(), nodes.end(), this) == nodes.end());

    nodes.push_back(this);
    mIndexInBodyNode = nodes.size()-1;

    destructors.insert(destructor);
  }

  assert(std::find(nodes.begin(), nodes.end(), this) != nodes.end());
  assert(destructors.find(destructor) != destructors.end());

  const SkeletonPtr& skel = mBodyNode->getSkeleton();
  if(skel)
    skel->registerNode(this);

  mAmAttached = true;
}

//==============================================================================
void Node::stageForRemoval()
{
  if(nullptr == mBodyNode)
  {
    REPORT_INVALID_NODE(stageForRemoval);
    return;
  }

  // If we are in release mode, and the Node believes it is detached, then we
  // can shortcut this procedure.
#ifdef NDEBUG
  if(!mAmAttached)
    return;
#endif

  BodyNode::NodeMap::iterator it = mBodyNode->mNodeMap.find(typeid(*this));
  NodeDestructorPtr destructor = getOrCreateDestructor();

  BodyNode::NodeDestructorSet& destructors = mBodyNode->mNodeDestructors;

  if(mBodyNode->mNodeMap.end() == it)
  {
    // If the Node was not in the map, then its index should be invalid
    assert(INVALID_INDEX == mIndexInBodyNode);

    // If the Node was not in the map, then its destructor should not be in the set
    assert(destructors.find(destructor) == destructors.end());
    return;
  }

  BodyNode::NodeDestructorSet::iterator destructor_iter = destructors.find(destructor);
  // This Node's destructor should be in the set of destructors
  assert(destructors.end() != destructor_iter);

  std::vector<Node*>& nodes = it->second;

  // This Node's index in the vector should be referring to this Node
  assert(nodes[mIndexInBodyNode] == this);
  nodes.erase(nodes.begin() + mIndexInBodyNode);
  destructors.erase(destructor_iter);

  // Reset all the Node indices that have been altered
  for(std::size_t i=mIndexInBodyNode; i < nodes.size(); ++i)
    nodes[i]->mIndexInBodyNode = i;

  assert(std::find(nodes.begin(), nodes.end(), this) == nodes.end());

  const SkeletonPtr& skel = mBodyNode->getSkeleton();
  if(skel)
    skel->unregisterNode(this);

  mIndexInBodyNode = INVALID_INDEX;
  mAmAttached = false;
}

} // namespace dynamics
} // namespace dart
