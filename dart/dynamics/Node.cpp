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

#include "dart/dynamics/Node.h"
#include "dart/dynamics/BodyNode.h"

#define REPORT_INVALID_NODE( func )                                          \
  dterr << "[Node::" #func "] This Node was not constructed correctly. It "  \
        << "needs to specify a valid BodyNode pointer during construction. " \
        << "Please report this as a bug if it is not a custom node type!\n"; \
  assert(false);

namespace kido {
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

  return (mBodyNode->mNodeMap.find(this) == mBodyNode->mNodeMap.end());
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
Node::Node(ConstructNode_t, BodyNode* _bn)
  : mBodyNode(_bn)
{
  if(nullptr == mBodyNode)
  {
    REPORT_INVALID_NODE(Node);
    return;
  }

  attach();
}

//==============================================================================
Node::Node(ConstructBodyNode_t)
  : mBodyNode(nullptr)
{
  // BodyNodes do not get "attached" to themselves, so we do nothing here
}

//==============================================================================
Node::Node(ConstructAbstract_t)
{
  dterr << "[Node::Node] Your class implementation is calling the Node "
        << "constructor that is meant to be reserved for abstract classes!\n";
  assert(false);
}

//==============================================================================
void Node::attach()
{
  if(nullptr == mBodyNode)
  {
    REPORT_INVALID_NODE(attach);
    return;
  }

  if(mBodyNode->mNodeMap.end() == mBodyNode->mNodeMap.find(this))
    mBodyNode->mNodeMap[this] = getOrCreateDestructor();
}

//==============================================================================
void Node::stageForRemoval()
{
  if(nullptr == mBodyNode)
  {
    REPORT_INVALID_NODE(stageForRemoval);
    return;
  }

  const auto it = mBodyNode->mNodeMap.find(this);

  if(mBodyNode->mNodeMap.end() == it)
    return;

  mBodyNode->mNodeMap.erase(it);
}

//==============================================================================
void AccessoryNode::remove()
{
  stageForRemoval();
}

//==============================================================================
void AccessoryNode::reattach()
{
  attach();
}

//==============================================================================
AccessoryNode::AccessoryNode()
  : Node(ConstructAbstract)
{
  // Do nothing
}

} // namespace dynamics
} // namespace kido
