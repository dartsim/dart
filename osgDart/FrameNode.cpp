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

#include "osgDart/FrameNode.h"
#include "osgDart/EntityNode.h"
#include "osgDart/Utils.h"

#include "dart/dynamics/Frame.h"

namespace osgDart
{

FrameNode::FrameNode(kido::dynamics::Frame* _frame, WorldNode* _worldNode,
                     bool _relative, bool _recursive)
  : mFrame(_frame),
    mWorldNode(_worldNode),
    mUtilized(false)
{
  refresh(_relative, _recursive);
  setName(_frame->getName()+" [frame]");
}

//==============================================================================
kido::dynamics::Frame* FrameNode::getFrame() const
{
  return mFrame;
}

//==============================================================================
WorldNode* FrameNode::getWorldNode()
{
  return mWorldNode;
}

//==============================================================================
const WorldNode* FrameNode::getWorldNode() const
{
  return mWorldNode;
}

//==============================================================================
void FrameNode::refresh(bool _relative, bool _recursive)
{
  mUtilized = true;

  if(_relative)
    setMatrix(eigToOsgMatrix(mFrame->getRelativeTransform()));
  else
    setMatrix(eigToOsgMatrix(mFrame->getWorldTransform()));

  if(!_recursive)
    return;

  clearChildUtilizationFlags();

  refreshEntityNode(mFrame);

  const std::set<kido::dynamics::Entity*>& entities =
      mFrame->getChildEntities();

  for(kido::dynamics::Entity* entity : entities)
    if(!entity->isFrame())
      refreshEntityNode(entity);

  const std::set<kido::dynamics::Frame*>& frames =
      mFrame->getChildFrames();

  for(kido::dynamics::Frame* frame : frames)
    refreshFrameNode(frame);

  clearUnusedNodes();
}

//==============================================================================
bool FrameNode::wasUtilized() const
{
  return mUtilized;
}

//==============================================================================
void FrameNode::clearUtilization()
{
  mUtilized = false;
}

//==============================================================================
FrameNode::~FrameNode()
{
  // Do nothing
}

//==============================================================================
void FrameNode::clearChildUtilizationFlags()
{
  for(auto& node : mNodeToFrame)
    node.first->clearUtilization();

  for(auto& node : mNodeToEntity)
    node.first->clearUtilization();
}

//==============================================================================
void FrameNode::clearUnusedNodes()
{
  for(auto& node_pair : mNodeToFrame)
  {
    FrameNode* node = node_pair.first;
    if(!node->wasUtilized())
    {
      mNodeToFrame.erase(node);
      mFrameToNode.erase(node_pair.second);

      removeChild(node);
    }
  }

  for(auto& node_pair : mNodeToEntity)
  {
    EntityNode* node = node_pair.first;
    if(!node->wasUtilized())
    {
      mNodeToEntity.erase(node);
      mEntityToNode.erase(node_pair.second);

      removeChild(node);
    }
  }
}

//==============================================================================
void FrameNode::refreshFrameNode(kido::dynamics::Frame* _frame)
{
  std::map<kido::dynamics::Frame*, FrameNode*>::iterator it =
      mFrameToNode.find(_frame);

  if(it == mFrameToNode.end())
  {
    createFrameNode(_frame);
    return;
  }

  (it->second)->refresh(true, true);
}

//==============================================================================
void FrameNode::createFrameNode(kido::dynamics::Frame* _frame)
{
  osg::ref_ptr<FrameNode> node = new FrameNode(_frame, mWorldNode, true, true);

  mFrameToNode[_frame] = node.get();
  mNodeToFrame[node.get()] = _frame;

  addChild(node);
}

//==============================================================================
void FrameNode::refreshEntityNode(kido::dynamics::Entity* _entity)
{
  std::map<kido::dynamics::Entity*, EntityNode*>::iterator it =
      mEntityToNode.find(_entity);

  if(it == mEntityToNode.end())
  {
    createEntityNode(_entity);
    return;
  }

  (it->second)->refresh();
}

//==============================================================================
void FrameNode::createEntityNode(kido::dynamics::Entity* _entity)
{
  osg::ref_ptr<EntityNode> node = new EntityNode(_entity, this);

  mEntityToNode[_entity] = node.get();
  mNodeToEntity[node.get()] = _entity;

  addChild(node);
}

} // namespace osgDart
