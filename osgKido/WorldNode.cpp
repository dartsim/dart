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

#include <osg/NodeCallback>

#include "osgKido/WorldNode.h"
#include "osgKido/FrameNode.h"
#include "osgKido/EntityNode.h"

#include "kido/simulation/World.h"
#include "kido/dynamics/Skeleton.h"
#include "kido/dynamics/BodyNode.h"

namespace osgKido
{

class WorldNodeCallback : public osg::NodeCallback
{
public:

  virtual void operator()(osg::Node* node, osg::NodeVisitor* nv)
  {
    osg::ref_ptr<WorldNode> currentNode = dynamic_cast<WorldNode*>(node);

    if(currentNode)
      currentNode->refresh();

    traverse(node, nv);
  }
};

//==============================================================================
WorldNode::WorldNode(std::shared_ptr<kido::simulation::World> _world)
  : mWorld(_world),
    mSimulating(false),
    mNumStepsPerCycle(1),
    mViewer(nullptr)
{
  setUpdateCallback(new WorldNodeCallback);
}

//==============================================================================
void WorldNode::setWorld(std::shared_ptr<kido::simulation::World> _newWorld)
{
  mWorld = _newWorld;
}

//==============================================================================
std::shared_ptr<kido::simulation::World> WorldNode::getWorld() const
{
  return mWorld;
}

//==============================================================================
void WorldNode::refresh()
{
  customPreRefresh();

  clearChildUtilizationFlags();

  if(mSimulating)
  {
    for(size_t i=0; i<mNumStepsPerCycle; ++i)
    {
      customPreStep();
      mWorld->step();
      customPostStep();
    }
  }

  refreshSkeletons();
  refreshCustomFrames();

  clearUnusedNodes();

  customPostRefresh();
}

//==============================================================================
void WorldNode::customPreRefresh()
{
  // Do nothing
}

//==============================================================================
void WorldNode::customPostRefresh()
{
  // Do nothing
}

//==============================================================================
void WorldNode::customPreStep()
{
  // Do nothing
}

//==============================================================================
void WorldNode::customPostStep()
{
  // Do nothing
}

//==============================================================================
bool WorldNode::isSimulating() const
{
  return mSimulating;
}

//==============================================================================
void WorldNode::simulate(bool _on)
{
  mSimulating = _on;
}

//==============================================================================
void WorldNode::setNumStepsPerCycle(size_t _steps)
{
  mNumStepsPerCycle = _steps;
}

//==============================================================================
size_t WorldNode::getNumStepsPerCycle() const
{
  return mNumStepsPerCycle;
}

//==============================================================================
WorldNode::~WorldNode()
{
  // Do nothing
}

//==============================================================================
void WorldNode::setupViewer()
{
  // Do nothing
}

//==============================================================================
void WorldNode::clearChildUtilizationFlags()
{
  for(auto& node : mNodeToFrame)
    node.first->clearUtilization();

  for(auto& node : mNodeToEntity)
    node.first->clearUtilization();
}

//==============================================================================
void WorldNode::clearUnusedNodes()
{
  // Clear unusued FrameNodes
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

      removeChild(node->getParentFrameNode());
    }
  }
}

//==============================================================================
void WorldNode::refreshSkeletons()
{
  if(!mWorld)
    return;

  // Apply the recursive Frame refreshing functionality to the root BodyNode of
  // each Skeleton
  for(size_t i=0, end=mWorld->getNumSkeletons(); i<end; ++i)
    refreshBaseFrameNode(mWorld->getSkeleton(i)->getBodyNode(0));
}

//==============================================================================
void WorldNode::refreshCustomFrames()
{
  if(!mWorld)
    return;

  for(size_t i=0, end=mWorld->getNumSimpleFrames(); i<end; ++i)
    refreshBaseFrameNode(mWorld->getSimpleFrame(i).get());
}

//==============================================================================
void WorldNode::refreshBaseFrameNode(kido::dynamics::Frame* _frame)
{
  std::map<kido::dynamics::Frame*, FrameNode*>::iterator it =
      mFrameToNode.find(_frame);

  if(it == mFrameToNode.end())
  {
    createBaseFrameNode(_frame);
    return;
  }

  (it->second)->refresh(false, true);
}

//==============================================================================
void WorldNode::createBaseFrameNode(kido::dynamics::Frame* _frame)
{
  osg::ref_ptr<FrameNode> node = new FrameNode(_frame, this, false, true);

  mFrameToNode[_frame] = node.get();
  mNodeToFrame[node.get()] = _frame;

  addChild(node);
}

//==============================================================================
void WorldNode::refreshBaseEntityNode(kido::dynamics::Entity* _entity)
{
  std::map<kido::dynamics::Entity*, EntityNode*>::iterator it =
      mEntityToNode.find(_entity);

  if(it == mEntityToNode.end())
  {
    createBaseEntityNode(_entity);
    return;
  }

  (it->second)->getParentFrameNode()->refresh(false, false);
  (it->second)->refresh();
}

//==============================================================================
void WorldNode::createBaseEntityNode(kido::dynamics::Entity* _entity)
{
  kido::dynamics::Frame* parentFrame;

  if(_entity->isFrame())
  {
    parentFrame = dynamic_cast<kido::dynamics::Frame*>(_entity);
    if(nullptr == parentFrame)
    {
      dtwarn << "[WorldNode::createBaseEntityNode] _entity named '"
             << _entity->getName() << "' claimed to be a Frame, but failed to "
             << "be dynamically cast into one\n";
      parentFrame = kido::dynamics::Frame::World();
    }
  }
  else
    parentFrame = _entity->getParentFrame();

  osg::ref_ptr<FrameNode> parentFrameNode =
      new FrameNode(parentFrame, this, false, false);
  addChild(parentFrameNode);

  osg::ref_ptr<EntityNode> entityNode =
      new EntityNode(_entity, parentFrameNode);
  parentFrameNode->addChild(entityNode);

  mEntityToNode[_entity] = entityNode;
  mNodeToEntity[entityNode] = _entity;
}

} // namespace osgKido
