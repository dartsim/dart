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

#include <deque>

#include <osg/NodeCallback>

#include "dart/gui/osg/WorldNode.h"
#include "dart/gui/osg/ShapeFrameNode.h"

#include "dart/simulation/World.h"
#include "dart/dynamics/Skeleton.h"
#include "dart/dynamics/BodyNode.h"

namespace dart {
namespace gui {
namespace osg {

class WorldNodeCallback : public ::osg::NodeCallback
{
public:

  virtual void operator()(::osg::Node* node, ::osg::NodeVisitor* nv)
  {
    ::osg::ref_ptr<WorldNode> currentNode = dynamic_cast<WorldNode*>(node);

    if(currentNode)
      currentNode->refresh();

    traverse(node, nv);
  }
};

//==============================================================================
WorldNode::WorldNode(std::shared_ptr<dart::simulation::World> _world)
  : mWorld(_world),
    mSimulating(false),
    mNumStepsPerCycle(1),
    mViewer(nullptr)
{
  setUpdateCallback(new WorldNodeCallback);
}

//==============================================================================
void WorldNode::setWorld(std::shared_ptr<dart::simulation::World> _newWorld)
{
  mWorld = _newWorld;
}

//==============================================================================
std::shared_ptr<dart::simulation::World> WorldNode::getWorld() const
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
  refreshSimpleFrames();

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
  for(auto& node_pair : mFrameToNode)
    node_pair.second->clearUtilization();
}

//==============================================================================
void WorldNode::clearUnusedNodes()
{
  std::vector<dart::dynamics::Frame*> unused;
  unused.reserve(mFrameToNode.size());

  // Find unusued ShapeFrameNodes
  for(auto& node_pair : mFrameToNode)
  {
    ShapeFrameNode* node = node_pair.second;
    if(node && !node->wasUtilized())
      unused.push_back(node_pair.first);
  }

  // Clear unused ShapeFrameNodes
  for(dart::dynamics::Frame* frame : unused)
  {
    NodeMap::iterator it = mFrameToNode.find(frame);
    ShapeFrameNode* node = it->second;
    removeChild(node);
    mFrameToNode.erase(it);
  }
}

//==============================================================================
void WorldNode::refreshSkeletons()
{
  if(!mWorld)
    return;

  // Apply the recursive Frame refreshing functionality to the root BodyNode of
  // each Skeleton
  for(size_t i=0; i < mWorld->getNumSkeletons(); ++i)
  {
    const dart::dynamics::SkeletonPtr& skeleton = mWorld->getSkeleton(i);
    for(size_t i=0; i < skeleton->getNumTrees(); ++i)
    {
      refreshBaseFrameNode(skeleton->getRootBodyNode(i));
    }
  }
}

//==============================================================================
void WorldNode::refreshSimpleFrames()
{
  if(!mWorld)
    return;

  for(size_t i=0, end=mWorld->getNumSimpleFrames(); i<end; ++i)
    refreshBaseFrameNode(mWorld->getSimpleFrame(i).get());
}

//==============================================================================
void WorldNode::refreshBaseFrameNode(dart::dynamics::Frame* frame)
{
  std::deque<dart::dynamics::Frame*> frames;
  frames.push_back(frame);
  while(!frames.empty())
  {
    dart::dynamics::Frame* nextFrame = frames.front();
    frames.pop_front();
    if(nextFrame->isShapeFrame())
      refreshShapeFrameNode(nextFrame);

    const std::set<dart::dynamics::Frame*>& childFrames =
        nextFrame->getChildFrames();

    for(dart::dynamics::Frame* child : childFrames)
      frames.push_back(child);
  }
}

//==============================================================================
void WorldNode::refreshShapeFrameNode(dart::dynamics::Frame* frame)
{
  std::pair<NodeMap::iterator, bool> insertion =
      mFrameToNode.insert(std::make_pair(frame, nullptr));
  NodeMap::iterator it = insertion.first;
  bool inserted = insertion.second;

  if(!inserted)
  {
    ShapeFrameNode* node = it->second;
    if(!node)
      return;

    node->refresh(true);
    return;
  }

  if(!frame->isShapeFrame())
  {
    dtwarn << "[WorldNode::refreshShapeFrameNode] Frame named ["
           << frame->getName() << "] (" << frame << ") claims to be a "
           << "ShapeFrame, but failed to be converted. Please report this as a "
           << "bug!\n";
    return;
  }

  ::osg::ref_ptr<ShapeFrameNode> node = new ShapeFrameNode(frame->asShapeFrame(),
                                                         this);
  it->second = node;
  addChild(node);
}

} // namespace osg
} // namespace gui
} // namespace dart
