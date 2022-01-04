/*
 * Copyright (c) 2011-2022, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/master/LICENSE
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

#include "dart/gui/osg/WorldNode.hpp"

#include <deque>

#include <osg/NodeCallback>
#include <osgShadow/ShadowMap>
#include <osgShadow/ShadowedScene>

#include "dart/dynamics/BodyNode.hpp"
#include "dart/dynamics/Skeleton.hpp"
#include "dart/gui/osg/ShapeFrameNode.hpp"
#include "dart/simulation/World.hpp"

namespace dart {
namespace gui {
namespace osg {

class WorldNodeCallback : public ::osg::NodeCallback
{
public:
  virtual void operator()(::osg::Node* node, ::osg::NodeVisitor* nv)
  {
    ::osg::ref_ptr<WorldNode> currentNode = dynamic_cast<WorldNode*>(node);

    if (currentNode)
      currentNode->refresh();

    traverse(node, nv);
  }
};

//==============================================================================
WorldNode::WorldNode(
    std::shared_ptr<dart::simulation::World> world,
    ::osg::ref_ptr<osgShadow::ShadowTechnique> shadowTechnique)
  : mWorld(world),
    mSimulating(false),
    mNumStepsPerCycle(1),
    mViewer(nullptr),
    mNormalGroup(new ::osg::Group)
{
  // Flags for shadowing; maybe this needs to be global?
  constexpr int ReceivesShadowTraversalMask = 0x2;
  constexpr int CastsShadowTraversalMask = 0x1;

  // Setup shadows
  // Create a ShadowedScene
  ::osg::ref_ptr<osgShadow::ShadowedScene> shadowedScene
      = new osgShadow::ShadowedScene;
  shadowedScene->setReceivesShadowTraversalMask(ReceivesShadowTraversalMask);
  shadowedScene->setCastsShadowTraversalMask(CastsShadowTraversalMask);

  // set the shadowed group
  mShadowedGroup = shadowedScene.get();
  mShadowedGroup->getOrCreateStateSet();

  // Add normal and shadowed groups
  addChild(mNormalGroup);
  addChild(mShadowedGroup);

  setShadowTechnique(shadowTechnique);

  setUpdateCallback(new WorldNodeCallback);
}

//==============================================================================
void WorldNode::setWorld(std::shared_ptr<dart::simulation::World> newWorld)
{
  mWorld = newWorld;
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

  if (mSimulating)
  {
    for (std::size_t i = 0; i < mNumStepsPerCycle; ++i)
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
void WorldNode::simulate(bool on)
{
  mSimulating = on;
}

//==============================================================================
void WorldNode::setNumStepsPerCycle(std::size_t steps)
{
  mNumStepsPerCycle = steps;
}

//==============================================================================
std::size_t WorldNode::getNumStepsPerCycle() const
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
  for (auto& node_pair : mFrameToNode)
    node_pair.second->clearUtilization();
}

//==============================================================================
void WorldNode::clearUnusedNodes()
{
  std::vector<dart::dynamics::Frame*> unused;
  unused.reserve(mFrameToNode.size());

  // Find unusued ShapeFrameNodes
  for (auto& node_pair : mFrameToNode)
  {
    ShapeFrameNode* node = node_pair.second;
    if (node && !node->wasUtilized())
      unused.push_back(node_pair.first);
  }

  // Clear unused ShapeFrameNodes
  for (dart::dynamics::Frame* frame : unused)
  {
    NodeMap::iterator it = mFrameToNode.find(frame);
    ShapeFrameNode* node = it->second;
    mNormalGroup->removeChild(node);
    mShadowedGroup->removeChild(node);
    mFrameToNode.erase(it);
  }
}

//==============================================================================
void WorldNode::refreshSkeletons()
{
  if (!mWorld)
    return;

  // Apply the recursive Frame refreshing functionality to the root BodyNode of
  // each Skeleton
  for (std::size_t i = 0; i < mWorld->getNumSkeletons(); ++i)
  {
    const dart::dynamics::SkeletonPtr& skeleton = mWorld->getSkeleton(i);
    for (std::size_t i = 0; i < skeleton->getNumTrees(); ++i)
    {
      refreshBaseFrameNode(skeleton->getRootBodyNode(i));
    }
  }
}

//==============================================================================
void WorldNode::refreshSimpleFrames()
{
  if (!mWorld)
    return;

  for (std::size_t i = 0, end = mWorld->getNumSimpleFrames(); i < end; ++i)
    refreshBaseFrameNode(mWorld->getSimpleFrame(i).get());
}

//==============================================================================
void WorldNode::refreshBaseFrameNode(dart::dynamics::Frame* frame)
{
  std::deque<dart::dynamics::Frame*> frames;
  frames.push_back(frame);
  while (!frames.empty())
  {
    dart::dynamics::Frame* nextFrame = frames.front();
    frames.pop_front();
    if (nextFrame->isShapeFrame())
      refreshShapeFrameNode(nextFrame);

    const std::set<dart::dynamics::Frame*>& childFrames
        = nextFrame->getChildFrames();

    for (dart::dynamics::Frame* child : childFrames)
      frames.push_back(child);
  }
}

//==============================================================================
void WorldNode::refreshShapeFrameNode(dart::dynamics::Frame* frame)
{
  std::pair<NodeMap::iterator, bool> insertion
      = mFrameToNode.insert(std::make_pair(frame, nullptr));
  NodeMap::iterator it = insertion.first;
  bool inserted = insertion.second;

  if (!inserted)
  {
    ShapeFrameNode* node = it->second;
    if (!node)
      return;

    node->refresh(true);

    // update the group that ShapeFrameNode should be
    if ((!node->getShapeFrame()->hasVisualAspect()
         || !node->getShapeFrame()->getVisualAspect(true)->getShadowed())
        && node->getParent(0) != mNormalGroup)
    {
      mShadowedGroup->removeChild(node);
      mNormalGroup->addChild(node);
    }
    else if (
        node->getShapeFrame()->hasVisualAspect()
        && node->getShapeFrame()->getVisualAspect(true)->getShadowed()
        && node->getParent(0) != mShadowedGroup)
    {
      mNormalGroup->removeChild(node);
      mShadowedGroup->addChild(node);
    }

    return;
  }

  if (!frame->isShapeFrame())
  {
    dtwarn << "[WorldNode::refreshShapeFrameNode] Frame named ["
           << frame->getName() << "] (" << frame << ") claims to be a "
           << "ShapeFrame, but failed to be converted. Please report this as a "
           << "bug!\n";
    return;
  }

  ::osg::ref_ptr<ShapeFrameNode> node
      = new ShapeFrameNode(frame->asShapeFrame(), this);
  it->second = node;
  if (!node->getShapeFrame()->hasVisualAspect()
      || !node->getShapeFrame()->getVisualAspect(true)->getShadowed())
  {
    mNormalGroup->addChild(node);
  }
  else
    mShadowedGroup->addChild(node);
}

//==============================================================================
bool WorldNode::isShadowed() const
{
  return mShadowed;
}

//==============================================================================
void WorldNode::setShadowTechnique(
    ::osg::ref_ptr<osgShadow::ShadowTechnique> shadowTechnique)
{
  if (!shadowTechnique)
  {
    mShadowed = false;
    mShadowedGroup->setShadowTechnique(nullptr);
  }
  else
  {
    mShadowed = true;
    mShadowedGroup->setShadowTechnique(shadowTechnique);
  }
}

//==============================================================================
::osg::ref_ptr<osgShadow::ShadowTechnique> WorldNode::getShadowTechnique() const
{
  if (!mShadowed)
    return nullptr;
  return mShadowedGroup->getShadowTechnique();
}

//==============================================================================
::osg::ref_ptr<osgShadow::ShadowTechnique>
WorldNode::createDefaultShadowTechnique(const Viewer* viewer)
{
  assert(viewer);

  ::osg::ref_ptr<osgShadow::ShadowMap> sm = new osgShadow::ShadowMap;
  // increase the resolution of default shadow texture for higher quality
  auto mapres = static_cast<short>(std::pow(2, 12));
  sm->setTextureSize(::osg::Vec2s(mapres, mapres));
  // we are using Light1 because this is the highest one (on up direction)
  sm->setLight(viewer->getLightSource(0));

  return sm;
}

} // namespace osg
} // namespace gui
} // namespace dart
