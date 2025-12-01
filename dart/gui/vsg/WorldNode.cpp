/*
 * Copyright (c) 2011-2025, The DART development contributors
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

#include "dart/gui/vsg/WorldNode.hpp"

#include <dart/dynamics/BodyNode.hpp>
#include <dart/dynamics/Frame.hpp>
#include <dart/dynamics/Skeleton.hpp>

#include <algorithm>
#include <deque>
#include <utility>

namespace dart::gui::vsg {

WorldNode::WorldNode(std::shared_ptr<dart::simulation::World> world)
  : mWorld(std::move(world)),
    mSimulating(false),
    mNumStepsPerCycle(1),
    mBuilder(::vsg::Builder::create()),
    mRoot(::vsg::Group::create())
{
}

void WorldNode::setWorld(std::shared_ptr<dart::simulation::World> world)
{
  mWorld = std::move(world);
}

std::shared_ptr<dart::simulation::World> WorldNode::getWorld() const
{
  return mWorld;
}

void WorldNode::simulate(bool on)
{
  mSimulating = on;
}

bool WorldNode::isSimulating() const
{
  return mSimulating;
}

void WorldNode::setNumStepsPerCycle(std::size_t steps)
{
  mNumStepsPerCycle = steps;
}

std::size_t WorldNode::getNumStepsPerCycle() const
{
  return mNumStepsPerCycle;
}

::vsg::ref_ptr<::vsg::Group> WorldNode::getGroup() const
{
  return mRoot;
}

void WorldNode::clearUtilization()
{
  for (auto& [_, node] : mShapeNodes) {
    if (node) {
      node->clearUtilization();
    }
  }
}

void WorldNode::refreshSkeletons()
{
  if (!mWorld) {
    return;
  }

  for (std::size_t i = 0; i < mWorld->getNumSkeletons(); ++i) {
    const auto& skeleton = mWorld->getSkeleton(i);
    if (!skeleton) {
      continue;
    }

    for (std::size_t tree = 0; tree < skeleton->getNumTrees(); ++tree) {
      refreshFrame(skeleton->getRootBodyNode(tree));
    }
  }
}

void WorldNode::refreshSimpleFrames()
{
  if (!mWorld) {
    return;
  }

  for (std::size_t i = 0; i < mWorld->getNumSimpleFrames(); ++i) {
    refreshFrame(mWorld->getSimpleFrame(i).get());
  }
}

void WorldNode::refreshFrame(dart::dynamics::Frame* frame)
{
  if (!frame) {
    return;
  }

  std::deque<dart::dynamics::Frame*> queue;
  queue.push_back(frame);

  while (!queue.empty()) {
    auto* current = queue.front();
    queue.pop_front();

    if (current->isShapeFrame()) {
      auto* shapeFrame = current->asShapeFrame();
      auto [it, inserted] = mShapeNodes.try_emplace(shapeFrame, nullptr);
      auto& node = it->second;
      if (!node) {
        node = std::make_unique<ShapeNode>(shapeFrame, mBuilder);
        mRoot->addChild(node->getNode());
      }
      node->refresh();
    }

    const auto& children = current->getChildFrames();
    queue.insert(queue.end(), children.begin(), children.end());
  }
}

void WorldNode::pruneUnused()
{
  auto& children = mRoot->children;
  for (auto it = mShapeNodes.begin(); it != mShapeNodes.end();) {
    const bool remove = !it->second || !it->second->wasUtilized();
    if (remove) {
      if (it->second && it->second->getNode()) {
        const auto nodeRef = it->second->getNode();
        children.erase(
            std::remove(children.begin(), children.end(), nodeRef),
            children.end());
      }
      it = mShapeNodes.erase(it);
    } else {
      ++it;
    }
  }
}

void WorldNode::refresh()
{
  clearUtilization();

  if (mSimulating && mWorld) {
    for (std::size_t i = 0; i < mNumStepsPerCycle; ++i) {
      mWorld->step();
    }
  }

  refreshSkeletons();
  refreshSimpleFrames();
  pruneUnused();
}

} // namespace dart::gui::vsg
