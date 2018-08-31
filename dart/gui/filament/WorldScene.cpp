/*
 * Copyright (c) 2011-2018, The DART development contributors
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

#include "dart/gui/filament/WorldScene.hpp"

#include <deque>

#include <utils/EntityManager.h>

#include "dart/dynamics/BodyNode.hpp"

namespace dart {
namespace gui {
namespace flmt {

WorldScene::WorldScene(simulation::WorldPtr world)
  : mWorld(std::move(world))
{
  // Do nothing
}

WorldScene::~WorldScene()
{
  // Do nothing
}

void WorldScene::setScene(filament::Engine* engine, filament::Scene* scene)
{
  if (scene == mScene)
    return;

  mEngine = engine;
  mScene = scene;
  if (mScene)
  {
    refresh();
  }
  else
  {
    // TODO(JS): clear?
  }
}

void WorldScene::refresh()
{
  refreshSkeletons();
  clearUnusedNodes();
}

void WorldScene::refreshSkeletons()
{
  if (!mWorld)
    return;

  for (auto i = 0u; i < mWorld->getNumSkeletons(); ++i)
  {
    const dynamics::SkeletonPtr& skel = mWorld->getSkeleton(i);
    for (auto j = 0u; j < skel->getNumTrees(); ++j)
      refreshBaseFrameNode(skel->getRootBodyNode(j));
  }
}

void WorldScene::refreshBaseFrameNode(dynamics::Frame* frame)
{
  std::deque<dynamics::Frame*> frames;
  frames.push_back(frame);
  while (!frames.empty())
  {
    dynamics::Frame* nextFrame = frames.front();
    frames.pop_front();

    if (nextFrame->isShapeFrame())
      refreshShapeFrameNode(nextFrame);

    const std::set<dynamics::Frame*>& childFrames = nextFrame->getChildFrames();
    for (auto* child : childFrames)
      frames.push_back(child);
  }
}

void WorldScene::refreshShapeFrameNode(dynamics::Frame* frame)
{
  std::pair<NodeMap::iterator, bool> insertion
      = mFrameToNode.insert(std::make_pair(frame, nullptr));
  NodeMap::iterator& itr = insertion.first;
  bool exist = !insertion.second;

  if (exist)
  {
    ShapeFrameEntity* sfEntity = itr->second.get();
    if (!sfEntity)
      return;

    sfEntity->refresh(true);

    return;
  }

  assert(frame->isShapeFrame());

  auto entity = ::utils::EntityManager::get().create();
  std::unique_ptr<ShapeFrameEntity> sfEntity
      = common::make_unique<ShapeFrameEntity>(this, frame->asShapeFrame(), entity);

  itr->second = std::move(sfEntity);
  mScene->addEntity(entity);
}

void WorldScene::clearUnusedNodes()
{

}

} // namespace flmt
} // namespace gui
} // namespace dart
