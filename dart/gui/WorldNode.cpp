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

#include <dart/gui/Viewer.hpp>
#include <dart/gui/WorldNode.hpp>

#include <dart/simulation/World.hpp>

#include <utility>

namespace dart {
namespace gui {
WorldNode::WorldNode(std::shared_ptr<simulation::World> world)
  : mWorld(std::move(world))
{
}
WorldNode::~WorldNode() = default;
void WorldNode::setWorld(std::shared_ptr<simulation::World> world)
{
  mWorld = std::move(world);
}
std::shared_ptr<simulation::World> WorldNode::getWorld() const
{
  return mWorld;
}
void WorldNode::refresh()
{
  customPreRefresh();
  if (mWorld && mSimulating && canStep()) {
    for (std::size_t i = 0; i < mNumStepsPerCycle; ++i) {
      const auto world = mWorld;
      customPreStep();
      if (!mWorld || mWorld != world || !mSimulating || !canStep())
        break;
      world->step();
      customPostStep();
    }
  }
  customPostRefresh();
}
void WorldNode::refreshForViewer(bool mayStep)
{
  const bool previous = mMayStep;
  const bool previouslyRefreshing = mRefreshingForViewer;
  mMayStep = mayStep;
  mRefreshingForViewer = true;
  try {
    refresh();
  } catch (...) {
    mMayStep = previous;
    mRefreshingForViewer = previouslyRefreshing;
    throw;
  }
  mMayStep = previous;
  mRefreshingForViewer = previouslyRefreshing;
}
bool WorldNode::canStep()
{
  return mMayStep
         && (!mRefreshingForViewer
             || (mViewer && mViewer->ownsWorldSteps(this)));
}
void WorldNode::customPreRefresh() {}
void WorldNode::customPostRefresh() {}
void WorldNode::customPreStep() {}
void WorldNode::customPostStep() {}
void WorldNode::setupViewer() {}
bool WorldNode::isSimulating() const
{
  return mSimulating;
}
void WorldNode::simulate(bool on)
{
  mSimulating = on;
}
void WorldNode::setNumStepsPerCycle(std::size_t steps)
{
  mNumStepsPerCycle = steps;
}
std::size_t WorldNode::getNumStepsPerCycle() const
{
  return mNumStepsPerCycle;
}
void WorldNode::setShadowsEnabled(bool enabled)
{
  mShadowed = enabled;
}
bool WorldNode::isShadowed() const
{
  return mShadowed;
}
Viewer* WorldNode::getViewer() const
{
  return mViewer;
}
} // namespace gui
} // namespace dart
