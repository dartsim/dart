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

#include "WorldHistory.hpp"

#include <dart/dynamics/BodyNode.hpp>

#include <algorithm>
#include <stdexcept>
#include <utility>

namespace dart_demos {

using dart::dynamics::SkeletonPtr;

//==============================================================================
WorldHistory::WorldHistory(
    dart::simulation::WorldPtr world, std::size_t maxFrames)
  : mWorld(std::move(world)), mMaxFrames(maxFrames)
{
  if (!mWorld)
    throw std::invalid_argument("world must not be null");
  if (mMaxFrames == 0)
    throw std::invalid_argument("max_frames must be positive");
  mHistory.push_back(grabTimeSlice());
}

//==============================================================================
WorldHistory::TimeSlice WorldHistory::grabTimeSlice() const
{
  TimeSlice slice;
  slice.reserve(mWorld->getNumSkeletons());

  for (std::size_t i = 0; i < mWorld->getNumSkeletons(); ++i) {
    const SkeletonPtr& skeleton = mWorld->getSkeleton(i);
    HistoryState state;
    state.config = skeleton->getConfiguration();
    state.bodyStates.reserve(skeleton->getNumBodyNodes());
    for (std::size_t j = 0; j < skeleton->getNumBodyNodes(); ++j)
      state.bodyStates.push_back(skeleton->getBodyNode(j)->getCompositeState());
    slice.push_back(std::move(state));
  }

  return slice;
}

//==============================================================================
void WorldHistory::record()
{
  // Resuming after a rewind discards the old future before appending.
  if (mCurrentIndex + 1 < mHistory.size())
    mHistory.resize(mCurrentIndex + 1);

  mHistory.push_back(grabTimeSlice());
  ++mFramesRecorded;
  ++mCurrentIndex;

  if (mHistory.size() > mMaxFrames) {
    mHistory.pop_front();
    --mCurrentIndex;
  }
}

//==============================================================================
void WorldHistory::moveTo(std::size_t index)
{
  if (mHistory.empty())
    return;

  if (index >= mHistory.size())
    index = mHistory.size() - 1;

  const TimeSlice& slice = mHistory[index];
  for (std::size_t i = 0; i < slice.size() && i < mWorld->getNumSkeletons();
       ++i) {
    const HistoryState& saved = slice[i];
    const SkeletonPtr& skeleton = mWorld->getSkeleton(i);
    skeleton->setConfiguration(saved.config);
    for (std::size_t j = 0;
         j < skeleton->getNumBodyNodes() && j < saved.bodyStates.size();
         ++j)
      skeleton->getBodyNode(j)->setCompositeState(saved.bodyStates[j]);
  }

  mCurrentIndex = index;
}

//==============================================================================
void WorldHistory::moveForward(std::size_t delta)
{
  const std::size_t remaining = mHistory.size() - 1 - mCurrentIndex;
  moveTo(mCurrentIndex + std::min(delta, remaining));
}

//==============================================================================
void WorldHistory::moveBackward(std::size_t delta)
{
  const std::size_t target = delta > mCurrentIndex ? 0 : mCurrentIndex - delta;
  moveTo(target);
}

//==============================================================================
const dart::simulation::WorldPtr& WorldHistory::getWorld() const
{
  return mWorld;
}

//==============================================================================
std::size_t WorldHistory::getCurrentIndex() const
{
  return mCurrentIndex;
}

//==============================================================================
std::size_t WorldHistory::getSize() const
{
  return mHistory.size();
}

//==============================================================================
std::size_t WorldHistory::getFramesRecorded() const
{
  return mFramesRecorded;
}

//==============================================================================
std::size_t WorldHistory::getMaxFrames() const
{
  return mMaxFrames;
}

} // namespace dart_demos
