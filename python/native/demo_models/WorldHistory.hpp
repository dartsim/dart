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

#ifndef DART_PYTHON_NATIVE_DEMO_MODELS_WORLDHISTORY_HPP_
#define DART_PYTHON_NATIVE_DEMO_MODELS_WORLDHISTORY_HPP_

#include <dart/simulation/World.hpp>

#include <dart/dynamics/Skeleton.hpp>

#include <dart/common/Composite.hpp>

#include <deque>
#include <vector>

#include <cstddef>

namespace dart_demos {

/// Bounded full-state playback used by the soft-body catalog scenes.
class WorldHistory
{
public:
  static constexpr std::size_t kMaxHistoryFrames = 4000;

  explicit WorldHistory(
      dart::simulation::WorldPtr world,
      std::size_t maxFrames = kMaxHistoryFrames);

  void record();
  void moveTo(std::size_t index);
  void moveForward(std::size_t delta = 1);
  void moveBackward(std::size_t delta = 1);

  const dart::simulation::WorldPtr& getWorld() const;
  std::size_t getCurrentIndex() const;
  std::size_t getSize() const;
  std::size_t getFramesRecorded() const;
  std::size_t getMaxFrames() const;

private:
  struct HistoryState
  {
    dart::dynamics::Skeleton::Configuration config;
    std::vector<dart::common::Composite::State> bodyStates;
  };
  using TimeSlice = std::vector<HistoryState>;

  TimeSlice grabTimeSlice() const;

  dart::simulation::WorldPtr mWorld;
  std::deque<TimeSlice> mHistory;
  std::size_t mCurrentIndex = 0;
  std::size_t mFramesRecorded = 1;
  std::size_t mMaxFrames;
};

} // namespace dart_demos

#endif // DART_PYTHON_NATIVE_DEMO_MODELS_WORLDHISTORY_HPP_
