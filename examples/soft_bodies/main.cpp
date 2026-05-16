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

#include <dart/gui/application.hpp>
#include <dart/gui/panel.hpp>
#include <dart/gui/viewer.hpp>

#include <dart/simulation/world.hpp>

#include <dart/dynamics/body_node.hpp>
#include <dart/dynamics/skeleton.hpp>

#include <dart/common/composite.hpp>

#include <dart/io/read.hpp>

#include <algorithm>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <cstddef>

namespace {

constexpr const char* kWorldUri = "dart://sample/skel/softBodies.skel";

std::string makeStatusLine(const char* label, std::size_t value)
{
  std::ostringstream stream;
  stream << label << value;
  return stream.str();
}

class SoftBodyHistory
{
public:
  explicit SoftBodyHistory(dart::simulation::WorldPtr world)
    : mWorld(std::move(world))
  {
    captureCurrentState();
  }

  void captureStepStart()
  {
    if (mHistory.empty()) {
      captureCurrentState();
      return;
    }

    if (mCurrentIndex + 1 < mHistory.size()) {
      mHistory.resize(mCurrentIndex + 1);
    }

    captureCurrentState();
    mCurrentIndex = mHistory.size() - 1;
  }

  void moveBackward(std::size_t delta)
  {
    if (mHistory.empty()) {
      return;
    }

    restoreIndex(mCurrentIndex > delta ? mCurrentIndex - delta : 0);
  }

  void moveForward(std::size_t delta)
  {
    if (mHistory.empty()) {
      return;
    }

    restoreIndex(std::min(mCurrentIndex + delta, mHistory.size() - 1));
  }

  void restart()
  {
    restoreIndex(0);
  }

  void moveToEnd()
  {
    if (!mHistory.empty()) {
      restoreIndex(mHistory.size() - 1);
    }
  }

  std::size_t historySize() const
  {
    return mHistory.size();
  }

  std::size_t currentIndex() const
  {
    return mCurrentIndex;
  }

private:
  struct SkeletonState
  {
    dart::dynamics::Skeleton::Configuration configuration;
    std::vector<dart::common::Composite::State> bodyStates;
  };

  using TimeSlice = std::vector<SkeletonState>;

  void captureCurrentState()
  {
    TimeSlice slice;
    slice.reserve(mWorld->getNumSkeletons());

    for (std::size_t i = 0; i < mWorld->getNumSkeletons(); ++i) {
      const auto& skeleton = mWorld->getSkeleton(i);
      SkeletonState state;
      state.configuration = skeleton->getConfiguration();
      state.bodyStates.reserve(skeleton->getNumBodyNodes());

      for (std::size_t j = 0; j < skeleton->getNumBodyNodes(); ++j) {
        state.bodyStates.push_back(
            skeleton->getBodyNode(j)->getCompositeState());
      }

      slice.push_back(std::move(state));
    }

    mHistory.push_back(std::move(slice));
  }

  void restoreIndex(std::size_t index)
  {
    if (mHistory.empty()) {
      return;
    }

    index = std::min(index, mHistory.size() - 1);
    const TimeSlice& slice = mHistory[index];
    const std::size_t skeletonCount
        = std::min(slice.size(), mWorld->getNumSkeletons());

    for (std::size_t i = 0; i < skeletonCount; ++i) {
      const auto& state = slice[i];
      const auto& skeleton = mWorld->getSkeleton(i);
      skeleton->setConfiguration(state.configuration);

      const std::size_t bodyCount
          = std::min(state.bodyStates.size(), skeleton->getNumBodyNodes());
      for (std::size_t j = 0; j < bodyCount; ++j) {
        skeleton->getBodyNode(j)->setCompositeState(state.bodyStates[j]);
      }
    }

    mCurrentIndex = index;
  }

  dart::simulation::WorldPtr mWorld;
  std::vector<TimeSlice> mHistory;
  std::size_t mCurrentIndex = 0;
};

dart::simulation::WorldPtr createSoftBodiesWorld()
{
  auto world = dart::io::readWorld(kWorldUri);
  if (world == nullptr) {
    throw std::runtime_error(
        "Failed to load soft_bodies world from " + std::string(kWorldUri));
  }

  return world;
}

dart::gui::Panel createSoftBodiesPanel(
    const std::shared_ptr<SoftBodyHistory>& history)
{
  dart::gui::Panel panel;
  panel.title = "Soft Bodies";
  panel.buildWithContext = [history](
                               dart::gui::PanelBuilder& builder,
                               dart::gui::PanelContext& context) {
    builder.text("Recorded soft-body state playback");
    builder.text(makeStatusLine("history frames: ", history->historySize()));
    builder.text(makeStatusLine("current frame: ", history->currentIndex()));
    builder.separator();
    if (context.lifecycle != nullptr) {
      if (builder.button(context.lifecycle->paused ? "Resume" : "Pause")) {
        dart::gui::togglePaused(*context.lifecycle);
      }
      builder.sameLine();
      if (builder.button("Step")) {
        dart::gui::requestSingleStep(*context.lifecycle);
      }
    }
    if (builder.button("Restart")) {
      history->restart();
    }
    builder.sameLine();
    if (builder.button("-10")) {
      history->moveBackward(10);
    }
    builder.sameLine();
    if (builder.button("-1")) {
      history->moveBackward(1);
    }
    builder.sameLine();
    if (builder.button("+1")) {
      history->moveForward(1);
    }
    builder.sameLine();
    if (builder.button("+10")) {
      history->moveForward(10);
    }
    builder.sameLine();
    if (builder.button("Latest")) {
      history->moveToEnd();
    }
  };
  return panel;
}

} // namespace

int main(int argc, char* argv[])
{
  auto world = createSoftBodiesWorld();
  auto history = std::make_shared<SoftBodyHistory>(world);

  dart::gui::ApplicationOptions options;
  options.world = std::move(world);
  options.preStep = [history]() {
    history->captureStepStart();
  };
  options.panels.push_back(createSoftBodiesPanel(history));

  return dart::gui::runApplication(argc, argv, options);
}
