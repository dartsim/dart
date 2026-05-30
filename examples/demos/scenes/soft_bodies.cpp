/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * This file is provided under the "BSD-style" License.
 */

#include "scenes.hpp"
#include "z_up.hpp"

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
#include <utility>
#include <vector>

#include <cstddef>

namespace dart::examples::demos {

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

  // softBodies.skel is authored Y-up; reorient to the canonical Z-up
  // convention.
  reorientWorldToZUp(world);

  return world;
}

template <typename Callback>
dart::gui::KeyboardAction makePlaybackAction(
    const std::shared_ptr<SoftBodyHistory>& history,
    char key,
    std::string label,
    Callback callback)
{
  dart::gui::KeyboardAction action;
  action.label = std::move(label);
  action.shortcut = dart::gui::KeyboardShortcut::characterKey(key);
  action.callback = [history, callback = std::move(callback)](
                        dart::gui::KeyboardActionContext&) {
    callback(*history);
  };
  return action;
}

std::vector<dart::gui::KeyboardAction> createSoftBodiesKeyboardActions(
    const std::shared_ptr<SoftBodyHistory>& history)
{
  std::vector<dart::gui::KeyboardAction> actions;
  actions.reserve(6);
  actions.push_back(makePlaybackAction(
      history,
      '[',
      "Move soft-body playback backward one frame",
      [](SoftBodyHistory& state) { state.moveBackward(1); }));
  actions.push_back(makePlaybackAction(
      history,
      ']',
      "Move soft-body playback forward one frame",
      [](SoftBodyHistory& state) { state.moveForward(1); }));
  actions.push_back(makePlaybackAction(
      history,
      '{',
      "Move soft-body playback backward ten frames",
      [](SoftBodyHistory& state) { state.moveBackward(10); }));
  actions.push_back(makePlaybackAction(
      history,
      '}',
      "Move soft-body playback forward ten frames",
      [](SoftBodyHistory& state) { state.moveForward(10); }));
  actions.push_back(makePlaybackAction(
      history, 'r', "Restart soft-body playback", [](SoftBodyHistory& state) {
        state.restart();
      }));
  actions.push_back(makePlaybackAction(
      history,
      '\\',
      "Jump soft-body playback to latest frame",
      [](SoftBodyHistory& state) { state.moveToEnd(); }));
  return actions;
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
    builder.text("'['/']': move backward/forward one frame");
    builder.text("'{'/'}': move backward/forward ten frames");
    builder.text("'r': restart playback");
    builder.text("'\\': jump to latest frame");
    builder.separator();
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

dart::gui::ApplicationOptions makeSoftBodiesScene()
{
  auto world = createSoftBodiesWorld();
  auto history = std::make_shared<SoftBodyHistory>(world);

  dart::gui::ApplicationOptions options;
  options.world = std::move(world);
  options.preStep = [history]() {
    history->captureStepStart();
  };
  options.keyboardActions = createSoftBodiesKeyboardActions(history);
  options.panels.push_back(createSoftBodiesPanel(history));
  return options;
}

} // namespace dart::examples::demos
