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

#pragma once

#include <dartsim_engine/command.hpp>
#include <dartsim_engine/event_bus.hpp>
#include <dartsim_engine/logger.hpp>
#include <dartsim_engine/object_manager.hpp>
#include <dartsim_engine/recorder.hpp>
#include <dartsim_engine/selection_manager.hpp>
#include <dartsim_engine/simulation_controller.hpp>

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <cstddef>

namespace dartsim {

/// Headless editor engine facade.
///
/// Owns the scene/object manager, selection, command history, simulation
/// control, and record/replay, and exposes a small surface that a GUI (or a
/// test) drives without touching the experimental World directly. The GUI is a
/// thin client over this engine; see docs/design/dartsim_gui_simulator.md.
class SimEngine
{
public:
  SimEngine();
  ~SimEngine();

  SimEngine(const SimEngine&) = delete;
  SimEngine& operator=(const SimEngine&) = delete;

  [[nodiscard]] ObjectManager& objects()
  {
    return m_objects;
  }
  [[nodiscard]] const ObjectManager& objects() const
  {
    return m_objects;
  }
  [[nodiscard]] SelectionManager& selection()
  {
    return m_selection;
  }
  [[nodiscard]] CommandManager& commands()
  {
    return m_commands;
  }
  [[nodiscard]] SimulationController& simulation()
  {
    return m_simulation;
  }
  [[nodiscard]] Recorder& recorder()
  {
    return m_recorder;
  }
  [[nodiscard]] Player& player()
  {
    return m_player;
  }
  [[nodiscard]] EventBus& events()
  {
    return m_events;
  }
  [[nodiscard]] Logger& logger()
  {
    return m_logger;
  }
  [[nodiscard]] const Logger& logger() const
  {
    return m_logger;
  }

  /// Run a command through the history and notify observers.
  void execute(std::unique_ptr<Command> command);
  bool undo();
  bool redo();
  [[nodiscard]] bool canEditScene() const;

  /// Select an object through the engine facade and notify observers when the
  /// selection actually changes.
  bool select(ObjectId id, bool additive = false);

  // Record & replay -----------------------------------------------------------

  /// Begin recording, capturing the current world as the first frame.
  void startRecording();
  void stopRecording();
  [[nodiscard]] bool isRecording() const;

  /// Copy the recorder's recording into the player for scrubbing.
  void loadRecordingIntoPlayer();
  /// Restore a recorded frame into the world (no re-simulation).
  bool replaySeek(std::size_t index);

  // Project I/O ---------------------------------------------------------------

  [[nodiscard]] bool saveProject(const std::string& path) const;
  bool loadProject(const std::string& path);

  /// Resolve current world transforms to renderable primitives.
  [[nodiscard]] std::vector<RenderItem> renderItems() const;

  /// Observer invoked whenever the scene or selection changes.
  void setOnChanged(std::function<void()> callback)
  {
    m_onChanged = std::move(callback);
  }

private:
  void notifyChanged();

  ObjectManager m_objects;
  SelectionManager m_selection;
  CommandManager m_commands;
  SimulationController m_simulation;
  Recorder m_recorder;
  Player m_player;
  EventBus m_events;
  Logger m_logger;
  std::function<void()> m_onChanged;
};

} // namespace dartsim
