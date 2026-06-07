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

#include <dartsim_engine/scene_model.hpp>

#include <functional>

#include <cstddef>

namespace dartsim {

class ObjectManager;

/// Owns the Edit/Simulation mode and stepping loop over the DART 7 World.
///
/// Entering Simulation Mode captures the authored edit state so Reset can
/// restore it (the DART 7 World's default synchronous stepping is the
/// reference behavior; async stepping is intentionally not used).
class SimulationController
{
public:
  enum class Mode
  {
    Edit,
    Simulation
  };

  explicit SimulationController(ObjectManager& objects);

  [[nodiscard]] Mode mode() const
  {
    return m_mode;
  }
  [[nodiscard]] bool isRunning() const
  {
    return m_running;
  }
  [[nodiscard]] bool hasCapturedEditState() const
  {
    return m_haveSnapshot;
  }

  /// Enter Simulation Mode (capturing the edit state) and start advancing.
  void play();

  /// Stop advancing but stay in Simulation Mode.
  void pause();
  void togglePause();

  /// Advance exactly `count` steps (entering Simulation Mode if needed).
  void step(std::size_t count = 1);

  /// Return to the captured edit state and Edit mode.
  void reset();

  /// Restore the captured edit state and stay in Simulation mode.
  ///
  /// Returns false when there is no captured edit state to restart from.
  [[nodiscard]] bool restart();

  /// Discard any captured edit state and return to a fresh Edit state.
  ///
  /// Use when the underlying scene is replaced wholesale (e.g. loading a
  /// project) so a later reset() targets the newly loaded scene instead of a
  /// stale pre-load snapshot.
  void clearForNewScene();

  /// Advance according to the real-time factor for `realSeconds` of wall time.
  /// Only does work while running; intended to be called once per GUI frame.
  void advance(double realSeconds);

  [[nodiscard]] double realTimeFactor() const
  {
    return m_realTimeFactor;
  }
  void setRealTimeFactor(double factor);

  [[nodiscard]] double simTime() const;
  [[nodiscard]] std::size_t frameCount() const;

  /// Invoked when Edit/Simulation mode or running state changes.
  std::function<void()> onModeChanged;

  /// Invoked after a restart restores the captured runtime reset target.
  std::function<void()> onRestarted;

  /// Invoked after each individual step (used to record replay frames).
  std::function<void()> onAfterStep;

private:
  void enterSimulationMode();
  void notifyModeChangedIfNeeded(Mode beforeMode, bool beforeRunning);
  void stepOnce();

  ObjectManager& m_objects;
  Mode m_mode = Mode::Edit;
  bool m_running = false;
  double m_realTimeFactor = 1.0;
  double m_accumulator = 0.0;
  SceneModel m_editSnapshot;
  bool m_haveSnapshot = false;
};

} // namespace dartsim
