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

/// Owns the Edit/Run mode and the stepping loop over the experimental World.
///
/// Entering Run captures the authored design state so Reset can restore it
/// (the experimental World's default synchronous stepping is the reference
/// behavior; async stepping is intentionally not used).
class SimulationController
{
public:
  enum class Mode
  {
    Edit,
    Run
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

  /// Enter Run mode (capturing the design state) and start advancing.
  void play();

  /// Stop advancing but stay in Run mode.
  void pause();
  void togglePause();

  /// Advance exactly `count` steps (entering Run mode if needed).
  void step(std::size_t count = 1);

  /// Return to the captured design state and Edit mode.
  void reset();

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

  /// Invoked after each individual step (used to record replay frames).
  std::function<void()> onAfterStep;

private:
  void enterRunMode();
  void stepOnce();

  ObjectManager& m_objects;
  Mode m_mode = Mode::Edit;
  bool m_running = false;
  double m_realTimeFactor = 1.0;
  double m_accumulator = 0.0;
  SceneModel m_designSnapshot;
  bool m_haveSnapshot = false;
};

} // namespace dartsim
