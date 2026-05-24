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

#include <dart/simulation/experimental/world.hpp>

#include <dartsim_engine/object_manager.hpp>
#include <dartsim_engine/simulation_controller.hpp>

#include <algorithm>

namespace dartsim {

SimulationController::SimulationController(ObjectManager& objects)
  : m_objects(objects)
{
}

void SimulationController::enterRunMode()
{
  if (m_mode == Mode::Edit) {
    m_designSnapshot = m_objects.model();
    m_haveSnapshot = true;
    m_mode = Mode::Run;
  }
}

void SimulationController::play()
{
  enterRunMode();
  m_running = true;
}

void SimulationController::pause()
{
  m_running = false;
}

void SimulationController::togglePause()
{
  if (m_running) {
    pause();
  } else {
    play();
  }
}

void SimulationController::stepOnce()
{
  m_objects.world().step();
  if (onAfterStep) {
    onAfterStep();
  }
}

void SimulationController::step(std::size_t count)
{
  enterRunMode();
  for (std::size_t i = 0; i < count; ++i) {
    stepOnce();
  }
}

void SimulationController::reset()
{
  if (m_haveSnapshot) {
    m_objects.setModel(m_designSnapshot);
  }
  m_mode = Mode::Edit;
  m_running = false;
  m_accumulator = 0.0;
}

void SimulationController::clearForNewScene()
{
  m_mode = Mode::Edit;
  m_running = false;
  m_accumulator = 0.0;
  m_haveSnapshot = false;
  m_designSnapshot = SceneModel{};
}

void SimulationController::advance(double realSeconds)
{
  if (!m_running || realSeconds <= 0.0) {
    return;
  }
  const double dt = m_objects.model().timeStep;
  if (dt <= 0.0) {
    return;
  }
  m_accumulator += realSeconds * m_realTimeFactor;
  // Cap work per call so a long stall cannot spiral into unbounded stepping.
  int budget = 1000;
  while (m_accumulator >= dt && budget-- > 0) {
    stepOnce();
    m_accumulator -= dt;
  }
}

void SimulationController::setRealTimeFactor(double factor)
{
  m_realTimeFactor = std::max(factor, 0.0);
}

double SimulationController::simTime() const
{
  return m_objects.world().getTime();
}

std::size_t SimulationController::frameCount() const
{
  return m_objects.world().getFrame();
}

} // namespace dartsim
