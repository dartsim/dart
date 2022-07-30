/*
 * Copyright (c) 2011-2022, The DART development contributors:
 * https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "dart/application/simple_simulator.hpp"

#include <thread>

#include "dart/common/all.hpp"
#include "dart/gui/all.hpp"
#include "dart/simulation/all.hpp"

namespace dart::application {

//==============================================================================
struct SimpleSimulator::Implementation
{
  SimpleSimulatorConfigs configs;

  std::shared_ptr<simulation::World> world{nullptr};

  std::shared_ptr<gui::Scene> scene{nullptr};
  std::shared_ptr<gui::Camera> camera{nullptr};

  // std::shared_ptr<gui::MainWindow> main_window{nullptr};

  Implementation()
  {
    // Do nothing
  }
};

//==============================================================================
SimpleSimulator::SimpleSimulator(const SimpleSimulatorConfigs& configs)
  : Simulator(configs), m_impl(std::make_unique<Implementation>())
{
  // Store configs
  m_impl->configs = configs;

  // Create an empty world
  m_impl->world = simulation::World::Create();

  // Create scene
  m_impl->scene = gui::Scene::Create();
  m_impl->scene->set_world(m_impl->world);

  // Create camera
  m_impl->camera = m_impl->scene->create_camera();

  // Create main window
  if (m_impl->configs.headless) {
    // m_impl->main_window = nullptr;
  } else {
    // m_impl->main_window = gui::MainWindow::Create();
  }
}

//==============================================================================
SimpleSimulator::~SimpleSimulator()
{
  // Clear resources
  // m_impl->main_window.reset();
  m_impl->camera.reset();
  m_impl->scene.reset();
  m_impl->world.reset();
}

//==============================================================================
void SimpleSimulator::run(long num_steps)
{
  long steps = 0;
  while (true) {
    if (!m_impl->configs.headless) {
      //      DART_ASSERT(m_impl->main_window);
      //      if (m_impl->main_window->should_close()) {
      //        break;
      //      }
    }

    if (num_steps != 0 && steps >= num_steps) {
      break;
    }

    // update world (or simulation)

    // update rendering scene
    m_impl->camera->render();

    if (m_impl->configs.headless) {
      // m_impl->world->step();
    } else {
      //      DART_ASSERT(m_impl->main_window);
      // m_impl->main_window->update();
    }

    steps++;
  }
}

} // namespace dart::application
