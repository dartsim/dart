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

#include "dart/gui/engine.hpp"

#include "dart/common/logging.hpp"
#include "dart/common/macro.hpp"
#include "dart/gui/scene.hpp"

namespace dart::gui {

//==============================================================================
struct Engine::Implementation
{
  Implementation()
  {
    // Do nothing
  }
};

//==============================================================================
Engine& Engine::Get()
{
  static Engine engine;
  return engine;
}

//==============================================================================
Engine::Engine() : m_impl(std::make_unique<Implementation>())
{
  // Do nothing
}

//==============================================================================
ScenePtr Engine::create_scene(const std::string& name)
{
  if (auto scene = get_scene_by_name(name)) {
    DART_DEBUG("A scene was already created with the name {}", name);
    return nullptr;
  }

  auto scene = Scene::Create();
  scene->set_name(name);

  if (!scene->init()) {
    DART_DEBUG("A scene cannot be created because it failed to initialize");
    return nullptr;
  }

  m_scenes[name] = scene;

  return scene;
}

//==============================================================================
ScenePtr Engine::get_scene_by_name(const std::string& name)
{
  const auto found = m_scenes.find(name);
  return (found != m_scenes.end()) ? found->second : nullptr;
}

//==============================================================================
Engine::~Engine()
{
  // Do nothing
}

} // namespace dart::gui
