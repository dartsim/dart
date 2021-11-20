/*
 * Copyright (c) 2011-2021, The DART development contributors:
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

#include "dart/simulation/system/collision_detection_system.hpp"

#include "dart/collision/dart/dart_engine.hpp"
#include "dart/collision/engine.hpp"
#include "dart/collision/scene.hpp"

namespace dart::simulation {

//==============================================================================
struct CollisionDetectionSystem::Implementation
{
  collision::EnginePtr<double> collision_engine;
  collision::Scene<double>* collision_scene;

  Implementation()
  {
    // Do nothing
  }
};

//==============================================================================
CollisionDetectionSystem::CollisionDetectionSystem()
  : m_impl(std::make_unique<Implementation>())
{
  m_impl->collision_engine = collision::DartEngine<double>::Create();
  m_impl->collision_scene = m_impl->collision_engine->create_scene();
}

//==============================================================================
CollisionDetectionSystem::~CollisionDetectionSystem()
{
  m_impl->collision_engine->destroy_scene(m_impl->collision_scene);
}

//==============================================================================
void CollisionDetectionSystem::update(double time_step)
{
  m_impl->collision_scene->update(time_step);
}

} // namespace dart::simulation
