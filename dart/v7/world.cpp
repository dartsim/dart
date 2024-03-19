/*
 * Copyright (c) The DART development contributors
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

#include "dart/v7/world.hpp"

#include "dart/v7/ecs/system_graph.hpp"
#include "dart/v7/logging.hpp"

namespace dart::v7 {

struct World::Private
{
  std::unique_ptr<Context> context;
  SystemGraph graph;
  Entity entity;
};

World::World() : data(std::make_unique<World::Private>())
{
  data->context = std::make_unique<Context>();
  data->entity = data->context->create();
}

World::~World()
{
  // Empty
}

const Context& World::getContext() const
{
  return *data->context;
}

Context& World::getMutableContext()
{
  return *data->context;
}

bool World::save(WorldSnapshot& data) const
{
  DART_FATAL("Not implemented yet");
  (void)data;
  return true;
}

bool World::saveToFile(const std::filesystem::path& path) const
{
  DART_FATAL("Not implemented yet");
  (void)path;
  return true;
}

bool World::load(const WorldSnapshot& data)
{
  DART_FATAL("Not implemented yet");
  (void)data;
  return true;
}

bool World::loadFromFile(const std::filesystem::path& path)
{
  DART_FATAL("Not implemented yet");
  (void)path;
  return true;
}

void World::step()
{
  DART_FATAL("Not implemented yet");
  data->graph.run();
}

RigidBody World::addRigidBody(const RigidBodyConfig& config)
{
  return RigidBody::Create(*data->context, this, config);
}

bool World::hasRigidBody(const RigidBody& rigidBody) const
{
  Context& context = *data->context;

  if (context.getId() != rigidBody.getContext().getId()) {
    return false;
  }

  if (!context.valid(rigidBody.getEntity())) {
    return false;
  }

  if (!rigidBody.isValid()) {
    return false;
  }

  return true;
}

void World::removeRigidBody(const RigidBody& rigidBody)
{
  RigidBody::Remove(*data->context, rigidBody.getEntity());
}

} // namespace dart::v7
