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

#include "dart/v7/rigid_body.hpp"

#include "dart/v7/comps/all.hpp"
#include "dart/v7/ecs/context.hpp"
#include "dart/v7/world.hpp"

namespace dart::v7 {

RigidBody RigidBody::Create(
    Context& context, World* world, const RigidBodyConfig& config)
{
  Entity e = context.create();
  context.emplace<comps::RigidBodyTag>(e);

  auto& nameComp = context.emplace<comps::Name>(e);
  nameComp.data = config.name;

  return RigidBody(world, e);
}

void RigidBody::Remove(Context& context, Entity e)
{
  context.destroy(e);
}

RigidBody::RigidBody(World* world, Entity e) : WorldObject(world, e)
{
  // Empty
}

bool RigidBody::isValid() const
{
  const auto& context = mWorld->getContext();
  if (!context.all_of<comps::RigidBodyTag, comps::Name>(mEntity)) {
    return false;
  }
  return true;
}

const std::string& RigidBody::getName() const
{
  const auto& context = getContext();
  return context.get<comps::Name>(mEntity).data;
}

void RigidBody::setName(const std::string& name)
{
  auto& context = getMutableContext();
  context.get<comps::Name>(mEntity).data = name;
}

} // namespace dart::v7
