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

#pragma once

#include <dart/config.hpp>

#include <dart/v7/fwd.hpp>

#include <dart/v7/ecs/context.hpp>
#include <dart/v7/rigid_body.hpp>

#include <filesystem>
#include <memory>

namespace dart::v7 {

struct SceneDesc
{
  //
};

struct WorldConfig
{
  double stepTime{0.001};
};

struct WorldSnapshot
{
  bool valid{false};
  Context context;
};

struct StepInfo
{
  double timeS;
};

class World
{
public:
  World();
  ~World();

  const Context& getContext() const;
  Context& getMutableContext();

  bool save(WorldSnapshot& data) const;
  bool saveToFile(const std::filesystem::path& path) const;

  bool load(const WorldSnapshot& data);
  bool loadFromFile(const std::filesystem::path& path);

  void step();

  RigidBody addRigidBody(const RigidBodyConfig& config = RigidBodyConfig());
  bool hasRigidBody(const RigidBody& rigidBody) const;
  void removeRigidBody(const RigidBody& rigidBody);

private:
  struct Private;
  std::unique_ptr<Private> data;
};

} // namespace dart::v7
