/*
 * Copyright (c) 2011-2025, The DART development contributors
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

#include <dart/simulation/experimental/fwd.hpp>

#include <dart/simulation/experimental/body/rigid_body_options.hpp>

#include <Eigen/Geometry>
#include <entt/entt.hpp>

#include <iosfwd>
#include <optional>
#include <string>
#include <string_view>

#include <cstddef>

namespace dart::simulation::experimental {

class DART_EXPERIMENTAL_API World
{
public:
  World();
  ~World() = default;

  World(const World&) = delete;
  World& operator=(const World&) = delete;
  World(World&&) = delete;
  World& operator=(World&&) = delete;

  //--------------------------------------------------------------------------
  // Frame creation
  //--------------------------------------------------------------------------
  FreeFrame addFreeFrame();
  FreeFrame addFreeFrame(std::string_view name);
  FreeFrame addFreeFrame(std::string_view name, const Frame& parent);

  FixedFrame addFixedFrame(std::string_view name, const Frame& parent);
  FixedFrame addFixedFrame(
      std::string_view name,
      const Frame& parent,
      const Eigen::Isometry3d& offset);

  //--------------------------------------------------------------------------
  // Multibody management
  //--------------------------------------------------------------------------
  MultiBody addMultiBody(std::string_view name);
  std::optional<MultiBody> getMultiBody(std::string_view name);
  std::size_t getMultiBodyCount() const;

  //--------------------------------------------------------------------------
  // Rigid body management
  //--------------------------------------------------------------------------
  RigidBody addRigidBody(
      std::string_view name,
      const RigidBodyOptions& options = RigidBodyOptions{});
  bool hasRigidBody(std::string_view name) const;
  std::size_t getRigidBodyCount() const;

  //--------------------------------------------------------------------------
  // Simulation control
  //--------------------------------------------------------------------------
  [[nodiscard]] bool isSimulationMode() const
  {
    return m_simulationMode;
  }

  void enterSimulationMode();
  void updateKinematics();

  //--------------------------------------------------------------------------
  // Registry access
  //--------------------------------------------------------------------------
  entt::registry& getRegistry();
  const entt::registry& getRegistry() const;

  //--------------------------------------------------------------------------
  // Serialization
  //--------------------------------------------------------------------------
  void saveBinary(std::ostream& output) const;
  void loadBinary(std::istream& input);

  //--------------------------------------------------------------------------
  // Utilities
  //--------------------------------------------------------------------------
  void clear();

private:
  friend class Frame;
  friend class FreeFrame;
  friend class FixedFrame;
  friend class Link;
  friend class Joint;
  friend class MultiBody;
  friend class RigidBody;

  Frame resolveParentFrame(const Frame& parent) const;
  entt::entity createFrameEntity(
      std::string_view name,
      const Frame& parentFrame,
      const Eigen::Isometry3d& localTransform,
      std::size_t* autoNameCounter,
      std::string_view autoNamePrefix,
      bool isFixedFrame,
      std::string& outName);

  void ensureDesignMode() const;
  void resetCountersFromRegistry();

  entt::registry m_registry;
  bool m_simulationMode{false};

  std::size_t m_freeFrameCounter{0};
  std::size_t m_fixedFrameCounter{0};
  std::size_t m_multiBodyCounter{0};
  std::size_t m_rigidBodyCounter{0};
  std::size_t m_linkCounter{0};
  std::size_t m_jointCounter{0};
};

} // namespace dart::simulation::experimental
