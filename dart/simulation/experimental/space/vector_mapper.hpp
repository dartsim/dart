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

#include <dart/simulation/experimental/export.hpp>
#include <dart/simulation/experimental/space/component_mapper.hpp>
#include <dart/simulation/experimental/space/state_space.hpp>

#include <Eigen/Core>
#include <entt/entt.hpp>

#include <memory>
#include <vector>

namespace dart::simulation::experimental {

/// Bidirectional converter between ECS state and flat vectors
///
/// VectorMapper uses a StateSpace definition and ComponentMappers to:
/// 1. Extract component data from ECS registry to flat vector
/// 2. Write flat vector data back to ECS registry
///
/// Key features:
/// - Pre-allocates buffers for efficiency
/// - Reusable across multiple conversions
/// - Supports both std::vector and Eigen::VectorXd
///
/// Example usage:
/// ```cpp
/// StateSpace space;
/// space.addVariable("joint_pos", 6, -M_PI, M_PI);
/// space.finalize();
///
/// VectorMapper mapper(space);
///
/// // Extract state
/// auto vec = mapper.toVector(registry);
///
/// // Modify state
/// vec[0] += 0.1;
///
/// // Write back
/// mapper.fromVector(registry, vec);
/// ```
class DART_EXPERIMENTAL_API VectorMapper
{
public:
  /// Constructor
  /// @param space StateSpace defining vector structure (copied internally)
  explicit VectorMapper(StateSpace space);

  /// Destructor
  ~VectorMapper();

  // Movable but not copyable (contains unique_ptr)
  VectorMapper(const VectorMapper&) = delete;
  VectorMapper& operator=(const VectorMapper&) = delete;
  VectorMapper(VectorMapper&&) noexcept = default;
  VectorMapper& operator=(VectorMapper&&) noexcept = default;

  /// Extract ECS state to vector
  /// @param registry ECS registry containing state
  /// @return Flat vector of doubles
  [[nodiscard]] std::vector<double> toVector(
      const entt::registry& registry) const;

  /// Extract ECS state to Eigen vector
  /// @param registry ECS registry containing state
  /// @return Eigen::VectorXd
  [[nodiscard]] Eigen::VectorXd toEigen(const entt::registry& registry) const;

  /// Extract ECS state into pre-allocated vector (in-place)
  /// Avoids allocation for repeated calls
  /// @param registry ECS registry containing state
  /// @param output Output vector (must be size >= getDimension())
  void toVector(
      const entt::registry& registry, std::vector<double>& output) const;

  /// Extract ECS state into pre-allocated Eigen vector (in-place)
  /// @param registry ECS registry containing state
  /// @param output Output vector (must be size >= getDimension())
  void toEigen(const entt::registry& registry, Eigen::VectorXd& output) const;

  /// Write vector to ECS state
  /// @param registry ECS registry to modify
  /// @param vec Input vector
  void fromVector(entt::registry& registry, const std::vector<double>& vec);

  /// Write Eigen vector to ECS state
  /// @param registry ECS registry to modify
  /// @param vec Input Eigen vector
  void fromEigen(entt::registry& registry, const Eigen::VectorXd& vec);

  /// Get total dimension
  [[nodiscard]] size_t getDimension() const
  {
    return m_space.getDimension();
  }

  /// Get underlying StateSpace
  [[nodiscard]] const StateSpace& getStateSpace() const
  {
    return m_space;
  }

  /// Add a component mapper
  /// Links a StateSpace variable to a ComponentMapper for extraction
  /// @param variableName Name of variable in StateSpace
  /// @param mapper ComponentMapper to use for this variable
  void addMapper(
      const std::string& variableName, std::unique_ptr<ComponentMapper> mapper);

private:
  StateSpace m_space;
  std::vector<std::unique_ptr<ComponentMapper>> m_mappers;
};

} // namespace dart::simulation::experimental
