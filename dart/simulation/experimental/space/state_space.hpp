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

#include <functional>
#include <limits>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace dart::simulation::experimental {

/// StateSpace defines the structure and metadata of a flat vector
/// representation of World state. It describes which components/fields should
/// be extracted, their dimensions, bounds, and ordering in the vector.
///
/// Key Design Principles:
/// - Configurable view: Multiple StateSpaces can exist for the same World
/// - Immutable after finalization: Ensures stable indexing for optimization
/// - Metadata-rich: Provides names, bounds, dimensions for algorithms
///
/// Example Usage:
/// ```cpp
/// StateSpace space;
/// space.addVariable("joint_pos", 6, -M_PI, M_PI);
/// space.addVariable("joint_vel", 6, -10.0, 10.0);
/// space.finalize();
///
/// size_t dim = space.getDimension();  // 12
/// auto bounds = space.getLowerBounds();
/// ```
class DART8_API StateSpace
{
public:
  /// Represents a single variable (or group of variables) in the state vector
  struct Variable
  {
    std::string name;     // Human-readable identifier
    size_t startIndex{0}; // Starting index in flat vector
    size_t dimension{0};  // Number of scalars (1 for scalar, N for vector)
    double lowerBound{-std::numeric_limits<double>::infinity()};
    double upperBound{std::numeric_limits<double>::infinity()};
  };

  StateSpace() = default;
  ~StateSpace() = default;

  // Movable and copyable
  StateSpace(const StateSpace&) = default;
  StateSpace& operator=(const StateSpace&) = default;
  StateSpace(StateSpace&&) noexcept = default;
  StateSpace& operator=(StateSpace&&) noexcept = default;

  /// Add a variable to the state space
  /// @param name Variable name (must be unique)
  /// @param dimension Number of scalars
  /// @param lower Lower bound for all components
  /// @param upper Upper bound for all components
  /// @return Reference to this for method chaining
  StateSpace& addVariable(
      const std::string& name,
      size_t dimension,
      double lower = -std::numeric_limits<double>::infinity(),
      double upper = std::numeric_limits<double>::infinity());

  /// Add multiple scalar variables with same bounds
  /// Convenience method for adding multiple variables at once
  /// @param names Vector of variable names
  /// @param lower Lower bound for all variables
  /// @param upper Upper bound for all variables
  /// @return Reference to this for method chaining
  StateSpace& addVariables(
      const std::vector<std::string>& names,
      double lower = -std::numeric_limits<double>::infinity(),
      double upper = std::numeric_limits<double>::infinity());

  /// Finalize the state space (makes it immutable)
  /// After finalization, no more variables can be added
  /// This ensures stable indexing for optimization algorithms
  void finalize();

  /// Check if state space is finalized
  [[nodiscard]] bool isFinalized() const
  {
    return m_finalized;
  }

  /// Get total dimension (sum of all variable dimensions)
  [[nodiscard]] size_t getDimension() const
  {
    return m_totalDimension;
  }

  /// Get number of variables
  [[nodiscard]] size_t getNumVariables() const
  {
    return m_variables.size();
  }

  /// Get all variables
  [[nodiscard]] const std::vector<Variable>& getVariables() const
  {
    return m_variables;
  }

  /// Get variable by name
  [[nodiscard]] std::optional<Variable> getVariable(
      const std::string& name) const;

  /// Get variable index by name
  [[nodiscard]] std::optional<size_t> getVariableIndex(
      const std::string& name) const;

  /// Get variable names in order
  [[nodiscard]] std::vector<std::string> getVariableNames() const;

  /// Get lower bounds as flat vector
  [[nodiscard]] std::vector<double> getLowerBounds() const;

  /// Get upper bounds as flat vector
  [[nodiscard]] std::vector<double> getUpperBounds() const;

  /// Check if variable exists
  [[nodiscard]] bool hasVariable(const std::string& name) const;

private:
  std::vector<Variable> m_variables;
  std::unordered_map<std::string, size_t> m_nameToIndex;
  size_t m_totalDimension{0};
  bool m_finalized{false};

  /// Recompute indices after adding variables
  void recomputeIndices();

  /// Check if space is mutable (throws if finalized)
  void ensureMutable() const;
};

} // namespace dart::simulation::experimental
