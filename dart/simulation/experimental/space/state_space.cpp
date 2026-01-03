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

#include "dart/simulation/experimental/space/state_space.hpp"

#include "dart/simulation/experimental/common/exceptions.hpp"

#include <format>
#include <stdexcept>

namespace dart::simulation::experimental {

StateSpace& StateSpace::addVariable(
    const std::string& name, size_t dimension, double lower, double upper)
{
  ensureMutable();

  if (dimension == 0) {
    throw std::invalid_argument(
        std::format("Variable '{}' dimension must be > 0", name));
  }

  if (m_nameToIndex.contains(name)) {
    throw std::invalid_argument(
        std::format("Variable '{}' already exists", name));
  }

  if (lower > upper) {
    throw std::invalid_argument(std::format(
        "Variable '{}': lower bound ({}) > upper bound ({})",
        name,
        lower,
        upper));
  }

  Variable var;
  var.name = name;
  var.dimension = dimension;
  var.lowerBound = lower;
  var.upperBound = upper;

  m_variables.push_back(std::move(var));
  m_nameToIndex[name] = m_variables.size() - 1;

  recomputeIndices();

  return *this;
}

StateSpace& StateSpace::addVariables(
    const std::vector<std::string>& names, double lower, double upper)
{
  for (const auto& name : names) {
    addVariable(name, 1, lower, upper);
  }
  return *this;
}

void StateSpace::finalize()
{
  if (m_finalized) {
    return; // Already finalized
  }

  // Recompute indices one final time
  recomputeIndices();

  m_finalized = true;
}

std::optional<StateSpace::Variable> StateSpace::getVariable(
    const std::string& name) const
{
  auto it = m_nameToIndex.find(name);
  if (it == m_nameToIndex.end()) {
    return std::nullopt;
  }
  return m_variables[it->second];
}

std::optional<size_t> StateSpace::getVariableIndex(
    const std::string& name) const
{
  auto it = m_nameToIndex.find(name);
  if (it == m_nameToIndex.end()) {
    return std::nullopt;
  }
  return it->second;
}

std::vector<std::string> StateSpace::getVariableNames() const
{
  std::vector<std::string> names;
  names.reserve(m_variables.size());
  for (const auto& var : m_variables) {
    names.push_back(var.name);
  }
  return names;
}

std::vector<double> StateSpace::getLowerBounds() const
{
  std::vector<double> bounds;
  bounds.reserve(m_totalDimension);

  for (const auto& var : m_variables) {
    for (size_t i = 0; i < var.dimension; ++i) {
      bounds.push_back(var.lowerBound);
    }
  }

  return bounds;
}

std::vector<double> StateSpace::getUpperBounds() const
{
  std::vector<double> bounds;
  bounds.reserve(m_totalDimension);

  for (const auto& var : m_variables) {
    for (size_t i = 0; i < var.dimension; ++i) {
      bounds.push_back(var.upperBound);
    }
  }

  return bounds;
}

bool StateSpace::hasVariable(const std::string& name) const
{
  return m_nameToIndex.contains(name);
}

void StateSpace::recomputeIndices()
{
  size_t offset = 0;
  for (auto& var : m_variables) {
    var.startIndex = offset;
    offset += var.dimension;
  }
  m_totalDimension = offset;
}

void StateSpace::ensureMutable() const
{
  if (m_finalized) {
    throw std::logic_error("StateSpace is finalized and cannot be modified");
  }
}

} // namespace dart::simulation::experimental
