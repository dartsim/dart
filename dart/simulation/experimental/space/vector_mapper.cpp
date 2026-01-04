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

#include "dart/simulation/experimental/space/vector_mapper.hpp"

#include "dart/simulation/experimental/common/exceptions.hpp"

#include <format>
#include <stdexcept>

namespace dart::simulation::experimental {

VectorMapper::VectorMapper(StateSpace space) : m_space(std::move(space))
{
  // Reserve space for mappers based on number of variables
  m_mappers.reserve(m_space.getNumVariables());
}

VectorMapper::~VectorMapper() = default;

std::vector<double> VectorMapper::toVector(const entt::registry& registry) const
{
  std::vector<double> result(m_space.getDimension());
  toVector(registry, result);
  return result;
}

Eigen::VectorXd VectorMapper::toEigen(const entt::registry& registry) const
{
  Eigen::VectorXd result(m_space.getDimension());
  toEigen(registry, result);
  return result;
}

void VectorMapper::toVector(
    const entt::registry& registry, std::vector<double>& output) const
{
  if (output.size() < m_space.getDimension()) {
    throw std::invalid_argument(
        std::format(
            "Output vector size ({}) < required dimension ({})",
            output.size(),
            m_space.getDimension()));
  }

  size_t offset = 0;
  const auto& variables = m_space.getVariables();

  for (size_t i = 0; i < m_mappers.size(); ++i) {
    if (m_mappers[i]) {
      size_t written = m_mappers[i]->toVector(registry, output, offset);
      offset += written;
    } else {
      // No mapper for this variable, fill with zeros
      for (size_t j = 0; j < variables[i].dimension; ++j) {
        output[offset++] = 0.0;
      }
    }
  }
}

void VectorMapper::toEigen(
    const entt::registry& registry, Eigen::VectorXd& output) const
{
  if (static_cast<size_t>(output.size()) < m_space.getDimension()) {
    throw std::invalid_argument(
        std::format(
            "Output vector size ({}) < required dimension ({})",
            output.size(),
            m_space.getDimension()));
  }

  // Use temporary std::vector for conversion
  std::vector<double> temp(m_space.getDimension());
  toVector(registry, temp);

  // Copy to Eigen vector
  for (size_t i = 0; i < temp.size(); ++i) {
    output[i] = temp[i];
  }
}

void VectorMapper::fromVector(
    entt::registry& registry, const std::vector<double>& vec)
{
  if (vec.size() < m_space.getDimension()) {
    throw std::invalid_argument(
        std::format(
            "Input vector size ({}) < required dimension ({})",
            vec.size(),
            m_space.getDimension()));
  }

  size_t offset = 0;
  const auto& variables = m_space.getVariables();

  for (size_t i = 0; i < m_mappers.size(); ++i) {
    if (m_mappers[i]) {
      size_t read = m_mappers[i]->fromVector(registry, vec, offset);
      offset += read;
    } else {
      // No mapper for this variable, skip
      offset += variables[i].dimension;
    }
  }
}

void VectorMapper::fromEigen(
    entt::registry& registry, const Eigen::VectorXd& vec)
{
  if (static_cast<size_t>(vec.size()) < m_space.getDimension()) {
    throw std::invalid_argument(
        std::format(
            "Input vector size ({}) < required dimension ({})",
            vec.size(),
            m_space.getDimension()));
  }

  // Convert Eigen to std::vector
  std::vector<double> temp(vec.size());
  for (Eigen::Index i = 0; i < vec.size(); ++i) {
    temp[i] = vec[i];
  }

  fromVector(registry, temp);
}

void VectorMapper::addMapper(
    const std::string& variableName, std::unique_ptr<ComponentMapper> mapper)
{
  auto varIdx = m_space.getVariableIndex(variableName);
  if (!varIdx) {
    throw std::invalid_argument(
        std::format("Variable '{}' not found in StateSpace", variableName));
  }

  // Ensure mappers vector is large enough
  if (m_mappers.size() <= *varIdx) {
    m_mappers.resize(*varIdx + 1);
  }

  m_mappers[*varIdx] = std::move(mapper);
}

} // namespace dart::simulation::experimental
