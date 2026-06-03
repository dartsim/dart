/*
 * Copyright (c) 2011, The DART development contributors
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

#include <algorithm>
#include <concepts>
#include <format>
#include <iterator>
#include <stdexcept>
#include <type_traits>
#include <utility>

#include <cstddef>

namespace dart::simulation::experimental {

namespace {

template <typename Registry>
constexpr bool IsWorldRegistry
    = std::same_as<std::remove_cvref_t<Registry>, detail::WorldRegistry>;

const WorldRegistryComponentMapper* requireWorldRegistryMapper(
    const ComponentMapper& mapper, const std::string& variableName)
{
  const auto* worldMapper
      = dynamic_cast<const WorldRegistryComponentMapper*>(&mapper);
  if (worldMapper == nullptr) {
    throw std::invalid_argument(
        std::format(
            "Mapper for variable '{}' does not support WorldRegistry; "
            "custom mappers used with World-owned registries must also "
            "implement WorldRegistryComponentMapper",
            variableName));
  }

  return worldMapper;
}

WorldRegistryComponentMapper* requireWorldRegistryMapper(
    ComponentMapper& mapper, const std::string& variableName)
{
  auto* worldMapper = dynamic_cast<WorldRegistryComponentMapper*>(&mapper);
  if (worldMapper == nullptr) {
    throw std::invalid_argument(
        std::format(
            "Mapper for variable '{}' does not support WorldRegistry; "
            "custom mappers used with World-owned registries must also "
            "implement WorldRegistryComponentMapper",
            variableName));
  }

  return worldMapper;
}

} // namespace

VectorMapper::VectorMapper(StateSpace space) : m_space(std::move(space))
{
  // Reserve space for mappers based on number of variables
  m_mappers.reserve(m_space.getNumVariables());
}

VectorMapper::~VectorMapper() = default;

template <typename Registry>
std::vector<double> VectorMapper::toVectorImpl(const Registry& registry) const
{
  std::vector<double> result(m_space.getDimension());
  toVector(registry, result);
  return result;
}

template <typename Registry>
Eigen::VectorXd VectorMapper::toEigenImpl(const Registry& registry) const
{
  Eigen::VectorXd result(m_space.getDimension());
  toEigen(registry, result);
  return result;
}

template <typename Registry>
void VectorMapper::toVectorImpl(
    const Registry& registry, std::vector<double>& output) const
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
      size_t written = 0;
      if constexpr (IsWorldRegistry<Registry>) {
        const auto* worldMapper = requireWorldRegistryMapper(
            std::as_const(*m_mappers[i]), variables[i].name);
        written = worldMapper->toVector(registry, output, offset);
      } else {
        written = m_mappers[i]->toVector(registry, output, offset);
      }
      offset += written;
    } else {
      // No mapper for this variable, fill with zeros
      std::ranges::fill_n(
          std::next(output.begin(), static_cast<std::ptrdiff_t>(offset)),
          static_cast<std::ptrdiff_t>(variables[i].dimension),
          0.0);
      offset += variables[i].dimension;
    }
  }
}

template <typename Registry>
void VectorMapper::toEigenImpl(
    const Registry& registry, Eigen::VectorXd& output) const
{
  if (std::cmp_less(output.size(), m_space.getDimension())) {
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
  std::ranges::copy(temp, output.data());
}

template <typename Registry>
void VectorMapper::fromVectorImpl(
    Registry& registry, std::span<const double> vec)
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
      size_t read = 0;
      if constexpr (IsWorldRegistry<Registry>) {
        auto* worldMapper
            = requireWorldRegistryMapper(*m_mappers[i], variables[i].name);
        read = worldMapper->fromVector(registry, vec, offset);
      } else {
        read = m_mappers[i]->fromVector(registry, vec, offset);
      }
      offset += read;
    } else {
      // No mapper for this variable, skip
      offset += variables[i].dimension;
    }
  }
}

template <typename Registry>
void VectorMapper::fromEigenImpl(Registry& registry, const Eigen::VectorXd& vec)
{
  if (std::cmp_less(vec.size(), m_space.getDimension())) {
    throw std::invalid_argument(
        std::format(
            "Input vector size ({}) < required dimension ({})",
            vec.size(),
            m_space.getDimension()));
  }

  fromVector(
      registry,
      std::span<const double>(vec.data(), static_cast<size_t>(vec.size())));
}

std::vector<double> VectorMapper::toVector(const entt::registry& registry) const
{
  return toVectorImpl(registry);
}

std::vector<double> VectorMapper::toVector(
    const detail::WorldRegistry& registry) const
{
  return toVectorImpl(registry);
}

Eigen::VectorXd VectorMapper::toEigen(const entt::registry& registry) const
{
  return toEigenImpl(registry);
}

Eigen::VectorXd VectorMapper::toEigen(
    const detail::WorldRegistry& registry) const
{
  return toEigenImpl(registry);
}

void VectorMapper::toVector(
    const entt::registry& registry, std::vector<double>& output) const
{
  toVectorImpl(registry, output);
}

void VectorMapper::toVector(
    const detail::WorldRegistry& registry, std::vector<double>& output) const
{
  toVectorImpl(registry, output);
}

void VectorMapper::toEigen(
    const entt::registry& registry, Eigen::VectorXd& output) const
{
  toEigenImpl(registry, output);
}

void VectorMapper::toEigen(
    const detail::WorldRegistry& registry, Eigen::VectorXd& output) const
{
  toEigenImpl(registry, output);
}

void VectorMapper::fromVector(
    entt::registry& registry, std::span<const double> vec)
{
  fromVectorImpl(registry, vec);
}

void VectorMapper::fromVector(
    detail::WorldRegistry& registry, std::span<const double> vec)
{
  fromVectorImpl(registry, vec);
}

void VectorMapper::fromEigen(
    entt::registry& registry, const Eigen::VectorXd& vec)
{
  fromEigenImpl(registry, vec);
}

void VectorMapper::fromEigen(
    detail::WorldRegistry& registry, const Eigen::VectorXd& vec)
{
  fromEigenImpl(registry, vec);
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
