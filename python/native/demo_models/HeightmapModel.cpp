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

#include "HeightmapModel.hpp"

#include <dart/math/Random.hpp>

#include <algorithm>
#include <limits>
#include <stdexcept>

#include <cmath>

namespace dart_demos {

std::shared_ptr<dart::dynamics::HeightmapShapef> createHeightmapShape(
    std::size_t xResolution,
    std::size_t yResolution,
    float xSize,
    float ySize,
    float zMin,
    float zMax)
{
  const auto maxElements = std::min(
      static_cast<std::size_t>(std::numeric_limits<Eigen::Index>::max()),
      std::numeric_limits<std::size_t>::max() / sizeof(float));
  if (xResolution == 0 || yResolution == 0
      || xResolution > maxElements / yResolution)
    throw std::invalid_argument(
        "Heightmap resolution must be positive and fit Eigen storage");
  if (!std::isfinite(xSize) || !std::isfinite(ySize) || xSize <= 0.f
      || ySize <= 0.f)
    throw std::invalid_argument("Heightmap size must be finite and positive");
  if (!std::isfinite(zMin) || !std::isfinite(zMax) || zMin > zMax
      || !std::isfinite(zMax - zMin))
    throw std::invalid_argument("Heightmap heights must be finite and ordered");

  const Eigen::Vector3f scale(
      xSize / static_cast<float>(xResolution),
      ySize / static_cast<float>(yResolution),
      1.f);
  if (!scale.allFinite() || (scale.array() <= 0.f).any())
    throw std::invalid_argument("Heightmap scale must be finite and positive");

  dart::dynamics::HeightmapShapef::HeightField data(yResolution, xResolution);
  for (std::size_t i = 0; i < yResolution; ++i) {
    for (std::size_t j = 0; j < xResolution; ++j)
      data(i, j) = dart::math::Random::uniform(zMin, zMax);
  }

  auto terrainShape = std::make_shared<dart::dynamics::HeightmapShapef>();
  terrainShape->setScale(scale);
  terrainShape->setHeightField(data);
  return terrainShape;
}

} // namespace dart_demos
