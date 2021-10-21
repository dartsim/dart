/*
 * Copyright (c) 2011-2021, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/master/LICENSE
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

#include "dart/common/logging.hpp"
#include "dart/common/macro.hpp"
#include "dart/math/geometry/heightmap.hpp"

namespace dart {
namespace math {

//==============================================================================
template <typename S>
const std::string& Heightmap<S>::GetType()
{
  static const std::string type("Heightmap");
  return type;
}

//==============================================================================
template <typename S>
Heightmap<S>::Heightmap() : m_scale(1, 1, 1)
{
  static_assert(
      std::is_same<S, float>::value || std::is_same<S, double>::value,
      "Height field needs to be double or float");
}

//==============================================================================
template <typename S>
const std::string& Heightmap<S>::get_type() const
{
  return GetType();
}

//==============================================================================
template <typename S>
void Heightmap<S>::set_scale(const Vector3& scale)
{
  DART_ASSERT(scale[0] > 0.0);
  DART_ASSERT(scale[1] > 0.0);
  DART_ASSERT(scale[2] > 0.0);
  m_scale = scale;
}

//==============================================================================
template <typename S>
auto Heightmap<S>::get_scale() const -> const Vector3&
{
  return m_scale;
}

//==============================================================================
template <typename S>
void Heightmap<S>::set_height_field(
    const std::size_t& width,
    const std::size_t& depth,
    const std::vector<S>& heights)
{
  DART_ASSERT(heights.size() == width * depth);

  if ((width * depth) != heights.size()) {
    DART_ERROR(
        "Size of height field [{}] needs to be width({})*depth({}) = [{}]",
        heights.size(),
        width,
        depth,
        width * depth);
    return;
  }

  if (heights.empty()) {
    DART_WARN("Empty height field makes no sense.");
    return;
  }

  // make heightmap data local copy
  const Eigen::Map<const HeightField> data(heights.data(), depth, width);

  set_height_field(data);
}

//==============================================================================
template <typename S>
void Heightmap<S>::set_height_field(const HeightField& heights)
{
  m_heights = heights;

  m_min_height = heights.minCoeff();
  m_max_height = heights.maxCoeff();
}

//==============================================================================
template <typename S>
auto Heightmap<S>::get_height_field() const -> const HeightField&
{
  return m_heights;
}

//==============================================================================
template <typename S>
auto Heightmap<S>::get_height_field_modifiable() const -> HeightField&
{
  return m_heights;
}

//==============================================================================
template <typename S>
void Heightmap<S>::flip_y() const
{
  m_heights = m_heights.colwise().reverse().eval();
}

//==============================================================================
template <typename S>
S Heightmap<S>::get_max_height() const
{
  return m_max_height;
}

//==============================================================================
template <typename S>
S Heightmap<S>::get_min_height() const
{
  return m_min_height;
}

//==============================================================================
template <typename S>
std::size_t Heightmap<S>::get_width() const
{
  return m_heights.cols();
}

//==============================================================================
template <typename S>
std::size_t Heightmap<S>::get_depth() const
{
  return m_heights.rows();
}

} // namespace math
} // namespace dart
