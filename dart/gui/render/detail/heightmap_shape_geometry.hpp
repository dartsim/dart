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

#ifndef DART_GUI_RENDER_DETAIL_HEIGHTMAPSHAPEGEOMETRY_HPP_
#define DART_GUI_RENDER_DETAIL_HEIGHTMAPSHAPEGEOMETRY_HPP_

#include <dart/dynamics/heightmap_shape.hpp>

#include <Eigen/Core>

namespace dart {
namespace gui {
namespace render {
namespace detail {

template <typename S>
struct HeightmapVertexOrigin
{
  S xOffset;
  S yOffset;
};

template <typename S>
inline HeightmapVertexOrigin<S> computeHeightmapVertexOrigin(
    const typename dynamics::HeightmapShape<S>::HeightField& heightmap,
    const Eigen::Matrix<S, 3, 1>& scale)
{
  const Eigen::Index rows = heightmap.rows();
  const Eigen::Index cols = heightmap.cols();
  const S spanX = (cols > 1 ? static_cast<S>(cols - 1) : S(0)) * scale.x();
  const S spanY = (rows > 1 ? static_cast<S>(rows - 1) : S(0)) * scale.y();

  HeightmapVertexOrigin<S> origin;
  origin.xOffset = static_cast<S>(-0.5) * spanX;
  origin.yOffset = static_cast<S>(0.5) * spanY;
  return origin;
}

template <typename S>
inline Eigen::Matrix<S, 3, 1> computeHeightmapVertexPosition(
    const typename dynamics::HeightmapShape<S>::HeightField& heightmap,
    const Eigen::Matrix<S, 3, 1>& scale,
    const HeightmapVertexOrigin<S>& origin,
    const Eigen::Index row,
    const Eigen::Index col)
{
  return Eigen::Matrix<S, 3, 1>(
      static_cast<S>(col) * scale.x() + origin.xOffset,
      -static_cast<S>(row) * scale.y() + origin.yOffset,
      heightmap(row, col) * scale.z());
}

} // namespace detail
} // namespace render
} // namespace gui
} // namespace dart

#endif // DART_GUI_RENDER_DETAIL_HEIGHTMAPSHAPEGEOMETRY_HPP_
