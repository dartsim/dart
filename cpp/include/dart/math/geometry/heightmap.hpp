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

#include <string>
#include <vector>

#include "dart/common/eigen_include.hpp"
#include "dart/math/geometry/convex3.hpp"

namespace dart {
namespace math {

template <typename S_>
class Heightmap : public Convex3<S_>
{
public:
  // Type aliases
  using S = S_;
  using Vector3 = typename Convex3<S>::Vector3;
  using HeightField
      = Eigen::Matrix<S, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

  /// Returns type string
  static const std::string& GetType();

  /// Constructor
  Heightmap();

  // Documentation inherited
  const std::string& get_type() const override;

  /// Sets scale of this heightmap.
  /// \param[in] scale Scale of the height map.
  void set_scale(const Vector3& scale);

  /// Returns scale of this heightmap.
  const Vector3& get_scale() const;

  /// Sets the height field.
  ///
  /// The data in \e heights will be copied locally. It would be nice to have
  /// the option to use the values in \e heights directly instead of copying
  /// them locally to a vector in this class (this would avoid any data being
  /// kept twice). However some collision engine implementations may require to
  /// transform the height values, e.g. bullet needs the y values flipped.
  /// Therefore, a (mutable) copy of the height values passed in \e heights will
  /// be kept in this class. The copied data can be modified via
  /// getHeightFieldModifiable() and with flipY().
  ///
  /// \param[in] width Width of the field (x axis)
  /// \param[in] depth Depth of the field (-y axis)
  /// \param[in] heights The height data of size \e width * \e depth. The
  /// heights are interpreted as z values, while \e width goes in x direction
  /// and \e depth in -y (it goes to -y because traditionally images are read
  /// from top row to bottom row). In the geometry which is to be generated from
  /// this shape, the min/max height value is also the min/max z value (so if
  /// the minimum height value is -100, the lowest terrain point will be -100,
  /// times the z scale to be applied).
  void set_height_field(
      const std::size_t& width,
      const std::size_t& depth,
      const std::vector<S>& heights);

  /// Sets the height field.
  ///
  /// The data in \e heights will be copied locally. It would be nice to have
  /// the option to use the values in \e heights directly instead of copying
  /// them locally to a vector in this class (this would avoid any data being
  /// kept twice). However some collision engine implementations may require to
  /// transform the height values, e.g. bullet needs the y values flipped.
  /// Therefore, a (mutable) copy of the height values passed in \e heights will
  /// be kept in this class. The copied data can be modified via
  /// getHeightFieldModifiable() and with flipY().
  ///
  /// \param[in] heights The height data of size \e width * \e depth where
  /// number of columns and number of rows are the width of the field (x axis)
  /// and the depth of the field (-y axis), respectively. The heights are
  /// interpreted as z values, while \e width goes in x direction and \e depth
  /// in -y (it goes to -y because traditionally images are read from top row to
  /// bottom row). In the geometry which is to be generated from this shape, the
  /// min/max height value is also the min/max z value (so if the minimum height
  /// value is -100, the lowest terrain point will be -100, times the z scale to
  /// be applied).
  void set_height_field(const HeightField& heights);

  /// Returns the height field.
  const HeightField& get_height_field() const;

  /// Returns the modified height field. See also setHeightField().
  HeightField& get_height_field_modifiable() const;

  /// Flips the y values in the height field.
  void flip_y() const;

  /// Returns the width dimension of the height field
  std::size_t get_width() const;

  /// Returns the height dimension of the height field
  std::size_t get_depth() const;

  /// Returns the minimum height set by setHeightField()
  S get_min_height() const;

  /// Returns the maximum height set by setHeightField()
  S get_max_height() const;

private:
  /// Scale of the heightmap
  Vector3 m_scale;

  /// Height field
  mutable HeightField m_heights;

  /// Minimum heights.
  /// Is computed each time the height field is set with setHeightField().
  S m_min_height;

  /// Maximum heights.
  /// Is computed each time the height field is set with setHeightField().
  S m_max_height;
};

using Heightmapf = Heightmap<float>;
using Heightmapd = Heightmap<double>;

extern template class DART_MATH_API Heightmap<double>;

} // namespace math
} // namespace dart

#include "dart/math/geometry/detail/heightmap_impl.hpp"
