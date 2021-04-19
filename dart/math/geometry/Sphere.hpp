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

#include <Eigen/Core>

#include "dart/math/geometry/Convex3.hpp"
#include "dart/math/geometry/Geometry3.hpp"

namespace dart {
namespace math {

template <typename S_>
class Sphere : public Convex3<S_>
{
public:
  // Type aliases
  using S = S_;
  using Vector3 = typename Convex3<S>::Vector3;

  static const std::string& GetType();

  static S ComputeVolume(S radius);

  static Eigen::Matrix<S, 3, 3> computeInertiaFromMass(S radius, S mass);

  static Eigen::Matrix<S, 3, 3> computeInertiaFromDensity(S radius, S density);

  /// Constructor
  ///
  /// \param[in] radius: The radius of this sphere to set.
  explicit Sphere(S radius);

  // Documentation inherited
  const std::string& getType() const override;

  /// Returns the radius
  S getRadius() const;

  /// Sets the radius
  void setRadius(S radius);

  // Documentation inherited
  Vector3 getLocalSupportPoint(const Vector3& direction) const override;

private:
  S mRadius;
};

using Spheref = Sphere<float>;
using Sphered = Sphere<double>;

extern template class Sphere<double>;

} // namespace math
} // namespace dart

#include "dart/math/geometry/detail/Sphere-impl.hpp"
