/*
 * Copyright (c) 2011-2022, The DART development contributors:
 * https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include "dart/common/macro.hpp"
#include "dart/math/export.hpp"
#include "dart/math/type.hpp"

namespace dart::math {

template <typename Scalar_>
struct Line3
{
public:
  using Scalar = Scalar_;

  DART_SCALAR_SHOULD_BE_FLOATING_POINT(Line3, Scalar);

  /// The origin of the ray.
  Vector3<Scalar> from;

  /// The direction of the ray.
  Vector3<Scalar> to;

  /// Constructs an empty ray that points from (0, 0, 0) to (1, 0, 0).
  Line3();

  /// Constructs a ray with given origin and riection.
  Line3(const Vector3<Scalar>& from, const Vector3<Scalar>& to);

  /// Copy constructor.
  Line3(const Line3& other);

  Vector3<Scalar> get_direction(bool normalized = true) const;

  /// Returns a point on the ray at distance \p t.
  Vector3<Scalar> get_point_at(Scalar t) const;
};

using Line3d = Line3<double>;
using Line3f = Line3<float>;

} // namespace dart::math

#include "dart/math/geometry/detail/line3_impl.hpp"
