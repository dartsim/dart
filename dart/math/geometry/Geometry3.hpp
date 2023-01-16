/*
 * Copyright (c) 2011-2023, The DART development contributors
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

#include <dart/math/bv/Aabb3.hpp>
#include <dart/math/geometry/Geometry.hpp>

namespace dart::math {

template <typename S>
class Geometry3 : public Geometry
{
public:
  using Scalar = S;

  /// Default constructor.
  Geometry3() = default;

  /// Destructor.
  ~Geometry3() override = default;

  /// Computes the volume of this geometry.
  virtual Scalar getVolume() const = 0;

  [[nodiscard]] const Aabb3<S>& getLocalAabb() const;

protected:
  /// The local AABB of this geometry.
  mutable Aabb3<S> mLocalAabb;

  /// True if the local AABB is dirty.
  mutable bool mLocalAabbDirty{true};
};

DART_TEMPLATE_CLASS_HEADER(MATH, Geometry3);

} // namespace dart::math

//==============================================================================
// Implementation
//==============================================================================

namespace dart::math {

//==============================================================================
template <typename S>
const Aabb3<S>& Geometry3<S>::getLocalAabb() const
{
  return mLocalAabb;
}

} // namespace dart::math
