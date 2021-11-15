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

#include "dart/math/bounding_volume/aabb3.hpp"
#include "dart/math/geometry/geometry.hpp"

namespace dart::math {

template <typename Scalar_>
class Geometry3 : public Geometry
{
public:
  // Type aliases
  using Scalar = Scalar_;

  /// Computes the volume of this 3D geometry.
  virtual Scalar get_volume() const
  {
    return 0;
  }
  // TODO(JS): Make pure virtual

  /// @{ @name AABB

  const Aabb3<Scalar>& get_local_aabb() const;

  virtual bool is_local_aabb_rotation_invariant() const;

  /// @}

protected:
  virtual void update_local_aabb_impl() const {}
  // TODO(JS): Make pure virtual

  mutable Aabb3<Scalar> m_local_aabb;
  mutable bool m_local_aabb_dirty{true};
};

//==============================================================================
template <typename Scalar>
const Aabb3<Scalar>& Geometry3<Scalar>::get_local_aabb() const
{
  if (m_local_aabb_dirty) {
    update_local_aabb_impl();
    m_local_aabb_dirty = false;
  }
  return m_local_aabb;
}

//==============================================================================
template <typename Scalar>
bool Geometry3<Scalar>::is_local_aabb_rotation_invariant() const
{
  return false;
}

} // namespace dart::math
