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

#include <dart/math/lie_group/InverseBase.hpp>

namespace Eigen::internal {

// TODO(JS): Move to a dedicated header file
template <typename SO3Derived>
struct traits<::dart::math::SO3Inverse<SO3Derived>>
  : traits<::dart::math::SO3<typename SO3Derived::Scalar>>
{
};

} // namespace Eigen::internal

namespace dart::math {

/// @brief SO3Inverse is a specialization of LieGroupBase for SO3Inverse
/// @tparam SO3Derived The derived type of SO3. It can be either SO3 or Map<SO3>
template <typename SO3Derived>
class SO3Inverse : public InverseBase<SO3Inverse<SO3Derived>>
{
public:
  using Base = InverseBase<SO3Inverse<SO3Derived>>;
  using LieGroup = typename Base::LieGroup;

  using Base::eval;

  explicit SO3Inverse(const SO3Derived& so3) : m_so3(so3)
  {
    // Do nothing
  }

  const SO3Derived& original() const
  {
    return m_so3;
  }

  SO3Derived& original()
  {
    return m_so3;
  }

protected:
  const SO3Derived& m_so3;
};

} // namespace dart::math

//==============================================================================
// Implementation
//==============================================================================

namespace dart::math {

//==============================================================================

} // namespace dart::math
