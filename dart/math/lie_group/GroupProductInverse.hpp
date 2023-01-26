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
template <typename GrouProductDerived>
struct traits<::dart::math::GroupProductInverse<GrouProductDerived>>
  : traits<
        ::dart::math::GroupProductInverse<typename GrouProductDerived::Scalar>>
{
};

} // namespace Eigen::internal

namespace dart::math {

/// @brief GrouProductInverse is a specialization of LieGroupBase for
/// GrouProductInverse
/// @tparam S The scalar type
template <typename GrouProductDerived>
class GrouProductInverse
  : public InverseBase<GrouProductInverse<GrouProductDerived>>
{
public:
  using Base = InverseBase<GrouProductInverse<GrouProductDerived>>;
  using LieGroup = typename Base::LieGroup;

  using Base::eval;

  explicit GrouProductInverse(const GrouProductDerived& x) : m_x(x)
  {
    // Do nothing
  }

  const GrouProductDerived& original() const
  {
    return m_x;
  }

  GrouProductDerived& original()
  {
    return m_x;
  }

protected:
  const GrouProductDerived& m_x;
};

} // namespace dart::math

//==============================================================================
// Implementation
//==============================================================================

namespace dart::math {

//==============================================================================

} // namespace dart::math
