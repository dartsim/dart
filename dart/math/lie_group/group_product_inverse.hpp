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

#include <dart/math/lie_group/inverse_base.hpp>

namespace Eigen::internal {

template <typename GroupProductDerived>
struct traits<::dart::math::GroupProductInverse<GroupProductDerived>>
  : traits<typename GroupProductDerived::LieGroup>
{
};

} // namespace Eigen::internal

namespace dart::math {

/// Lightweight inverse expression for GroupProduct.
///
/// The expression references the original product and materializes only when
/// evaluated, matching SO3Inverse and SE3Inverse.
template <typename GroupProductDerived>
class GroupProductInverse
  : public InverseBase<GroupProductInverse<GroupProductDerived>>
{
public:
  using Base = InverseBase<GroupProductInverse<GroupProductDerived>>;
  using LieGroup = typename Base::LieGroup;

  using Base::eval;

  explicit GroupProductInverse(const GroupProductDerived& groupProduct)
    : m_groupProduct(groupProduct)
  {
    // Do nothing
  }

  const GroupProductDerived& original() const
  {
    return m_groupProduct;
  }

protected:
  const GroupProductDerived& m_groupProduct;
};

} // namespace dart::math
