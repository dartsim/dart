/*
 * Copyright (c) 2011-2021, The DART development contributors
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

#pragma once

#include <Eigen/Core>

#include "dart/math/lie_group/detail/lie_group_base.hpp"
#include "dart/math/type.hpp"

namespace dart::math {

//==============================================================================
template <typename Derived>
class RBase : public LieGroupBase<Derived>
{
public:
  using This = RBase<Derived>;
  using Base = LieGroupBase<Derived>;

  DART_LIE_GROUP_USE_BASE_TYPES

  //  template <typename OtherDerived>
  //  bool operator==(const RBase<OtherDerived>& other) const
  //  {
  //    return vector() == other.vector();
  //  }

  //  bool is_zero() const
  //  {
  //    return derived().vector().isZero();
  //  }

  using Base::coeffs;
  using Base::data;

protected:
  using Base::derived;
};

namespace detail {

//==============================================================================
template <typename Derived>
struct NormalizationOperator<RBase<Derived>>
{
  template <typename T>
  static void run(T& x)
  {
    // TODO(JS)
    (void)x;
  }
};

//==============================================================================
template <typename Derived>
struct RandomSetter<RBase<Derived>>
{
  template <typename T>
  static void run(T& x)
  {
    using LieGroup = typename RBase<Derived>::LieGroup;
    using LieGroupData = typename RBase<Derived>::LieGroupData;

    if constexpr (RBase<Derived>::GroupDim > 0) {
      x = LieGroup(LieGroupData::Random());
    } else {
      if (x.coeffs().size() > 0) {
        x = LieGroup(LieGroupData::Random(x.coeffs().size()));
      }
    }
  }
};

} // namespace detail

} // namespace dart::math
