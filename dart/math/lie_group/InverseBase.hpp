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

#include <dart/math/Fwd.hpp>

namespace dart::math {

template <typename InverseDerived>
class InverseBase
//  : public LieGroupBase<
//        typename ::Eigen::internal::traits<InverseDerived>::LieGroup>
{
public:
  using Scalar = typename ::Eigen::internal::traits<InverseDerived>::Scalar;

  using LieGroup = typename ::Eigen::internal::traits<InverseDerived>::LieGroup;
  using Params = typename ::Eigen::internal::traits<InverseDerived>::Params;

  template <typename LieGroupDerived>
  [[nodiscard]] LieGroup operator*(
      const LieGroupBase<LieGroupDerived>& other) const;

  /// Returns the inverse of the Lie group.
  [[nodiscard]] LieGroup eval() const;

  operator LieGroup() const
  {
    return eval();
  }

  /// Returns the original Lie group, which is the inverse of the inverse.
  [[nodiscard]] const LieGroup& inverse() const;

  [[nodiscard]] const Params& params() const
  {
    return eval().params();
  }

  [[nodiscard]] Params& params()
  {
    return eval().params();
  }

  [[nodiscard]] const InverseDerived& derived() const noexcept;

  [[nodiscard]] InverseDerived& derived() noexcept;
};

// template <typename Derived, typename LieGroupType>
// typename Derived::LieGroup operator*(
//     const LieGroupBase<Derived>& lhs, const Inverse<LieGroupType>& rhs)
//{
//   if constexpr (std::is_same_v<Derived, LieGroupType>) {
//     if (&(lhs.derived()) == &(rhs.original())) {
//       return typename LieGroupType::LieGroup();
//     }
//   }

//  return lhs.derived() * rhs.eval();
//}

template <typename DerivedA, typename DerivedB>
typename DerivedA::LieGroup operator*(
    const LieGroupBase<DerivedA>& lhs, const InverseBase<DerivedB>& rhs)
{
  return lhs.derived() * rhs.eval();
}

} // namespace dart::math

//==============================================================================
// Implementation
//==============================================================================

namespace dart::math {

//==============================================================================
template <typename InverseDerived>
template <typename LieGroupDerived>
typename InverseBase<InverseDerived>::LieGroup
InverseBase<InverseDerived>::operator*(
    const LieGroupBase<LieGroupDerived>& other) const
{
  if constexpr (std::is_same_v<LieGroup, LieGroupDerived>) {
    if (&derived().original() == &(other.derived())) {
      return LieGroup();
    }
  }

  return eval() * other;
}

//==============================================================================
template <typename InverseDerived>
typename InverseBase<InverseDerived>::LieGroup
InverseBase<InverseDerived>::eval() const
{
  return LieGroup(derived().original()).inverseInPlace();
}

//==============================================================================
template <typename InverseDerived>
const typename InverseBase<InverseDerived>::LieGroup&
InverseBase<InverseDerived>::inverse() const
{
  return derived().original();
}

//==============================================================================
template <typename InverseDerived>
const InverseDerived& InverseBase<InverseDerived>::derived() const noexcept
{
  return *static_cast<const InverseDerived*>(this);
}

//==============================================================================
template <typename InverseDerived>
InverseDerived& InverseBase<InverseDerived>::derived() noexcept
{
  return *static_cast<InverseDerived*>(this);
}

} // namespace dart::math
