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

#include <dart/math/lie_group/LieGroupBase.hpp>

namespace dart::math {

// TODO(JS): Move this to under detail/
namespace detail {

template <size_t PackSize, int i, int j, int... Args>
constexpr auto GetIndicesImpl()
{
  if constexpr (PackSize == 1) {
    return std::array{0, Args...};
  } else {
    return GetIndicesImpl<PackSize - 1, i + j, Args..., i + j>();
  }
}

template <int... Args>
constexpr auto GetIndices()
{
  return GetIndicesImpl<sizeof...(Args), 0, Args...>();
}

#if defined(_MSC_VER)
template <typename T>
constexpr T Sum(T t)
{
  return t;
}

template <typename T, typename... Rest>
constexpr T Sum(T t, Rest... rest)
{
  return t + Sum(rest...);
}
#endif

} // namespace detail

/// @brief Base class for GroupProduct
/// @tparam Derived The derived class
template <typename Derived>
class GroupProductBase : public LieGroupBase<Derived>
{
public:
  using Base = LieGroupBase<Derived>;
  using Scalar = typename Base::Scalar;

  // GroupProduct specifics
  static constexpr int ProductSize
      = ::Eigen::internal::traits<Derived>::ProductSize;
  static constexpr auto ParamSizeIndices
      = ::Eigen::internal::traits<Derived>::ParamSizeIndices;
  using Components = typename ::Eigen::internal::traits<Derived>::Components;
  template <std::size_t Index>
  using Component =
      typename ::Eigen::internal::traits<Derived>::template Component<Index>;
  template <size_t Index>
  using ComponentMap =
      typename ::Eigen::internal::traits<Derived>::template ComponentMap<Index>;
  template <size_t Index>
  using ConstComponentMap = typename ::Eigen::internal::traits<
      Derived>::template ConstComponentMap<Index>;

  // LieGroupBase common
  using Base::Dim;
  using Base::DoF;
  using Base::MatrixRepDim;
  using Base::ParamSize;
  using Params = typename Base::Params;
  using PlainObject = typename Base::PlainObject;
  using MatrixType = typename Base::MatrixType;
  using Tangent = typename Base::Tangent;

  using Base::Tolerance;

  using Base::operator=;
  using Base::derived;
  using Base::params;

  /**
   * @brief Sets the components of the group product to random values
   *
   * @return Derived& Reference to this GroupProduct
   */
  Derived& setRandom();

  /**
   * @brief Returns the map of the component at the given index
   *
   * @tparam Index The index of the component
   * @return ConstComponentMap<Index> The map of the component at the given
   * index
   */
  template <size_t Index>
  ConstComponentMap<Index> get() const;

  /**
   * @brief Returns the map of the component at the given index
   *
   * @tparam Index The index of the component
   * @return ComponentMap<Index> The map of the component at the given index
   */
  template <size_t Index>
  ComponentMap<Index> get();

private:
  template <std::size_t... Index_>
  Derived& setRandom(std::integer_sequence<std::size_t, Index_...>);
};

} // namespace dart::math

//==============================================================================
// Implementation
//==============================================================================

namespace dart::math {

//==============================================================================
template <typename Derived>
Derived& GroupProductBase<Derived>::setRandom()
{
  return setRandom(
      std::make_integer_sequence<std::size_t, Derived::ProductSize>{});
}

//==============================================================================
template <typename Derived>
template <std::size_t... Index_>
Derived& GroupProductBase<Derived>::setRandom(
    std::integer_sequence<std::size_t, Index_...>)
{
  (..., [&] {
    get<Index_>().setRandom();
  }());
  return derived();
}

//==============================================================================
template <typename Derived>
template <size_t Index>
typename GroupProductBase<Derived>::template ConstComponentMap<Index>
GroupProductBase<Derived>::get() const
{
  return ConstComponentMap<Index>(
      params().data() + std::get<Index>(ParamSizeIndices));
}

//==============================================================================
template <typename Derived>
template <size_t Index>
typename GroupProductBase<Derived>::template ComponentMap<Index>
GroupProductBase<Derived>::get()
{
  return ComponentMap<Index>(
      params().data() + std::get<Index>(ParamSizeIndices));
}

} // namespace dart::math
