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

#include <dart/math/lie_group/lie_group_base.hpp>

#include <dart/common/macros.hpp>

#include <array>
#include <tuple>
#include <type_traits>
#include <utility>

#include <cstddef>

namespace dart::math::detail {

template <int... Values>
struct GroupProductStaticSum;

template <>
struct GroupProductStaticSum<>
{
  static constexpr int value = 0;
};

template <int First, int... Rest>
struct GroupProductStaticSum<First, Rest...>
{
  static constexpr int value = First + GroupProductStaticSum<Rest...>::value;
};

template <int... Values>
constexpr int sumGroupProductStaticSizes()
{
  return GroupProductStaticSum<Values...>::value;
}

template <int... Sizes>
constexpr auto getGroupProductParamIndices()
{
  std::array<int, sizeof...(Sizes)> indices{};
  int offset = 0;
  std::size_t index = 0;
  ((indices[index++] = offset, offset += Sizes), ...);
  return indices;
}

template <typename Tangent>
decltype(auto) getTangentParams(const Tangent& tangent)
{
  if constexpr (requires { tangent.params(); }) {
    return tangent.params();
  } else {
    return tangent;
  }
}

} // namespace dart::math::detail

namespace dart::math {

/// Base class for direct products of Lie groups.
///
/// GroupProductBase applies common Lie-group operations componentwise. Concrete
/// products and Eigen maps provide storage through params().
template <typename Derived>
class GroupProductBase : public LieGroupBase<Derived>
{
public:
  using Base = LieGroupBase<Derived>;
  using Scalar = typename Base::Scalar;

  // GroupProduct specifics
  static constexpr std::size_t ProductSize
      = ::Eigen::internal::traits<Derived>::ProductSize;
  static constexpr auto ParamSizes
      = ::Eigen::internal::traits<Derived>::ParamSizes;
  static constexpr auto ParamSizeIndices
      = ::Eigen::internal::traits<Derived>::ParamSizeIndices;
  using Components = typename ::Eigen::internal::traits<Derived>::Components;
  template <std::size_t Index>
  using Component =
      typename ::Eigen::internal::traits<Derived>::template Component<Index>;
  template <std::size_t Index>
  using ComponentMap =
      typename ::Eigen::internal::traits<Derived>::template ComponentMap<Index>;
  template <std::size_t Index>
  using ConstComponentMap = typename ::Eigen::internal::traits<
      Derived>::template ConstComponentMap<Index>;

  // LieGroup common
  using Base::Dim;
  using Base::DoF;
  using Base::MatrixRepDim;
  using Base::ParamSize;
  using InverseType = typename Base::InverseType;
  using LieGroup = typename Base::LieGroup;
  using MatrixType = typename Base::MatrixType;
  using Params = typename Base::Params;
  using PlainObject = LieGroup;
  using Tangent = typename Base::Tangent;
  using AdjointMatrix = Matrix<Scalar, DoF, DoF>;

  using Base::derived;
  using Base::operator=;
  using Base::params;

  /// Composes this product with another product of the same component shape.
  template <typename OtherDerived>
  [[nodiscard]] LieGroup operator*(
      const LieGroupBase<OtherDerived>& other) const;

  /// Sets every component to a random Lie-group element.
  Derived& setRandom();

  /// Inverts every component in place.
  Derived& inverseInPlace();

  /// Normalizes every component in place.
  void normalize();

  /// Returns the componentwise logarithm concatenated in parameter order.
  [[nodiscard]] Tangent log(Scalar tol = LieGroupTol<Scalar>()) const;

  /// Returns the componentwise logarithm and its block-diagonal Jacobian.
  template <typename MatrixDerived>
  [[nodiscard]] Tangent log(
      Eigen::MatrixBase<MatrixDerived>* jacobian,
      Scalar tol = LieGroupTol<Scalar>()) const;

  /// Returns the block-diagonal adjoint matrix of all components.
  [[nodiscard]] AdjointMatrix toAdjointMatrix() const;

  /// Returns the block-diagonal matrix representation of all components.
  [[nodiscard]] MatrixType toMatrix() const;

  /// Returns a map of the component at the given index.
  template <std::size_t Index>
  [[nodiscard]] const ConstComponentMap<Index> get() const;

  /// Returns a mutable map of the component at the given index.
  template <std::size_t Index>
  [[nodiscard]] ComponentMap<Index> get();

private:
  template <typename OtherDerived, std::size_t... Indices>
  [[nodiscard]] LieGroup compose(
      std::integer_sequence<std::size_t, Indices...>,
      const LieGroupBase<OtherDerived>& other) const;

  template <std::size_t... Indices>
  Derived& setRandom(std::integer_sequence<std::size_t, Indices...>);

  template <std::size_t... Indices>
  Derived& inverseInPlace(std::integer_sequence<std::size_t, Indices...>);

  template <std::size_t... Indices>
  void normalize(std::integer_sequence<std::size_t, Indices...>);

  template <std::size_t... Indices>
  [[nodiscard]] Tangent log(
      std::integer_sequence<std::size_t, Indices...>, Scalar tol) const;

  template <typename MatrixDerived, std::size_t... Indices>
  [[nodiscard]] Tangent log(
      std::integer_sequence<std::size_t, Indices...>,
      Eigen::MatrixBase<MatrixDerived>* jacobian,
      Scalar tol) const;

  template <std::size_t... Indices>
  [[nodiscard]] AdjointMatrix toAdjointMatrix(
      std::integer_sequence<std::size_t, Indices...>) const;

  template <std::size_t... Indices>
  [[nodiscard]] MatrixType toMatrix(
      std::integer_sequence<std::size_t, Indices...>) const;
};

} // namespace dart::math

//==============================================================================
// Implementation
//==============================================================================

namespace dart::math {

//==============================================================================
template <typename Derived>
template <typename OtherDerived>
typename GroupProductBase<Derived>::LieGroup
GroupProductBase<Derived>::operator*(
    const LieGroupBase<OtherDerived>& other) const
{
  static_assert(
      ProductSize == OtherDerived::ProductSize,
      "GroupProduct operands must have the same number of components.");
  return compose(std::make_integer_sequence<std::size_t, ProductSize>{}, other);
}

//==============================================================================
template <typename Derived>
template <typename OtherDerived, std::size_t... Indices>
typename GroupProductBase<Derived>::LieGroup GroupProductBase<Derived>::compose(
    std::integer_sequence<std::size_t, Indices...>,
    const LieGroupBase<OtherDerived>& other) const
{
  static_assert(
      (std::is_same_v<
           Component<Indices>,
           typename OtherDerived::template Component<Indices>>
       && ...),
      "GroupProduct operands must have the same component types.");
  return LieGroup(
      (get<Indices>() * other.derived().template get<Indices>())...);
}

//==============================================================================
template <typename Derived>
Derived& GroupProductBase<Derived>::setRandom()
{
  return setRandom(std::make_integer_sequence<std::size_t, ProductSize>{});
}

//==============================================================================
template <typename Derived>
template <std::size_t... Indices>
Derived& GroupProductBase<Derived>::setRandom(
    std::integer_sequence<std::size_t, Indices...>)
{
  (get<Indices>().setRandom(), ...);
  return derived();
}

//==============================================================================
template <typename Derived>
Derived& GroupProductBase<Derived>::inverseInPlace()
{
  return inverseInPlace(std::make_integer_sequence<std::size_t, ProductSize>{});
}

//==============================================================================
template <typename Derived>
template <std::size_t... Indices>
Derived& GroupProductBase<Derived>::inverseInPlace(
    std::integer_sequence<std::size_t, Indices...>)
{
  (get<Indices>().inverseInPlace(), ...);
  return derived();
}

//==============================================================================
template <typename Derived>
void GroupProductBase<Derived>::normalize()
{
  normalize(std::make_integer_sequence<std::size_t, ProductSize>{});
}

//==============================================================================
template <typename Derived>
template <std::size_t... Indices>
void GroupProductBase<Derived>::normalize(
    std::integer_sequence<std::size_t, Indices...>)
{
  (get<Indices>().normalize(), ...);
}

//==============================================================================
template <typename Derived>
typename GroupProductBase<Derived>::Tangent GroupProductBase<Derived>::log(
    Scalar tol) const
{
  return log(std::make_integer_sequence<std::size_t, ProductSize>{}, tol);
}

//==============================================================================
template <typename Derived>
template <std::size_t... Indices>
typename GroupProductBase<Derived>::Tangent GroupProductBase<Derived>::log(
    std::integer_sequence<std::size_t, Indices...>, Scalar tol) const
{
  Tangent out;
  int offset = 0;
  ((out.template segment<Component<Indices>::DoF>(offset)
    = detail::getTangentParams(get<Indices>().log(tol)),
    offset += Component<Indices>::DoF),
   ...);
  return out;
}

//==============================================================================
template <typename Derived>
template <typename MatrixDerived>
typename GroupProductBase<Derived>::Tangent GroupProductBase<Derived>::log(
    Eigen::MatrixBase<MatrixDerived>* jacobian, Scalar tol) const
{
  if (jacobian != nullptr) {
    jacobian->derived().setZero();
  }

  return log(
      std::make_integer_sequence<std::size_t, ProductSize>{}, jacobian, tol);
}

//==============================================================================
template <typename Derived>
template <typename MatrixDerived, std::size_t... Indices>
typename GroupProductBase<Derived>::Tangent GroupProductBase<Derived>::log(
    std::integer_sequence<std::size_t, Indices...>,
    Eigen::MatrixBase<MatrixDerived>* jacobian,
    Scalar tol) const
{
  Tangent out;
  int offset = 0;
  (([&] {
     constexpr int componentDoF = Component<Indices>::DoF;
     if (jacobian != nullptr) {
       auto jacobianBlock
           = jacobian->derived().template block<componentDoF, componentDoF>(
               offset, offset);
       out.template segment<componentDoF>(offset)
           = detail::getTangentParams(get<Indices>().log(&jacobianBlock, tol));
     } else {
       out.template segment<componentDoF>(offset)
           = detail::getTangentParams(get<Indices>().log(tol));
     }
     offset += componentDoF;
   }()),
   ...);
  return out;
}

//==============================================================================
template <typename Derived>
typename GroupProductBase<Derived>::AdjointMatrix
GroupProductBase<Derived>::toAdjointMatrix() const
{
  return toAdjointMatrix(
      std::make_integer_sequence<std::size_t, ProductSize>{});
}

//==============================================================================
template <typename Derived>
template <std::size_t... Indices>
typename GroupProductBase<Derived>::AdjointMatrix
GroupProductBase<Derived>::toAdjointMatrix(
    std::integer_sequence<std::size_t, Indices...>) const
{
  AdjointMatrix out = AdjointMatrix::Zero();
  int offset = 0;
  ((out.template block<Component<Indices>::DoF, Component<Indices>::DoF>(
        offset, offset) = get<Indices>().toAdjointMatrix(),
    offset += Component<Indices>::DoF),
   ...);
  return out;
}

//==============================================================================
template <typename Derived>
typename GroupProductBase<Derived>::MatrixType
GroupProductBase<Derived>::toMatrix() const
{
  return toMatrix(std::make_integer_sequence<std::size_t, ProductSize>{});
}

//==============================================================================
template <typename Derived>
template <std::size_t... Indices>
typename GroupProductBase<Derived>::MatrixType
GroupProductBase<Derived>::toMatrix(
    std::integer_sequence<std::size_t, Indices...>) const
{
  MatrixType out = MatrixType::Zero();
  int offset = 0;
  ((out.template block<
        Component<Indices>::MatrixRepDim,
        Component<Indices>::MatrixRepDim>(offset, offset)
    = get<Indices>().toMatrix(),
    offset += Component<Indices>::MatrixRepDim),
   ...);
  return out;
}

//==============================================================================
template <typename Derived>
template <std::size_t Index>
const typename GroupProductBase<Derived>::template ConstComponentMap<Index>
GroupProductBase<Derived>::get() const
{
  return detail::makeLieGroupMap<ConstComponentMap<Index>>(
      params(), std::get<Index>(ParamSizeIndices));
}

//==============================================================================
template <typename Derived>
template <std::size_t Index>
typename GroupProductBase<Derived>::template ComponentMap<Index>
GroupProductBase<Derived>::get()
{
  return detail::makeLieGroupMap<ComponentMap<Index>>(
      params(), std::get<Index>(ParamSizeIndices));
}

} // namespace dart::math
