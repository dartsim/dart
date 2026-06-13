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

#include <dart/math/lie_group/group_product_base.hpp>
#include <dart/math/lie_group/group_product_inverse.hpp>

#include <dart/common/macros.hpp>

#include <dart/export.hpp>

#include <array>
#include <tuple>
#include <utility>

namespace Eigen::internal {

/// Specialization of Eigen::internal::traits for GroupProduct.
template <typename S, template <typename> class... ComponentsT>
struct traits<::dart::math::GroupProduct<S, ComponentsT...>>
{
  static_assert(
      sizeof...(ComponentsT) > 0,
      "GroupProduct must have at least one component.");

  using Scalar = S;

  // GroupProduct specifics
  static constexpr std::size_t ProductSize = sizeof...(ComponentsT);
  static constexpr std::array<int, ProductSize> ParamSizes{
      ComponentsT<S>::ParamSize...};
  static constexpr std::array<int, ProductSize> ParamSizeIndices
      = ::dart::math::detail::getGroupProductParamIndices<
          ComponentsT<S>::ParamSize...>();
  using Components = std::tuple<ComponentsT<S>...>;
  template <std::size_t Index>
  using Component = typename std::tuple_element<Index, Components>::type;
  template <std::size_t Index>
  using ComponentMap = ::Eigen::Map<Component<Index>>;
  template <std::size_t Index>
  using ConstComponentMap = ::Eigen::Map<const Component<Index>>;

  // LieGroup common
  static constexpr int Dim = ::dart::math::detail::sumGroupProductStaticSizes<
      ComponentsT<S>::Dim...>();
  static constexpr int DoF = ::dart::math::detail::sumGroupProductStaticSizes<
      ComponentsT<S>::DoF...>();
  static constexpr int MatrixRepDim
      = ::dart::math::detail::sumGroupProductStaticSizes<
          ComponentsT<S>::MatrixRepDim...>();
  static constexpr int ParamSize
      = ::dart::math::detail::sumGroupProductStaticSizes<
          ComponentsT<S>::ParamSize...>();
  using LieGroup = ::dart::math::GroupProduct<S, ComponentsT...>;
  using InverseType = ::dart::math::GroupProductInverse<LieGroup>;
  using MatrixType = ::Eigen::Matrix<S, MatrixRepDim, MatrixRepDim>;
  using Params = ::Eigen::Matrix<S, ParamSize, 1>;
  using PlainObject = LieGroup;
  using Tangent = ::Eigen::Matrix<S, DoF, 1>;
};

} // namespace Eigen::internal

namespace dart::math {

/// Direct product of one or more Lie groups.
///
/// A GroupProduct stores the parameters of each component contiguously and
/// applies composition, inverse, normalization, and logarithm componentwise.
template <typename S, template <typename> class... ComponentsT>
class GroupProduct : public GroupProductBase<GroupProduct<S, ComponentsT...>>
{
public:
  using Base = GroupProductBase<GroupProduct<S, ComponentsT...>>;
  using Scalar = typename Base::Scalar;

  // GroupProduct specifics
  static constexpr std::size_t ProductSize = Base::ProductSize;
  static constexpr auto ParamSizes = Base::ParamSizes;
  static constexpr auto ParamSizeIndices = Base::ParamSizeIndices;
  using Components = typename Base::Components;
  template <std::size_t Index>
  using Component = typename Base::template Component<Index>;
  template <std::size_t Index>
  using ComponentMap = typename Base::template ComponentMap<Index>;
  template <std::size_t Index>
  using ConstComponentMap = typename Base::template ConstComponentMap<Index>;

  // LieGroup common
  static constexpr int Dim = ::Eigen::internal::traits<GroupProduct>::Dim;
  static constexpr int DoF = ::Eigen::internal::traits<GroupProduct>::DoF;
  static constexpr int MatrixRepDim
      = ::Eigen::internal::traits<GroupProduct>::MatrixRepDim;
  static constexpr int ParamSize
      = ::Eigen::internal::traits<GroupProduct>::ParamSize;
  using LieGroup = typename Base::LieGroup;
  using MatrixType = typename Base::MatrixType;
  using Params = typename Base::Params;
  using PlainObject = typename Base::PlainObject;
  using Tangent = typename Base::Tangent;

  /// Default constructor that initializes every component to identity.
  GroupProduct();

  /// Constructs without initializing the underlying parameters.
  explicit GroupProduct(NoInitializeTag);

  DART_LIEGROUP_CONSTRUCTORS(GroupProduct);

  /// Constructs from concrete component values.
  explicit GroupProduct(const ComponentsT<S>&... components);

  /// Constructs from concrete component values.
  explicit GroupProduct(ComponentsT<S>&&... components);

  /// Copy assignment operator.
  GroupProduct& operator=(const GroupProduct& other);

  /// Move assignment operator.
  GroupProduct& operator=(GroupProduct&& other) noexcept;

  /// Returns the parameters of a specific component.
  template <std::size_t Index>
  [[nodiscard]] auto params() const;

  /// Returns the parameters of a specific component.
  template <std::size_t Index>
  [[nodiscard]] auto params();

  /// Returns the parameters of a specific component by runtime index.
  [[nodiscard]] auto params(std::size_t index) const;

  /// Returns the parameters of a specific component by runtime index.
  [[nodiscard]] auto params(std::size_t index);

  /// Returns the parameters of the whole product.
  [[nodiscard]] const Params& params() const;

  /// Returns the parameters of the whole product.
  [[nodiscard]] Params& params();

private:
  Params m_params;
};

} // namespace dart::math

//==============================================================================
// Implementation
//==============================================================================

namespace dart::math {

//==============================================================================
template <typename S, template <typename> class... ComponentsT>
GroupProduct<S, ComponentsT...>::GroupProduct()
{
  std::size_t index = 0;
  (..., [&] {
    using Component = ComponentsT<S>;
    constexpr int paramSize = Component::ParamSize;
    m_params.template segment<paramSize>(index)
        = Component::Identity().params();
    index += paramSize;
  }());
}

//==============================================================================
template <typename S, template <typename> class... ComponentsT>
GroupProduct<S, ComponentsT...>::GroupProduct(NoInitializeTag)
{
  // Do nothing
}

//==============================================================================
template <typename S, template <typename> class... ComponentsT>
GroupProduct<S, ComponentsT...>::GroupProduct(
    const ComponentsT<S>&... components)
{
  std::size_t index = 0;
  (..., [&] {
    using Component = ComponentsT<S>;
    constexpr int paramSize = Component::ParamSize;
    m_params.template segment<paramSize>(index) = components.params();
    index += paramSize;
  }());
}

//==============================================================================
template <typename S, template <typename> class... ComponentsT>
GroupProduct<S, ComponentsT...>::GroupProduct(ComponentsT<S>&&... components)
{
  std::size_t index = 0;
  (..., [&] {
    using Component = ComponentsT<S>;
    constexpr int paramSize = Component::ParamSize;
    m_params.template segment<paramSize>(index) = components.params();
    index += paramSize;
  }());
}

//==============================================================================
template <typename S, template <typename> class... ComponentsT>
GroupProduct<S, ComponentsT...>& GroupProduct<S, ComponentsT...>::operator=(
    const GroupProduct& other)
{
  m_params = other.m_params;
  return *this;
}

//==============================================================================
template <typename S, template <typename> class... ComponentsT>
GroupProduct<S, ComponentsT...>& GroupProduct<S, ComponentsT...>::operator=(
    GroupProduct&& other) noexcept
{
  m_params = std::move(other.m_params);
  return *this;
}

//==============================================================================
template <typename S, template <typename> class... ComponentsT>
template <std::size_t Index>
auto GroupProduct<S, ComponentsT...>::params() const
{
  return m_params.template segment<Component<Index>::ParamSize>(
      std::get<Index>(ParamSizeIndices));
}

//==============================================================================
template <typename S, template <typename> class... ComponentsT>
template <std::size_t Index>
auto GroupProduct<S, ComponentsT...>::params()
{
  return m_params.template segment<Component<Index>::ParamSize>(
      std::get<Index>(ParamSizeIndices));
}

//==============================================================================
template <typename S, template <typename> class... ComponentsT>
auto GroupProduct<S, ComponentsT...>::params(std::size_t index) const
{
  DART_ASSERT(index < ProductSize);
  return m_params.segment(ParamSizeIndices[index], ParamSizes[index]);
}

//==============================================================================
template <typename S, template <typename> class... ComponentsT>
auto GroupProduct<S, ComponentsT...>::params(std::size_t index)
{
  DART_ASSERT(index < ProductSize);
  return m_params.segment(ParamSizeIndices[index], ParamSizes[index]);
}

//==============================================================================
template <typename S, template <typename> class... ComponentsT>
const typename GroupProduct<S, ComponentsT...>::Params&
GroupProduct<S, ComponentsT...>::params() const
{
  return m_params;
}

//==============================================================================
template <typename S, template <typename> class... ComponentsT>
typename GroupProduct<S, ComponentsT...>::Params&
GroupProduct<S, ComponentsT...>::params()
{
  return m_params;
}

} // namespace dart::math
