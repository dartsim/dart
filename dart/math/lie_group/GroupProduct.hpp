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

#include <dart/math/lie_group/GroupProductBase.hpp>

namespace Eigen::internal {

// TODO(JS): Move to a dedicated header file
/// @brief Specialization of Eigen::internal::traits for GroupProduct
template <typename S, template <typename> class... T>
struct traits<::dart::math::GroupProduct<S, T...>>
{
  using Scalar = S;

  // GroupProduct specifics
  static constexpr std::size_t ProductSize = sizeof...(T);
  static constexpr auto ParamSizes = std::array{T<S>::ParamSize...};
  static constexpr auto ParamSizeIndices
      = ::dart::math::detail::GetIndices<T<S>::ParamSize...>();
  using Components = std::tuple<T<S>...>;
  template <std::size_t Index>
  using Component = typename std::tuple_element<Index, Components>::type;
  template <std::size_t Index>
  using ComponentMap = ::Eigen::Map<Component<Index>>;
  template <std::size_t Index>
  using ConstComponentMap = ::Eigen::Map<const Component<Index>>;

  // LieGroup common
#if defined(_MSC_VER)
  static constexpr int ParamSize
      = ::dart::math::detail::Sum(T<S>::ParamSize...);
  static constexpr int Dim = ::dart::math::detail::Sum(T<S>::Dim...);
  static constexpr int DoF = ::dart::math::detail::Sum(T<S>::DoF...);
  static constexpr int MatrixRepDim
      = ::dart::math::detail::Sum(T<S>::MatrixRepDim...);
#else
  static constexpr int ParamSize = (T<S>::ParamSize + ...);
  static constexpr int Dim = (T<S>::Dim + ...);
  static constexpr int DoF = (T<S>::DoF + ...);
  static constexpr int MatrixRepDim = (T<S>::DoF + ...);
#endif
  using Params = ::Eigen::Matrix<S, ParamSize, 1>;
  using PlainObject = ::dart::math::GroupProduct<S, T...>;
  using MatrixType = ::Eigen::Matrix<S, MatrixRepDim, MatrixRepDim>;
  using Tangent = ::Eigen::Matrix<S, Dim, 1>;
};

} // namespace Eigen::internal

namespace dart::math {

/// @brief GroupProduct is a specialization of LieGroupBase for
/// GroupProduct
/// @tparam S The scalar type
template <typename S, template <typename> class... T>
class GroupProduct : public GroupProductBase<GroupProduct<S, T...>>
{
public:
  using Base = GroupProductBase<GroupProduct<S, T...>>;
  using This = GroupProduct<S, T...>;
  using Scalar = typename Base::Scalar;

  // GroupProduct specifics
  static constexpr std::size_t ProductSize = Base::ProductSize;
  static constexpr auto ParamSizes = Base::ParamSizes;
  static constexpr auto ParamSizeIndices = Base::ParamSizeIndices;
  using Components = typename Base::Components;
  template <std::size_t Index>
  using Component = typename Base::template Component<Index>;
  template <size_t Index>
  using ComponentMap = typename Base::template ComponentMap<Index>;
  template <size_t Index>
  using ConstComponentMap = typename Base::template ConstComponentMap<Index>;

  // LieGroup common
  using Params = typename Base::Params;
  using PlainObject = typename Base::PlainObject;
  using MatrixType = typename Base::MatrixType;
  using Tangent = typename Base::Tangent;

  using Base::Tolerance;

  /// Default constructor that initializes the components to identity
  GroupProduct();

  /// Default constructor that does not initialize the components to identity
  ///
  /// This constructor is only available when all the components are default
  /// constructible. Otherwise, this constructor is deleted. This constructor is
  /// useful when you want to use GroupProduct as a member variable of another
  /// class, and you want to initialize the GroupProduct later.
  GroupProduct(NoInitializeTag);

  /// Constructs a GroupProduct from the given components
  ///
  /// @param[in] components The components of the GroupProduct. The numer and
  /// order of components must match the template parameters.
  GroupProduct(const T<S>&... components);

  /// Constructs a GroupProduct from the given components
  ///
  /// @param[in] components The components of the GroupProduct. The numer and
  /// order of components must match the template parameters.
  GroupProduct(T<S>&&... components);

  DART_DEFINE_CONSTRUCTORS_FOR_CONCRETE(GroupProduct);

  /// Returns the parameters of the specific component group
  template <std::size_t Index_>
  [[nodiscard]] auto params() const;

  /// Returns the parameters of the specific component group
  template <std::size_t Index_>
  [[nodiscard]] auto params();

  /// Returns the parameters of the whole group product
  [[nodiscard]] auto params(std::size_t index) const;

  /// Returns the parameters of the whole group product
  [[nodiscard]] auto params(std::size_t index);

  /// Returns the parameters of the whole group product
  [[nodiscard]] const Params& params() const;

  /// Returns the parameters of the whole group product
  [[nodiscard]] Params& params();

private:
  static_assert(
      sizeof...(T) > 0, "GroupProduct must have at least one component");

  /// The underlying quaternion parameters
  Params m_params;
};

} // namespace dart::math

//==============================================================================
// Implementation
//==============================================================================

namespace dart::math {

//==============================================================================
template <typename S, template <typename> class... T>
GroupProduct<S, T...>::GroupProduct()
{
  std::size_t index = 0;
  (..., [&] {
    m_params.template segment<T<S>::ParamSize>(index)
        = T<S>::Identity().params();
    index += T<S>::ParamSize;
  }());
}

//==============================================================================
template <typename S, template <typename> class... T>
GroupProduct<S, T...>::GroupProduct(NoInitializeTag)
{
  // Do nothing
}

//==============================================================================
template <typename S, template <typename> class... T>
GroupProduct<S, T...>::GroupProduct(const T<S>&... components)
{
  std::size_t index = 0;
  (..., [&] {
    m_params.template segment<T<S>::ParamSize>(index) = components.params();
    index += T<S>::ParamSize;
  }());
}

//==============================================================================
template <typename S, template <typename> class... T>
GroupProduct<S, T...>::GroupProduct(T<S>&&... components)
{
  std::size_t index = 0;
  (..., [&] {
    m_params.template segment<T<S>::ParamSize>(index)
        = std::move(components.params());
    index += T<S>::ParamSize;
  }());
}

//==============================================================================
template <typename S, template <typename> class... T>
template <std::size_t Index_>
auto GroupProduct<S, T...>::params() const
{
  return m_params.template segment<Component<Index_>::ParamSize>(
      std::get<Index_>(ParamSizeIndices));
}

//==============================================================================
template <typename S, template <typename> class... T>
auto GroupProduct<S, T...>::params(std::size_t index)
{
  return m_params.segment(ParamSizeIndices[index], ParamSizes[index]);
}

//==============================================================================
template <typename S, template <typename> class... T>
auto GroupProduct<S, T...>::params(std::size_t index) const
{
  return m_params.segment(ParamSizeIndices[index], ParamSizes[index]);
}

//==============================================================================
template <typename S, template <typename> class... T>
template <std::size_t Index_>
auto GroupProduct<S, T...>::params()
{
  return m_params.template segment<Component<Index_>::ParamSize>(
      std::get<Index_>(ParamSizeIndices));
}

//==============================================================================
template <typename S, template <typename> class... T>
const typename GroupProduct<S, T...>::Params& GroupProduct<S, T...>::params()
    const
{
  return m_params;
}

//==============================================================================
template <typename S, template <typename> class... T>
typename GroupProduct<S, T...>::Params& GroupProduct<S, T...>::params()
{
  return m_params;
}

} // namespace dart::math
