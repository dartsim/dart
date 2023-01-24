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

#include <dart/math/lie_group/LieGroupProductBase.hpp>

namespace Eigen::internal {

// TODO(JS): Move to a dedicated header file
/// @brief Specialization of Eigen::internal::traits for LieGroupProduct
template <typename S, template <typename> class... T>
struct traits<::dart::math::LieGroupProduct<S, T...>>
{
  using Scalar = S;

  // LieGroupProduct specifics
  static constexpr std::size_t ProductSize = sizeof...(T);
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
  static constexpr int MatrixDim
      = ::dart::math::detail::Sum(T<S>::MatrixDim...);
#else
  static constexpr int ParamSize = (T<S>::ParamSize + ...);
  static constexpr int Dim = (T<S>::Dim + ...);
  static constexpr int DoF = (T<S>::DoF + ...);
  static constexpr int MatrixDim = (T<S>::DoF + ...);
#endif
  using Params = ::Eigen::Matrix<S, ParamSize, 1>;
  using PlainObject = ::dart::math::LieGroupProduct<S, T...>;
  using MatrixType = ::Eigen::Matrix<S, MatrixDim, MatrixDim>;
  using Tangent = ::Eigen::Matrix<S, Dim, 1>;
};

} // namespace Eigen::internal

namespace dart::math {

/// @brief LieGroupProduct is a specialization of LieGroupBase for
/// LieGroupProduct
/// @tparam S The scalar type
/// @tparam Options_ The options for the underlying Eigen::Matrix
template <typename S, template <typename> class... T>
class LieGroupProduct : public LieGroupProductBase<LieGroupProduct<S, T...>>
{
public:
  using Base = LieGroupProductBase<LieGroupProduct<S, T...>>;
  using This = LieGroupProduct<S, T...>;
  using Scalar = typename Base::Scalar;

  // LieGroupProduct specifics
  static constexpr std::size_t ProductSize = Base::ProductSize;
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

  /// Default constructor that initializes the quaternion to identity
  LieGroupProduct();

  /// Constructs a LieGroupProduct from the given components
  ///
  /// @param[in] components The components of the LieGroupProduct. The numer and
  /// order of components must match the template parameters.
  LieGroupProduct(const T<S>&... components);

  /// Constructs a LieGroupProduct from the given components
  ///
  /// @param[in] components The components of the LieGroupProduct. The numer and
  /// order of components must match the template parameters.
  LieGroupProduct(T<S>&&... components);

  /// Returns the coefficients of the underlying quaternion
  [[nodiscard]] const Params& params() const;

  /// Returns the coefficients of the underlying quaternion
  [[nodiscard]] Params& params();

private:
#if defined(_MSC_VER)
  template <std::size_t... Index_>
  LieGroupProduct(detail::int_sequence<Index_...>);

  template <std::size_t... Index_>
  LieGroupProduct(detail::int_sequence<Index_...>, const T<S>&... components);

  template <std::size_t... Index_>
  LieGroupProduct(detail::int_sequence<Index_...>, T<S>&&... components);
#endif
  /// The underlying quaternion coefficients
  Params m_params;
};

} // namespace dart::math

//==============================================================================
// Implementation
//==============================================================================

namespace dart::math {

#if defined(_MSC_VER)
//==============================================================================
template <typename S, template <typename> class... T>
LieGroupProduct<S, T...>::LieGroupProduct()
  : LieGroupProduct(detail::make_int_sequence<ProductSize>{})
{
  // Do nothing
}

//==============================================================================
template <typename S, template <typename> class... T>
template <std::size_t... Index_>
LieGroupProduct<S, T...>::LieGroupProduct(detail::int_sequence<Index_...>)
{
  (void)(std::initializer_list<int>{
      ((m_params.template segment<Component<Index_>::ParamSize>(
            std::get<Index_>(Eigen::internal::traits<This>::ParamSizeIndices))
        = Component<Index_>::Identity().params()),
       0)...});
}

//==============================================================================
template <typename S, template <typename> class... T>
LieGroupProduct<S, T...>::LieGroupProduct(const T<S>&... components)
  : LieGroupProduct(detail::make_int_sequence<ProductSize>{}, components...)
{
  // Do nothing
}

//==============================================================================
template <typename S, template <typename> class... T>
template <std::size_t... Index_>
LieGroupProduct<S, T...>::LieGroupProduct(
    detail::int_sequence<Index_...>, const T<S>&... components)
{
  (void)(std::initializer_list<int>{
      ((m_params.template segment<Component<Index_>::ParamSize>(
            std::get<Index_>(Eigen::internal::traits<This>::ParamSizeIndices))
        = components.params()),
       0)...});
}

//==============================================================================
template <typename S, template <typename> class... T>
LieGroupProduct<S, T...>::LieGroupProduct(T<S>&&... components)
  : LieGroupProduct(
      detail::make_int_sequence<ProductSize>{},
      std::forward<T<S>...>(components)...)
{
  // Do nothing
}

//==============================================================================
template <typename S, template <typename> class... T>
template <std::size_t... Index_>
LieGroupProduct<S, T...>::LieGroupProduct(
    detail::int_sequence<Index_...>, T<S>&&... components)
{
  (void)(std::initializer_list<int>{
      ((m_params.template segment<Component<Index_>::ParamSize>(
            std::get<Index_>(Eigen::internal::traits<This>::ParamSizeIndices))
        = std::move(components.params())),
       0)...});
}
#else
//==============================================================================
template <typename S, template <typename> class... T>
LieGroupProduct<S, T...>::LieGroupProduct()
{
  auto assign = [&](auto&& arg, std::size_t index) {
    m_params.template segment<arg.ParamSize>(index) = arg.params();
  };
  std::apply(
      [&](auto... i) { (assign(T<S>::Identity(), i), ...); }, ParamSizeIndices);
}

//==============================================================================
template <typename S, template <typename> class... T>
LieGroupProduct<S, T...>::LieGroupProduct(const T<S>&... components)
{
  auto assign = [&](const auto& arg, std::size_t index) {
    m_params.template segment<arg.ParamSize>(index) = arg.params();
  };
  std::apply(
      [&](auto... i) { (assign(components, i), ...); }, ParamSizeIndices);
}

//==============================================================================
template <typename S, template <typename> class... T>
LieGroupProduct<S, T...>::LieGroupProduct(T<S>&&... components)
{
  auto assign = [&](auto&& arg, std::size_t index) {
    m_params.template segment<arg.ParamSize>(index) = std::move(arg.params());
  };
  std::apply(
      [&](auto... i) { (assign(std::move(components), i), ...); },
      ParamSizeIndices);
}
#endif

//==============================================================================
template <typename S, template <typename> class... T>
const typename LieGroupProduct<S, T...>::Params&
LieGroupProduct<S, T...>::params() const
{
  return m_params;
}

//==============================================================================
template <typename S, template <typename> class... T>
typename LieGroupProduct<S, T...>::Params& LieGroupProduct<S, T...>::params()
{
  return m_params;
}

} // namespace dart::math
