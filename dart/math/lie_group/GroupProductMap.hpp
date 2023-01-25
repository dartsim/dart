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
struct traits<::Eigen::Map<const ::dart::math::GroupProduct<S, T...>>>
  : traits<const ::dart::math::GroupProduct<S, T...>>
{
  using Base = const ::dart::math::GroupProduct<S, T...>;
  using Scalar = S;

  // LieGroup common
  static constexpr auto ParamSize = Base::ParamSize;
  using Params = ::Eigen::Map<const ::Eigen::Matrix<S, ParamSize, 1>>;
};

/// @brief Specialization of Eigen::internal::traits for GroupProduct
template <typename S, template <typename> class... T>
struct traits<::Eigen::Map<::dart::math::GroupProduct<S, T...>>>
  : traits<::dart::math::GroupProduct<S, T...>>
{
  using Base = ::dart::math::GroupProduct<S, T...>;
  using Scalar = S;

  // LieGroup common
  static constexpr auto ParamSize = Base::ParamSize;
  using Params = ::Eigen::Map<::Eigen::Matrix<S, ParamSize, 1>>;
};

} // namespace Eigen::internal

namespace Eigen {

/// @brief GroupProduct is a specialization of LieGroupBase for
/// GroupProduct
/// @tparam S The scalar type
template <typename S, template <typename> class... T>
class Map<const ::dart::math::GroupProduct<S, T...>>
  : public ::dart::math::GroupProductBase<
        Map<const ::dart::math::GroupProduct<S, T...>>>
{
public:
  using Base = ::dart::math::GroupProductBase<
      Map<const ::dart::math::GroupProduct<S, T...>>>;
  using This = Map<const ::dart::math::GroupProduct<S, T...>>;
  using Scalar = typename Base::Scalar;

  // LieGroup common
  using Params = typename Base::Params;

  /** Constructor */
  explicit Map(const Scalar* data);

  /** Returns the underlying parameters */
  [[nodiscard]] const Params& params() const;

private:
  /** The underlying parameters */
  Params m_params;
};

/// @brief GroupProduct is a specialization of LieGroupBase for
/// GroupProduct
/// @tparam S The scalar type
template <typename S, template <typename> class... T>
class Map<::dart::math::GroupProduct<S, T...>>
  : public ::dart::math::GroupProductBase<
        Map<::dart::math::GroupProduct<S, T...>>>
{
public:
  using Base = ::dart::math::GroupProductBase<
      Map<::dart::math::GroupProduct<S, T...>>>;
  using This = Map<::dart::math::GroupProduct<S, T...>>;
  using Scalar = typename Base::Scalar;

  // LieGroup common
  using Params = typename Base::Params;

  /** Constructor */
  explicit Map(Scalar* data);

  /** Returns the underlying parameters */
  [[nodiscard]] const Params& params() const;

  /** Returns the underlying parameters */
  [[nodiscard]] Params& params();

private:
  /** The underlying parameters */
  Params m_params;
};

} // namespace Eigen

//==============================================================================
// Implementation
//==============================================================================

namespace Eigen {

//==============================================================================
template <typename S, template <typename> class... T>
Map<const ::dart::math::GroupProduct<S, T...>>::Map(const Scalar* data)
  : m_params(data)
{
  // Do nothing
}

//==============================================================================
template <typename S, template <typename> class... T>
const typename Map<const ::dart::math::GroupProduct<S, T...>>::Params&
Map<const ::dart::math::GroupProduct<S, T...>>::params() const
{
  return m_params;
}

//==============================================================================
template <typename S, template <typename> class... T>
Map<::dart::math::GroupProduct<S, T...>>::Map(Scalar* data) : m_params(data)
{
  // Do nothing
}

//==============================================================================
template <typename S, template <typename> class... T>
const typename Map<::dart::math::GroupProduct<S, T...>>::Params&
Map<::dart::math::GroupProduct<S, T...>>::params() const
{
  return m_params;
}

//==============================================================================
template <typename S, template <typename> class... T>
typename Map<::dart::math::GroupProduct<S, T...>>::Params&
Map<::dart::math::GroupProduct<S, T...>>::params()
{
  return m_params;
}

} // namespace Eigen
