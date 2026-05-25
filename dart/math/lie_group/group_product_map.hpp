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

#include <dart/math/lie_group/group_product.hpp>

namespace Eigen::internal {

template <
    typename S,
    int Options,
    typename StrideType,
    template <typename> class... ComponentsT>
struct traits<::Eigen::Map<
    const ::dart::math::GroupProduct<S, ComponentsT...>,
    Options,
    StrideType>> : traits<::dart::math::GroupProduct<S, ComponentsT...>>
{
  using Base = traits<::dart::math::GroupProduct<S, ComponentsT...>>;
  using Scalar = typename Base::Scalar;

  static constexpr int ParamSize = Base::ParamSize;
  using Params = ::Eigen::
      Map<const ::Eigen::Matrix<S, ParamSize, 1>, Options, StrideType>;
};

template <
    typename S,
    int Options,
    typename StrideType,
    template <typename> class... ComponentsT>
struct traits<
    ::Eigen::
        Map<::dart::math::GroupProduct<S, ComponentsT...>, Options, StrideType>>
  : traits<::dart::math::GroupProduct<S, ComponentsT...>>
{
  using Base = traits<::dart::math::GroupProduct<S, ComponentsT...>>;
  using Scalar = typename Base::Scalar;

  static constexpr int ParamSize = Base::ParamSize;
  using Params
      = ::Eigen::Map<::Eigen::Matrix<S, ParamSize, 1>, Options, StrideType>;
};

} // namespace Eigen::internal

namespace Eigen {

template <
    typename S,
    int Options,
    typename StrideType,
    template <typename> class... ComponentsT>
class Map<
    const ::dart::math::GroupProduct<S, ComponentsT...>,
    Options,
    StrideType>
  : public ::dart::math::GroupProductBase<
        Map<const ::dart::math::GroupProduct<S, ComponentsT...>,
            Options,
            StrideType>>
{
public:
  using Base = ::dart::math::GroupProductBase<
      Map<const ::dart::math::GroupProduct<S, ComponentsT...>,
          Options,
          StrideType>>;
  using Scalar = typename Base::Scalar;
  using Params = typename Base::Params;

  explicit Map(const Scalar* data);

  Map(const Scalar* data, const StrideType& stride);

  template <std::size_t Index>
  [[nodiscard]] auto params() const;

  [[nodiscard]] auto params(std::size_t index) const;

  [[nodiscard]] const Params& params() const;

private:
  Params m_params;
};

template <
    typename S,
    int Options,
    typename StrideType,
    template <typename> class... ComponentsT>
class Map<::dart::math::GroupProduct<S, ComponentsT...>, Options, StrideType>
  : public ::dart::math::GroupProductBase<
        Map<::dart::math::GroupProduct<S, ComponentsT...>, Options, StrideType>>
{
public:
  using Base = ::dart::math::GroupProductBase<
      Map<::dart::math::GroupProduct<S, ComponentsT...>, Options, StrideType>>;
  using Scalar = typename Base::Scalar;
  using Params = typename Base::Params;

  using Base::operator=;

  explicit Map(Scalar* data);

  Map(Scalar* data, const StrideType& stride);

  template <std::size_t Index>
  [[nodiscard]] auto params() const;

  template <std::size_t Index>
  [[nodiscard]] auto params();

  [[nodiscard]] auto params(std::size_t index) const;

  [[nodiscard]] auto params(std::size_t index);

  [[nodiscard]] const Params& params() const;

  [[nodiscard]] Params& params();

private:
  Params m_params;
};

} // namespace Eigen

//==============================================================================
// Implementation
//==============================================================================

namespace Eigen {

//==============================================================================
template <
    typename S,
    int Options,
    typename StrideType,
    template <typename> class... ComponentsT>
Map<const ::dart::math::GroupProduct<S, ComponentsT...>, Options, StrideType>::
    Map(const Scalar* data)
  : m_params(data)
{
  // Do nothing
}

//==============================================================================
template <
    typename S,
    int Options,
    typename StrideType,
    template <typename> class... ComponentsT>
Map<const ::dart::math::GroupProduct<S, ComponentsT...>, Options, StrideType>::
    Map(const Scalar* data, const StrideType& stride)
  : m_params(data, stride)
{
  // Do nothing
}

//==============================================================================
template <
    typename S,
    int Options,
    typename StrideType,
    template <typename> class... ComponentsT>
template <std::size_t Index>
auto Map<
    const ::dart::math::GroupProduct<S, ComponentsT...>,
    Options,
    StrideType>::params() const
{
  return m_params.template segment<Base::template Component<Index>::ParamSize>(
      std::get<Index>(Base::ParamSizeIndices));
}

//==============================================================================
template <
    typename S,
    int Options,
    typename StrideType,
    template <typename> class... ComponentsT>
auto Map<
    const ::dart::math::GroupProduct<S, ComponentsT...>,
    Options,
    StrideType>::params(std::size_t index) const
{
  DART_ASSERT(index < Base::ProductSize);
  return m_params.segment(
      Base::ParamSizeIndices[index], Base::ParamSizes[index]);
}

//==============================================================================
template <
    typename S,
    int Options,
    typename StrideType,
    template <typename> class... ComponentsT>
const typename Map<
    const ::dart::math::GroupProduct<S, ComponentsT...>,
    Options,
    StrideType>::Params&
Map<const ::dart::math::GroupProduct<S, ComponentsT...>, Options, StrideType>::
    params() const
{
  return m_params;
}

//==============================================================================
template <
    typename S,
    int Options,
    typename StrideType,
    template <typename> class... ComponentsT>
Map<::dart::math::GroupProduct<S, ComponentsT...>, Options, StrideType>::Map(
    Scalar* data)
  : m_params(data)
{
  // Do nothing
}

//==============================================================================
template <
    typename S,
    int Options,
    typename StrideType,
    template <typename> class... ComponentsT>
Map<::dart::math::GroupProduct<S, ComponentsT...>, Options, StrideType>::Map(
    Scalar* data, const StrideType& stride)
  : m_params(data, stride)
{
  // Do nothing
}

//==============================================================================
template <
    typename S,
    int Options,
    typename StrideType,
    template <typename> class... ComponentsT>
template <std::size_t Index>
auto Map<::dart::math::GroupProduct<S, ComponentsT...>, Options, StrideType>::
    params() const
{
  return m_params.template segment<Base::template Component<Index>::ParamSize>(
      std::get<Index>(Base::ParamSizeIndices));
}

//==============================================================================
template <
    typename S,
    int Options,
    typename StrideType,
    template <typename> class... ComponentsT>
template <std::size_t Index>
auto Map<::dart::math::GroupProduct<S, ComponentsT...>, Options, StrideType>::
    params()
{
  return m_params.template segment<Base::template Component<Index>::ParamSize>(
      std::get<Index>(Base::ParamSizeIndices));
}

//==============================================================================
template <
    typename S,
    int Options,
    typename StrideType,
    template <typename> class... ComponentsT>
auto Map<::dart::math::GroupProduct<S, ComponentsT...>, Options, StrideType>::
    params(std::size_t index) const
{
  DART_ASSERT(index < Base::ProductSize);
  return m_params.segment(
      Base::ParamSizeIndices[index], Base::ParamSizes[index]);
}

//==============================================================================
template <
    typename S,
    int Options,
    typename StrideType,
    template <typename> class... ComponentsT>
auto Map<::dart::math::GroupProduct<S, ComponentsT...>, Options, StrideType>::
    params(std::size_t index)
{
  DART_ASSERT(index < Base::ProductSize);
  return m_params.segment(
      Base::ParamSizeIndices[index], Base::ParamSizes[index]);
}

//==============================================================================
template <
    typename S,
    int Options,
    typename StrideType,
    template <typename> class... ComponentsT>
const typename Map<
    ::dart::math::GroupProduct<S, ComponentsT...>,
    Options,
    StrideType>::Params&
Map<::dart::math::GroupProduct<S, ComponentsT...>, Options, StrideType>::
    params() const
{
  return m_params;
}

//==============================================================================
template <
    typename S,
    int Options,
    typename StrideType,
    template <typename> class... ComponentsT>
typename Map<
    ::dart::math::GroupProduct<S, ComponentsT...>,
    Options,
    StrideType>::Params&
Map<::dart::math::GroupProduct<S, ComponentsT...>, Options, StrideType>::
    params()
{
  return m_params;
}

} // namespace Eigen
