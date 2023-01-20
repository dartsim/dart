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

#include <dart/common/Macros.hpp>

#include <Eigen/Core>

#include <memory>

namespace dart::common {

//==============================================================================
template <typename _Tp, typename... _Args>
std::shared_ptr<_Tp> make_aligned_shared(_Args&&... __args)
{
  using _Tp_nc = typename std::remove_const<_Tp>::type;

  return std::allocate_shared<_Tp>(
      Eigen::aligned_allocator<_Tp_nc>(), std::forward<_Args>(__args)...);
}

//==============================================================================
constexpr std::size_t GetPadding(
    const std::size_t base_address, const std::size_t alignment)
{
  //
  // 0       (alignment)  (2*alignment)          (multiplier*alignment)
  // +------------+-------------+-----...----+-------------+--------------
  //                                            ^          ^
  //                                            |          |
  //                                       base_address   aligned_address
  //                                            |--------->|
  //                                               padding
  //
  return (alignment == 0)
             ? 0
             : (alignment - (base_address % alignment)) % alignment;
}

//==============================================================================
template <size_t Alignment>
constexpr std::size_t GetPadding(const std::size_t base_address)
{
  //
  // 0       (alignment)  (2*alignment)          (multiplier*alignment)
  // +------------+-------------+-----...----+-------------+--------------
  //                                            ^          ^
  //                                            |          |
  //                                       base_address   aligned_address
  //                                            |--------->|
  //                                               padding
  //
  return (base_address % Alignment == 0)
             ? 0
             : (Alignment - (base_address % Alignment));
}

//==============================================================================
template <typename HeaderType>
constexpr std::size_t GetPaddingIncludingHeader(
    const std::size_t base_address, const std::size_t alignment)
{
  constexpr auto header_size = sizeof(HeaderType);
  return GetPadding(base_address + header_size, alignment) + header_size;
}

//==============================================================================
template <size_t Alignment, typename HeaderType>
constexpr std::size_t GetPaddingIncludingHeader(const std::size_t base_address)
{
  constexpr auto header_size = sizeof(HeaderType);
  return GetPadding<Alignment>(base_address + header_size) + header_size;
}

} // namespace dart::common
