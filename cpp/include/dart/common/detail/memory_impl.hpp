/*
 * Copyright (c) 2011-2022, The DART development contributors:
 * https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include <memory>

#include "dart/common/eigen_include.hpp"
#include "dart/common/macro.hpp"
#include "dart/common/memory.hpp"

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
constexpr std::size_t get_padding(
    const std::size_t base_address, const std::size_t alignment)
{
  if (alignment == 0) {
    return 0;
  }

  //
  // 0       (alignment)  (2*alignment)          (multiplier*alignment)
  // +------------+-------------+-----...----+-------------+--------------
  //                                            ^          ^
  //                                            |          |
  //                                       base_address   aligned_address
  //                                            |--------->|
  //                                               padding
  //

  const std::size_t multiplier = (base_address / alignment) + 1;
  const std::size_t aligned_address = multiplier * alignment;
  DART_ASSERT(aligned_address >= base_address);
  const std::size_t padding = aligned_address - base_address;

  return padding;
}

//==============================================================================
template <class T>
bool is_aligned(const void* ptr) noexcept
{
  const auto iptr = reinterpret_cast<std::uintptr_t>(ptr);
  return !(iptr % alignof(T));
}

//==============================================================================
constexpr std::size_t operator"" _KiB(unsigned long long value) noexcept
{
  return std::size_t(value * 1024);
}

//==============================================================================
constexpr std::size_t operator"" _KB(unsigned long long value) noexcept
{
  return std::size_t(value * 1000);
}

//==============================================================================
constexpr std::size_t operator"" _MiB(unsigned long long value) noexcept
{
  return std::size_t(value * 1024 * 1024);
}

//==============================================================================
constexpr std::size_t operator"" _MB(unsigned long long value) noexcept
{
  return std::size_t(value * 1000 * 1000);
}

//==============================================================================
constexpr std::size_t operator"" _GiB(unsigned long long value) noexcept
{
  return std::size_t(value * 1024 * 1024 * 1024);
}

//==============================================================================
constexpr std::size_t operator"" _GB(unsigned long long value) noexcept
{
  return std::size_t(value * 1000 * 1000 * 1000);
}

} // namespace dart::common
