/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
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

#include <type_traits>
#include <utility>

#include <cstddef>

namespace dart::collision::native {

template <typename T>
class span
{
public:
  using element_type = T;
  using value_type = typename std::remove_cv<T>::type;
  using size_type = std::size_t;
  using pointer = T*;
  using reference = T&;
  using iterator = pointer;

  constexpr span() noexcept = default;

  constexpr span(pointer data, size_type size) noexcept
    : mData(data), mSize(size)
  {
  }

  template <
      typename U,
      typename
      = typename std::enable_if<std::is_convertible<U*, pointer>::value>::type>
  constexpr span(const span<U>& other) noexcept
    : mData(other.data()), mSize(other.size())
  {
  }

  template <
      typename Container,
      typename = typename std::enable_if<std::is_convertible<
          decltype(std::declval<Container&>().data()),
          pointer>::value>::type>
  constexpr span(Container& container) noexcept
    : mData(container.data()), mSize(container.size())
  {
  }

  template <
      typename Container,
      typename = typename std::enable_if<std::is_convertible<
          decltype(std::declval<const Container&>().data()),
          pointer>::value>::type,
      typename = void>
  constexpr span(const Container& container) noexcept
    : mData(container.data()), mSize(container.size())
  {
  }

  constexpr iterator begin() const noexcept
  {
    return mData;
  }

  constexpr iterator end() const noexcept
  {
    return mData + mSize;
  }

  constexpr size_type size() const noexcept
  {
    return mSize;
  }

  constexpr bool empty() const noexcept
  {
    return mSize == 0u;
  }

  constexpr reference operator[](size_type index) const noexcept
  {
    return mData[index];
  }

  constexpr pointer data() const noexcept
  {
    return mData;
  }

private:
  pointer mData{nullptr};
  size_type mSize{0u};
};

} // namespace dart::collision::native
