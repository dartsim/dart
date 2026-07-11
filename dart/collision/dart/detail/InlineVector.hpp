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

#include "dart/common/Macros.hpp"

#include <algorithm>
#include <array>
#include <initializer_list>

#include <cstddef>

namespace dart::collision::native {

/// Fixed-capacity vector with inline storage.
///
/// The narrowphase hot paths run inside the steady-state stepping loop, where
/// heap allocations are forbidden by the StepAllocation gates; use this for
/// work sets whose size has a proven static bound. Exceeding \c Capacity is a
/// logic error: it asserts in debug builds and drops the element in release
/// builds.
template <typename T, std::size_t Capacity>
class InlineVector
{
public:
  InlineVector() = default;

  InlineVector(std::initializer_list<T> values)
  {
    DART_ASSERT(values.size() <= Capacity);
    for (const auto& value : values)
      push_back(value);
  }

  void push_back(const T& value)
  {
    DART_ASSERT(mSize < Capacity);
    if (mSize >= Capacity)
      return;

    mData[mSize++] = value;
  }

  void clear()
  {
    mSize = 0u;
  }

  [[nodiscard]] std::size_t size() const
  {
    return mSize;
  }

  [[nodiscard]] bool empty() const
  {
    return mSize == 0u;
  }

  [[nodiscard]] static constexpr std::size_t capacity()
  {
    return Capacity;
  }

  [[nodiscard]] T& operator[](std::size_t i)
  {
    DART_ASSERT(i < mSize);
    return mData[i];
  }

  [[nodiscard]] const T& operator[](std::size_t i) const
  {
    DART_ASSERT(i < mSize);
    return mData[i];
  }

  [[nodiscard]] T& back()
  {
    DART_ASSERT(mSize > 0u);
    return mData[mSize - 1u];
  }

  [[nodiscard]] const T& back() const
  {
    DART_ASSERT(mSize > 0u);
    return mData[mSize - 1u];
  }

  [[nodiscard]] T* begin()
  {
    return mData.data();
  }

  [[nodiscard]] const T* begin() const
  {
    return mData.data();
  }

  [[nodiscard]] T* end()
  {
    return mData.data() + mSize;
  }

  [[nodiscard]] const T* end() const
  {
    return mData.data() + mSize;
  }

  void erase(T* position)
  {
    DART_ASSERT(position >= begin() && position < end());
    std::move(position + 1, end(), position);
    --mSize;
  }

private:
  std::array<T, Capacity> mData;
  std::size_t mSize = 0u;
};

} // namespace dart::collision::native
