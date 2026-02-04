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

#include <dart/simd/eigen/interop.hpp>

#include <Eigen/Core>

#include <iterator>
#include <span>
#include <vector>

namespace dart::simd {

template <typename T, std::size_t N>
[[nodiscard]] inline EigenSoA3<T, N> loadSoa3(
    const Eigen::Vector3<T>* ptr) noexcept
{
  EigenSoA3<T, N> result;
  alignas(64) T xs[N];
  alignas(64) T ys[N];
  alignas(64) T zs[N];
  for (std::size_t i = 0; i < N; ++i) {
    xs[i] = ptr[i][0];
    ys[i] = ptr[i][1];
    zs[i] = ptr[i][2];
  }
  result.x = Vec<T, N>::load(xs);
  result.y = Vec<T, N>::load(ys);
  result.z = Vec<T, N>::load(zs);
  return result;
}

template <typename T, std::size_t N>
class SimdChunksIterator
{
public:
  using iterator_category = std::forward_iterator_tag;
  using value_type = EigenSoA3<T, N>;
  using difference_type = std::ptrdiff_t;
  using pointer = void;
  using reference = value_type;

private:
  const Eigen::Vector3<T>* ptr_;
  std::size_t remaining_;

public:
  SimdChunksIterator() noexcept : ptr_(nullptr), remaining_(0) {}

  SimdChunksIterator(const Eigen::Vector3<T>* ptr, std::size_t count) noexcept
    : ptr_(ptr), remaining_(count)
  {
  }

  [[nodiscard]] value_type operator*() const noexcept
  {
    return loadSoa3<T, N>(ptr_);
  }

  SimdChunksIterator& operator++() noexcept
  {
    ptr_ += N;
    remaining_ = (remaining_ >= N) ? (remaining_ - N) : 0;
    return *this;
  }

  SimdChunksIterator operator++(int) noexcept
  {
    auto copy = *this;
    ++(*this);
    return copy;
  }

  [[nodiscard]] bool operator==(const SimdChunksIterator& other) const noexcept
  {
    return ptr_ == other.ptr_;
  }

  [[nodiscard]] bool operator!=(const SimdChunksIterator& other) const noexcept
  {
    return ptr_ != other.ptr_;
  }

  [[nodiscard]] std::size_t remaining() const noexcept
  {
    return remaining_;
  }
};

template <typename T, std::size_t N>
class SimdChunksView
{
private:
  const Eigen::Vector3<T>* data_;
  std::size_t count_;
  std::size_t full_chunks_;

public:
  SimdChunksView(std::span<const Eigen::Vector3<T>> data) noexcept
    : data_(data.data()), count_(data.size()), full_chunks_(count_ / N)
  {
  }

  [[nodiscard]] SimdChunksIterator<T, N> begin() const noexcept
  {
    return SimdChunksIterator<T, N>(data_, count_);
  }

  [[nodiscard]] SimdChunksIterator<T, N> end() const noexcept
  {
    return SimdChunksIterator<T, N>(data_ + full_chunks_ * N, 0);
  }

  [[nodiscard]] std::size_t size() const noexcept
  {
    return full_chunks_;
  }

  [[nodiscard]] std::size_t remainder() const noexcept
  {
    return count_ % N;
  }

  [[nodiscard]] std::span<const Eigen::Vector3<T>> remainderSpan()
      const noexcept
  {
    std::size_t rem = remainder();
    if (rem == 0) {
      return {};
    }
    return std::span<const Eigen::Vector3<T>>(
        data_ + full_chunks_ * N, remainder());
  }
};

template <std::size_t N, typename T>
[[nodiscard]] SimdChunksView<T, N> simdChunks(
    std::span<const Eigen::Vector3<T>> data) noexcept
{
  return SimdChunksView<T, N>(data);
}

template <std::size_t N, typename T>
[[nodiscard]] SimdChunksView<T, N> simdChunks(
    const std::vector<Eigen::Vector3<T>>& data) noexcept
{
  return SimdChunksView<T, N>(
      std::span<const Eigen::Vector3<T>>(data.data(), data.size()));
}

template <std::size_t N, typename T, std::size_t ArrayN>
[[nodiscard]] SimdChunksView<T, N> simdChunks(
    const std::array<Eigen::Vector3<T>, ArrayN>& data) noexcept
{
  return SimdChunksView<T, N>(
      std::span<const Eigen::Vector3<T>>(data.data(), ArrayN));
}

} // namespace dart::simd
