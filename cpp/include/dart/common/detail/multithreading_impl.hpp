/*
 * Copyright (c) 2011-2021, The DART development contributors:
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

#include <algorithm>
#include <cmath>
#include <functional>
#include <future>
#include <vector>
#if DART_ENABLE_TBB
  #include <execution>
#else
  #include <omp.h>
#endif

#include "dart/common/multithreading.hpp"

namespace dart::common {

//==============================================================================
template <typename RandomIterator, typename T>
void parallel_fill(
    const RandomIterator& begin,
    const RandomIterator& end,
    const T& value,
    ExecutionPolicy policy)
{
  switch (policy) {
    case ExecutionPolicy::Parallel: {
#if DART_ENABLE_TBB
      std::fill(std::execution::par_unseq, begin, end, value);
#else
      auto diff = end - begin;
      if (diff <= 0) {
        return;
      }
      const size_t size = static_cast<size_t>(diff);
      parallel_for(
          size_t(0),
          size,
          [begin, value](size_t i) {
            begin[i] = value;
          },
          policy);
#endif
      break;
    }
    default: {
      std::fill(begin, end, value);
      break;
    }
  }
}

//==============================================================================
template <typename IndexType, typename Function>
void parallel_for(
    IndexType begin, IndexType end, Function&& func, ExecutionPolicy policy)
{
  if (begin > end) {
    return;
  }

  switch (policy) {
    case ExecutionPolicy::Parallel: {
#if DART_ENABLE_TBB
      std::for_each(std::execution::par_unseq, begin, end, std::move(func));
#else
  #pragma omp parallel for
  #if defined(_MSC_VER) && !defined(__INTEL_COMPILER)
      for (ssize_t i = begin; i < ssize_t(end); ++i) {
  #else
      for (auto i = begin; i < end; ++i) {
  #endif
        func(i);
      }
#endif
      break;
    }
    default: {
      for (auto i = begin; i < end; ++i) {
        func(i);
      }
    }
  }
}

} // namespace dart::common
