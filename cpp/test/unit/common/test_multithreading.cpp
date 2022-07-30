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

#include <gtest/gtest.h>

#include "dart/common/multithreading.hpp"

using namespace dart;
using namespace common;

//==============================================================================
TEST(ParallelTest, Basics)
{
  set_max_thread_count(1);
  EXPECT_EQ(get_max_thread_count(), 1);

  set_max_thread_count(get_available_core_count());
  EXPECT_EQ(get_max_thread_count(), get_available_core_count());
}

//==============================================================================
TEST(ParallelTest, ParallelFill)
{
  size_t n = std::max<size_t>(1e+7, (3 * get_available_core_count()) / 2);
  const double val = 3.0;
  std::cout << "n: " << n << std::endl;

  {
    std::vector<double> a(n);
    auto start = std::chrono::steady_clock::now();
    parallel_fill(a.begin(), a.end(), val, ExecutionPolicy::Parallel);
    auto end = std::chrono::steady_clock::now();
    std::cout << "Parallel : "
              << std::chrono::duration_cast<std::chrono::microseconds>(
                     end - start)
                     .count()
              << " us" << std::endl;
    for (auto val : a) {
      EXPECT_DOUBLE_EQ(val, val);
    }
  }

  {
    std::vector<double> a(n);
    auto start = std::chrono::steady_clock::now();
    parallel_fill(a.begin(), a.end(), val, ExecutionPolicy::Serial);
    auto end = std::chrono::steady_clock::now();
    std::cout << "Serial   : "
              << std::chrono::duration_cast<std::chrono::microseconds>(
                     end - start)
                     .count()
              << " us" << std::endl;
    for (auto val : a) {
      EXPECT_DOUBLE_EQ(val, val);
    }
  }

  {
    std::vector<double> a(n);
    auto start = std::chrono::steady_clock::now();
    std::fill(a.begin(), a.end(), val);
    auto end = std::chrono::steady_clock::now();
    std::cout << "std::fill: "
              << std::chrono::duration_cast<std::chrono::microseconds>(
                     end - start)
                     .count()
              << " us" << std::endl;
    for (auto val : a) {
      EXPECT_DOUBLE_EQ(val, val);
    }
  }
}

//==============================================================================
TEST(ParallelTest, ParallelFor)
{
  size_t n = std::max<size_t>(1e+7, (3 * get_available_core_count()) / 2);
  const double val = 3.0;
  std::cout << "n: " << n << std::endl;

  {
    std::vector<double> a(n);
    auto start = std::chrono::steady_clock::now();
    auto it_begin = a.begin();
    auto it_end = a.end();
    parallel_for(
        size_t(0),
        size_t(it_end - it_begin),
        [it_begin, val](size_t i) {
          it_begin[i] = val;
        },
        ExecutionPolicy::Parallel);
    auto end = std::chrono::steady_clock::now();
    std::cout << "Parallel : "
              << std::chrono::duration_cast<std::chrono::microseconds>(
                     end - start)
                     .count()
              << " us" << std::endl;
    for (auto val : a) {
      EXPECT_DOUBLE_EQ(val, val);
    }
  }

  {
    std::vector<double> a(n);
    auto start = std::chrono::steady_clock::now();
    auto it_begin = a.begin();
    auto it_end = a.end();
    parallel_for(
        size_t(0),
        size_t(it_end - it_begin),
        [it_begin, val](size_t i) {
          it_begin[i] = val;
        },
        ExecutionPolicy::Serial);
    auto end = std::chrono::steady_clock::now();
    std::cout << "Serial   : "
              << std::chrono::duration_cast<std::chrono::microseconds>(
                     end - start)
                     .count()
              << " us" << std::endl;
    for (auto val : a) {
      EXPECT_DOUBLE_EQ(val, val);
    }
  }
}
