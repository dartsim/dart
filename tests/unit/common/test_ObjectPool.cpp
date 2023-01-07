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

#include <dart/common/ObjectPool.hpp>

#include <gtest/gtest.h>

using namespace dart;
using namespace dart::common;

//==============================================================================
GTEST_TEST(ObjectPoolTest, Alignment)
{
  auto pool = AlignedObjectPool<double>();

  double* p1 = pool.create();
  double* p2 = pool.create();
  double* p3 = pool.create();
  double* p4 = pool.create();

  EXPECT_EQ(0u, reinterpret_cast<uintptr_t>(p1) % alignof(double));
  EXPECT_EQ(0u, reinterpret_cast<uintptr_t>(p2) % alignof(double));
  EXPECT_EQ(0u, reinterpret_cast<uintptr_t>(p3) % alignof(double));
  EXPECT_EQ(0u, reinterpret_cast<uintptr_t>(p4) % alignof(double));

  pool.destroy(p1);
  pool.destroy(p2);
  pool.destroy(p3);
  pool.destroy(p4);
}

//==============================================================================
GTEST_TEST(ObjectPoolTest, StressTest)
{
  auto pool = AlignedObjectPool<double>();

  const int kNumIterations = 1000000;
  std::vector<double*> allocated;

  for (int i = 0; i < kNumIterations; ++i) {
    double* p = pool.create();
    allocated.push_back(p);
  }

  for (double* p : allocated)
    pool.destroy(p);
}
