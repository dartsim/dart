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

#include "dart/common/gpu/GpuKernel.hpp"

#include <gtest/gtest.h>

using namespace dart;
using namespace common;

//==============================================================================
GTEST_TEST(GpuDeviceTest, GpuDevice)
{
#if DART_ENABLED_GPU
  auto gpu_device = std::make_shared<GpuDevice>();
  if (gpu_device->isValid()) {
    auto kernel = gpu_device->createKernel(
        R"CLC(
        __kernel void add(__global int* data, int val) {
            int i = get_global_id(0);
            data[i] += val;
        }
    )CLC");
    EXPECT_TRUE(kernel.has("add"));
    EXPECT_FALSE(kernel.has("__does_not_exist__"));

    {
      int data[] = {1, 2, 3, 4, 5};
      ASSERT_TRUE(kernel.run("add", data, 2));
      EXPECT_EQ(data[0], 3);
      EXPECT_EQ(data[1], 4);
      EXPECT_EQ(data[2], 5);
      EXPECT_EQ(data[3], 6);
      EXPECT_EQ(data[4], 7);
    }

    {
      std::array<int, 5> data{1, 2, 3, 4, 5};
      ASSERT_TRUE(kernel.run("add", data, 1));
      EXPECT_EQ(data[0], 2);
      EXPECT_EQ(data[1], 3);
      EXPECT_EQ(data[2], 4);
      EXPECT_EQ(data[3], 5);
      EXPECT_EQ(data[4], 6);
      ASSERT_TRUE(kernel.run("add", data.begin(), data.end(), 1));
      EXPECT_EQ(data[0], 3);
      EXPECT_EQ(data[1], 4);
      EXPECT_EQ(data[2], 5);
      EXPECT_EQ(data[3], 6);
      EXPECT_EQ(data[4], 7);
    }

    {
      const std::array<int, 5> data{1, 2, 3, 4, 5};
      ASSERT_TRUE(kernel.run("add", data, 1));
      EXPECT_EQ(data[0], 2);
      EXPECT_EQ(data[1], 3);
      EXPECT_EQ(data[2], 4);
      EXPECT_EQ(data[3], 5);
      EXPECT_EQ(data[4], 6);
      ASSERT_TRUE(kernel.run("add", data.begin(), data.end(), 1));
      EXPECT_EQ(data[0], 3);
      EXPECT_EQ(data[1], 4);
      EXPECT_EQ(data[2], 5);
      EXPECT_EQ(data[3], 6);
      EXPECT_EQ(data[4], 7);
    }

    {
      std::vector<int> data{1, 2, 3, 4, 5};
      ASSERT_TRUE(kernel.run("add", data, 1));
      EXPECT_EQ(data[0], 2);
      EXPECT_EQ(data[1], 3);
      EXPECT_EQ(data[2], 4);
      EXPECT_EQ(data[3], 5);
      EXPECT_EQ(data[4], 6);
      ASSERT_TRUE(kernel.run("add", data.begin(), data.end(), 1));
      EXPECT_EQ(data[0], 3);
      EXPECT_EQ(data[1], 4);
      EXPECT_EQ(data[2], 5);
      EXPECT_EQ(data[3], 6);
      EXPECT_EQ(data[4], 7);
    }

    {
      const std::vector<int> data{1, 2, 3, 4, 5};
      ASSERT_TRUE(kernel.run("add", data, 1));
      EXPECT_EQ(data[0], 2);
      EXPECT_EQ(data[1], 3);
      EXPECT_EQ(data[2], 4);
      EXPECT_EQ(data[3], 5);
      EXPECT_EQ(data[4], 6);
      ASSERT_TRUE(kernel.run("add", data.begin(), data.end(), 1));
      EXPECT_EQ(data[0], 3);
      EXPECT_EQ(data[1], 4);
      EXPECT_EQ(data[2], 5);
      EXPECT_EQ(data[3], 6);
      EXPECT_EQ(data[4], 7);
    }
  }
#endif
}
