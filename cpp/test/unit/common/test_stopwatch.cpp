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

#include "dart/common/stopwatch.hpp"

using namespace dart;
using namespace common;

//==============================================================================
TEST(StopwatchTest, Basics)
{
  auto sw = StopwatchNS();

  // Stopwatch is started by default
  EXPECT_TRUE(sw.is_started());
  EXPECT_TRUE(common::StopwatchNS(true).is_started());
  EXPECT_FALSE(common::StopwatchNS(false).is_started());

  // Stop the stopwatch
  sw.stop();
  EXPECT_FALSE(sw.is_started());

  // Elapsed time should be the same
  auto elapsed1 = sw.elapsed_s();
  EXPECT_DOUBLE_EQ(elapsed1, sw.elapsed_s());
  EXPECT_DOUBLE_EQ(elapsed1, sw.elapsed_s());

  // Elapsed time monotonically increase while the stopwatch is running
  sw.start();
  EXPECT_GE(sw.elapsed_s(), elapsed1);
  EXPECT_GE(sw.elapsed_s(), elapsed1);

  // Starting a stopwatch already started doesn't have any effect
  sw.start();
  EXPECT_TRUE(sw.is_started());

  // Restting a started stopwatch resets the elapsed time but doesn't stop the
  // stopwatch
  sw.start();
  sw.reset();
  EXPECT_TRUE(sw.is_started());
  EXPECT_GE(sw.elapsed_s(), 0.0);
  EXPECT_GE(sw.elapsed_s(), 0.0);

  // Restting a stopped stopwatch resets the elapsed time but doesn't start the
  // stopwatch
  sw.stop();
  sw.reset();
  EXPECT_FALSE(sw.is_started());
  EXPECT_DOUBLE_EQ(sw.elapsed_s(), 0.0);

  std::cout << sw;
}
