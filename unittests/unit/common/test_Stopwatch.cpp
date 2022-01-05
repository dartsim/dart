/*
 * Copyright (c) 2011-2022, The DART development contributors
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

#include <dart/common/Stopwatch.hpp>
#include <gtest/gtest.h>

#include "TestHelpers.hpp"

using namespace dart;
using namespace common;

//==============================================================================
TEST(StopwatchTest, Basics)
{
  auto sw = StopwatchNS();

  // Stopwatch is started by default
  EXPECT_TRUE(sw.isStarted());
  EXPECT_TRUE(StopwatchNS(true).isStarted());
  EXPECT_FALSE(StopwatchNS(false).isStarted());

  // Stop the stopwatch
  sw.stop();
  EXPECT_FALSE(sw.isStarted());

  // Elapsed time should be the same
  auto elapsed1 = sw.elapsedS();
  EXPECT_DOUBLE_EQ(elapsed1, sw.elapsedS());
  EXPECT_DOUBLE_EQ(elapsed1, sw.elapsedS());

  // Elapsed time monotonically increase while the stopwatch is running
  sw.start();
  EXPECT_GE(sw.elapsedS(), elapsed1);
  EXPECT_GE(sw.elapsedS(), elapsed1);

  // Starting a stopwatch already started doesn't have any effect
  sw.start();
  EXPECT_TRUE(sw.isStarted());

  // Restting a started stopwatch resets the elapsed time but doesn't stop the
  // stopwatch
  sw.start();
  sw.reset();
  EXPECT_TRUE(sw.isStarted());
  EXPECT_GE(sw.elapsedS(), 0.0);
  EXPECT_GE(sw.elapsedS(), 0.0);

  // Restting a stopped stopwatch resets the elapsed time but doesn't start the
  // stopwatch
  sw.stop();
  sw.reset();
  EXPECT_FALSE(sw.isStarted());
  EXPECT_DOUBLE_EQ(sw.elapsedS(), 0.0);

  std::cout << sw;
}
