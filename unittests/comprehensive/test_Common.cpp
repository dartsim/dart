/*
 * Copyright (c) 2011-2019, The DART development contributors
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

#include <gtest/gtest.h>

#include "dart/common/Timer.hpp"

using namespace dart::common;

//==============================================================================
TEST(Common, Timer)
{
  Timer timer1("Timer1");
  Timer timer2("Timer2");

  // Run for 2 seconds
  timer1.start();
  for (int i = 0; i < 1e+3; ++i)
  {
    timer2.start();
#ifdef _WIN32
	Sleep(2);  // 2 milliseconds
#else
	usleep(2000);  // 2 milliseconds
#endif
    timer2.stop();
  }
  timer1.stop();

  timer1.print();
  timer2.print();

  // Both timer should have counted more than 2 seconds
#ifdef _WIN32
  // On Windows, Sleep(2) takes less than exact 2 milliseconds..
  EXPECT_GE(timer1.getTotalElapsedTime(), 1.99);
  EXPECT_GE(timer2.getTotalElapsedTime(), 1.99);
#else
  EXPECT_GE(timer1.getTotalElapsedTime(), 2.0);
  EXPECT_GE(timer2.getTotalElapsedTime(), 2.0);
#endif
}
