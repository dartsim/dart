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

#include <chrono>
#include <future>

#include <gtest/gtest.h>

#include "dart/simulation/World.hpp"

#include "TestHelpers.hpp"

using namespace dart;
using namespace dynamics;

//==============================================================================
void createAndDestroyFrames(int threadNum)
{
  for(std::size_t i=0; i < 100; ++i)
  {
    EXPECT_EQ(Frame::World()->getNumChildEntities(), 0);
    EXPECT_EQ(Frame::World()->getNumChildFrames(), 0);

    SimpleFrame someFrame(Frame::World(),
                          "Frame_"+std::to_string(threadNum)+std::to_string(i));

    EXPECT_EQ(Frame::World()->getNumChildEntities(), 0);
    EXPECT_EQ(Frame::World()->getNumChildFrames(), 0);
  }
}

//==============================================================================
TEST(Concurrency, FrameDeletion)
{
  // Regression test for issue #576
  std::vector<std::future<void>> futures;
  for(std::size_t i=0; i < 10; ++i)
    futures.push_back(std::async(std::launch::async,
                                 &createAndDestroyFrames, i));

  for(std::size_t i=0; i < futures.size(); ++i)
  {
    EXPECT_EQ(Frame::World()->getNumChildEntities(), 0);
    EXPECT_EQ(Frame::World()->getNumChildFrames(), 0);
    futures[i].get();
  }

  EXPECT_EQ(Frame::World()->getNumChildEntities(), 0);
  EXPECT_EQ(Frame::World()->getNumChildFrames(), 0);
}
