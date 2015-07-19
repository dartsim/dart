/*
 * Copyright (c) 2015, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Michael Koval <mkoval@cs.cmu.edu>
 *
 * Georgia Tech Graphics Lab and Humanoid Robotics Lab
 *
 * Directed by Prof. C. Karen Liu and Prof. Mike Stilman
 * <karenliu@cc.gatech.edu> <mstilman@cc.gatech.edu>
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
#include "dart/utils/RelativeResourceRetriever.h"
#include "TestHelpers.h"

using dart::utils::Resource;
using dart::utils::RelativeResourceRetriever;

TEST(RelativeResourceRetriever, exists_AbsolutePath_CallsDelegate)
{
  auto presentDelegate = std::make_shared<PresentResourceRetriever>();
  RelativeResourceRetriever presentRetriever("file:///prefix", presentDelegate);

  EXPECT_TRUE(presentRetriever.exists("file:///absolute/path"));
  EXPECT_TRUE(presentDelegate->mRetrieve.empty());
  ASSERT_EQ(1, presentDelegate->mExists.size());
  EXPECT_EQ("file:///absolute/path", presentDelegate->mExists.front());

  auto absentDelegate = std::make_shared<AbsentResourceRetriever>();
  RelativeResourceRetriever absentRetriever("file:///prefix", absentDelegate);

  EXPECT_FALSE(absentRetriever.exists("file:///absolute/path"));
  EXPECT_TRUE(absentDelegate->mRetrieve.empty());
  ASSERT_EQ(1, absentDelegate->mExists.size());
  EXPECT_EQ("file:///absolute/path", absentDelegate->mExists.front());
}

TEST(RelativeResourceRetriever, exists_RelativePath_Appends)
{
  auto presentDelegate = std::make_shared<PresentResourceRetriever>();
  RelativeResourceRetriever presentRetriever("file:///prefix", presentDelegate);

  EXPECT_TRUE(presentRetriever.exists("relative/path"));
  EXPECT_TRUE(presentDelegate->mRetrieve.empty());
  ASSERT_EQ(1, presentDelegate->mExists.size());
  EXPECT_EQ("file:///prefix/relative/path", presentDelegate->mExists.front());

  auto absentDelegate = std::make_shared<AbsentResourceRetriever>();
  RelativeResourceRetriever absentRetriever("file:///prefix", absentDelegate);

  EXPECT_FALSE(absentRetriever.exists("relative/path"));
  EXPECT_TRUE(absentDelegate->mRetrieve.empty());
  ASSERT_EQ(1, absentDelegate->mExists.size());
  EXPECT_EQ("file:///prefix/relative/path", absentDelegate->mExists.front());
}

TEST(RelativeResourceRetriever, retrieve_AbsolutePath_CallsDelegate)
{
  auto presentDelegate = std::make_shared<PresentResourceRetriever>();
  RelativeResourceRetriever presentRetriever("file:///prefix", presentDelegate);

  EXPECT_TRUE(nullptr != presentRetriever.retrieve("file:///absolute/path"));
  EXPECT_TRUE(presentDelegate->mExists.empty());
  ASSERT_EQ(1, presentDelegate->mRetrieve.size());
  EXPECT_EQ("file:///absolute/path", presentDelegate->mRetrieve.front());

  auto absentDelegate = std::make_shared<AbsentResourceRetriever>();
  RelativeResourceRetriever absentRetriever("file:///prefix", absentDelegate);

  EXPECT_EQ(nullptr, absentRetriever.retrieve("file:///absolute/path"));
  EXPECT_TRUE(absentDelegate->mExists.empty());
  ASSERT_EQ(1, absentDelegate->mRetrieve.size());
  EXPECT_EQ("file:///absolute/path", absentDelegate->mRetrieve.front());
}

TEST(RelativeResourceRetriever, retrieve_RelativePath_Appends)
{
  auto presentDelegate = std::make_shared<PresentResourceRetriever>();
  RelativeResourceRetriever presentRetriever("file:///prefix", presentDelegate);

  EXPECT_TRUE(nullptr != presentRetriever.retrieve("relative/path"));
  EXPECT_TRUE(presentDelegate->mExists.empty());
  ASSERT_EQ(1, presentDelegate->mRetrieve.size());
  EXPECT_EQ("file:///prefix/relative/path", presentDelegate->mRetrieve.front());

  auto absentDelegate = std::make_shared<AbsentResourceRetriever>();
  RelativeResourceRetriever absentRetriever("file:///prefix", absentDelegate);

  EXPECT_EQ(nullptr, absentRetriever.retrieve("relative/path"));
  EXPECT_TRUE(absentDelegate->mExists.empty());
  ASSERT_EQ(1, absentDelegate->mRetrieve.size());
  EXPECT_EQ("file:///prefix/relative/path", absentDelegate->mRetrieve.front());
}

int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
