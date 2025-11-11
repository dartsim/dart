/*
 * Copyright (c) 2011-2025, The DART development contributors
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

#include "dart/utils/SceneParser.hpp"

#include <gtest/gtest.h>

using namespace dart;
using namespace utils;

//==============================================================================
TEST(SceneParser, SkelAutoDetection)
{
  const auto contents
      = SceneParser::readFile("dart://sample/skel/test/single_pendulum.skel");

  ASSERT_EQ(contents.worlds.size(), 1u);
  ASSERT_EQ(contents.skeletons.size(), 1u);
  EXPECT_EQ(contents.skeletons.front()->getName(), "single_pendulum");
}

//==============================================================================
TEST(SceneParser, SdfAutoDetection)
{
  const auto contents = SceneParser::readFile("dart://sample/sdf/empty.world");

  ASSERT_EQ(contents.worlds.size(), 1u);
  EXPECT_TRUE(contents.skeletons.empty());
}

#if defined(DART_SCENEPARSER_HAS_URDF)
//==============================================================================
TEST(SceneParser, UrdfAutoDetection)
{
  const auto contents
      = SceneParser::readFile("dart://sample/urdf/KR5/ground.urdf");

  EXPECT_TRUE(contents.worlds.empty());
  ASSERT_EQ(contents.skeletons.size(), 1u);
  EXPECT_EQ(contents.skeletons.front()->getName(), "ground");
}
#endif

//==============================================================================
TEST(SceneParser, VskExplicitFormat)
{
  SceneParser::Options options;
  options.mFormatHint = SceneParser::Format::Vsk;

  const auto contents
      = SceneParser::readFile("dart://sample/vsk/Nick01.vsk", options);

  EXPECT_TRUE(contents.worlds.empty());
  ASSERT_EQ(contents.skeletons.size(), 1u);
  EXPECT_EQ(contents.skeletons.front()->getName(), "Nick01");
}
