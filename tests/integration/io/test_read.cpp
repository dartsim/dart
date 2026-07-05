/*
 * Copyright (c) 2011, The DART development contributors
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

#include "dart/io/All.hpp"

#include <gtest/gtest.h>

#include <array>
#include <filesystem>
#include <fstream>
#include <string_view>

using namespace dart;

//==============================================================================
TEST(Read, AutoDetectsConvertedSinglePendulumSdfSkeleton)
{
  const auto skeleton
      = io::readSkeleton("dart://sample/sdf/test/single_pendulum.sdf");
#if DART_HAS_SDFORMAT
  EXPECT_NE(skeleton, nullptr);
#else
  EXPECT_EQ(skeleton, nullptr);
#endif
}

//==============================================================================
TEST(Read, AutoDetectsSdfSkeleton)
{
  const auto skeleton
      = io::readSkeleton("dart://sample/sdf/test/two_link_revolute_model.sdf");
#if DART_HAS_SDFORMAT
  ASSERT_NE(skeleton, nullptr);
#else
  EXPECT_EQ(skeleton, nullptr);
#endif
}

//==============================================================================
TEST(Read, ConvertedSkelFixturesLoadAsSdf)
{
#if DART_HAS_SDFORMAT
  struct Fixture
  {
    std::string_view uri;
    std::string_view expectedName;
  };

  constexpr std::array fixtures = {
      Fixture{"dart://sample/sdf/test/single_pendulum.sdf", "single_pendulum"},
      Fixture{"dart://sample/sdf/test/cube.sdf", "box skeleton"},
      Fixture{"dart://sample/sdf/test/shapes.sdf", "box skeleton"},
      Fixture{"dart://sample/sdf/test/test_shapes.sdf", "ground skeleton"},
  };

  for (const auto& fixture : fixtures) {
    const auto skeleton = io::readSkeleton(fixture.uri);
    ASSERT_NE(skeleton, nullptr) << fixture.uri;
    EXPECT_EQ(skeleton->getName(), fixture.expectedName) << fixture.uri;
  }
#endif
}

//==============================================================================
TEST(Read, SkelIsNotSupported)
{
  const auto path = std::filesystem::temp_directory_path()
                    / "dart_io_removed_skel_fixture.skel";
  {
    std::ofstream output(path);
    output << "<skel version=\"1.0\"><world name=\"legacy\" /></skel>";
  }

  const auto skeleton = io::readSkeleton(path.string());
  EXPECT_EQ(skeleton, nullptr);

  std::filesystem::remove(path);
}

//==============================================================================
TEST(Read, MjcfDoesNotExposeDirectSkeletonLoading)
{
  const auto skeleton = io::readSkeleton("dart://sample/mjcf/openai/ant.xml");
  EXPECT_EQ(skeleton, nullptr);
}

//==============================================================================
TEST(Read, ExplicitFormatOverridesAuto)
{
  io::ReadOptions options;
  options.format = io::ModelFormat::Sdf;
  const auto skeleton
      = io::readSkeleton("dart://sample/sdf/test/single_pendulum.sdf", options);
#if DART_HAS_SDFORMAT
  EXPECT_NE(skeleton, nullptr);
#else
  EXPECT_EQ(skeleton, nullptr);
#endif
}

//==============================================================================
TEST(Read, HandlesUrdfSkeletonSupport)
{
  const auto skeleton
      = io::readSkeleton("dart://sample/urdf/test/primitive_geometry.urdf");
#if DART_IO_HAS_URDF
  EXPECT_NE(skeleton, nullptr);
#else
  EXPECT_EQ(skeleton, nullptr);
#endif
}
