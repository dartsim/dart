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

#include "dart/utils/scene/SceneParser.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <random>

using namespace dart;
using namespace utils;

namespace {

struct TempFile
{
  std::filesystem::path path;

  ~TempFile()
  {
    if (path.empty())
      return;

    std::error_code ec;
    std::filesystem::remove(path, ec);
  }
};

TempFile makeTempFile(const std::string& contents, const std::string& extension)
{
  TempFile file;
  const auto tempDir = std::filesystem::temp_directory_path();

  auto buildCandidate = [&]() {
    static std::mt19937 rng(std::random_device{}());
    static const char hex[] = "0123456789abcdef";
    std::uniform_int_distribution<int> dist(0, 15);

    std::string suffix(8, '0');
    for (char& ch : suffix)
      ch = hex[dist(rng)];

    return tempDir / ("dart_scene_parser_" + suffix + extension);
  };

  for (int attempt = 0; attempt < 32; ++attempt) {
    auto candidate = buildCandidate();
    if (!std::filesystem::exists(candidate)) {
      file.path = candidate;
      break;
    }
  }

  if (file.path.empty())
    file.path = buildCandidate();

  std::ofstream stream(file.path);
  stream << contents;
  stream.close();

  return file;
}

} // namespace

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
  EXPECT_FALSE(contents.skeletons.front()->getName().empty());
}

//==============================================================================
TEST(SceneParser, AutoDetectVsk)
{
  const auto contents = SceneParser::readFile("dart://sample/vsk/Nick01.vsk");

  EXPECT_TRUE(contents.worlds.empty());
  ASSERT_EQ(contents.skeletons.size(), 1u);
}

//==============================================================================
TEST(SceneParser, SdfSkeletonFallback)
{
  SceneParser::Options options;
  options.mFormatHint = SceneParser::Format::Sdf;

  const auto contents = SceneParser::readFile(
      "dart://sample/sdf/atlas/atlas_v3_no_head.sdf", options);

  EXPECT_TRUE(contents.worlds.empty());
  ASSERT_FALSE(contents.skeletons.empty());
}

#if defined(DART_SCENEPARSER_HAS_URDF)
//==============================================================================
TEST(SceneParser, UrdfWorld)
{
  SceneParser::Options options;
  options.mFormatHint = SceneParser::Format::Urdf;

  const auto contents = SceneParser::readFile(
      "dart://sample/urdf/test/testWorld.urdf", options);

  ASSERT_EQ(contents.worlds.size(), 1u);
  ASSERT_GE(contents.skeletons.size(), 1u);
}
#endif

//==============================================================================
TEST(SceneParser, MjcfExplicitHint)
{
  SceneParser::Options options;
  options.mFormatHint = SceneParser::Format::Mjcf;

  const auto contents
      = SceneParser::readFile("dart://sample/mjcf/openai/ant.xml", options);

  ASSERT_EQ(contents.worlds.size(), 1u);
}

//==============================================================================
TEST(SceneParser, MjcfAutoFallback)
{
  const auto contents
      = SceneParser::readFile("dart://sample/mjcf/openai/ant.xml");

  ASSERT_EQ(contents.worlds.size(), 1u);
}

//==============================================================================
TEST(SceneParser, SdfWorldAlsoReturnsStandaloneModels)
{
  const std::string sdf = R"(
<?xml version="1.0" ?>
<sdf version="1.5">
  <world name="mixed">
    <gravity>0 0 -9.81</gravity>
    <model name="world_model">
      <static>true</static>
      <link name="world_link">
        <inertial>
          <mass>1</mass>
          <pose>0 0 0 0 0 0</pose>
          <inertia>
            <ixx>1</ixx>
            <iyy>1</iyy>
            <izz>1</izz>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyz>0</iyz>
          </inertia>
        </inertial>
        <visual name="world_visual">
          <geometry>
            <box>
              <size>0.1 0.1 0.1</size>
            </box>
          </geometry>
        </visual>
        <collision name="world_collision">
          <geometry>
            <box>
              <size>0.1 0.1 0.1</size>
            </box>
          </geometry>
        </collision>
      </link>
    </model>
  </world>
  <model name="standalone_model">
    <static>true</static>
    <link name="standalone_link">
      <inertial>
        <mass>1</mass>
        <pose>0 0 0 0 0 0</pose>
        <inertia>
          <ixx>1</ixx>
          <iyy>1</iyy>
          <izz>1</izz>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyz>0</iyz>
        </inertia>
      </inertial>
      <visual name="standalone_visual">
        <geometry>
          <box>
            <size>0.2 0.2 0.2</size>
          </box>
        </geometry>
      </visual>
      <collision name="standalone_collision">
        <geometry>
          <box>
            <size>0.2 0.2 0.2</size>
          </box>
        </geometry>
      </collision>
    </link>
  </model>
</sdf>
)";

  const auto file = makeTempFile(sdf, ".sdf");
  ASSERT_FALSE(file.path.empty());

  const auto contents = SceneParser::readFile(file.path.string());

  ASSERT_EQ(contents.worlds.size(), 1u);
  ASSERT_GE(contents.skeletons.size(), 2u);

  bool hasStandalone = false;
  for (const auto& skeleton : contents.skeletons) {
    if (skeleton && skeleton->getName() == "standalone_model")
      hasStandalone = true;
  }

  EXPECT_TRUE(hasStandalone);
}

//==============================================================================
TEST(SceneParser, DetectsUrdfByContent)
{
  const std::string urdf = R"(
<?xml version="1.0" ?>
<robot name="temp_robot">
  <link name="base">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="1.0"/>
      <inertia ixx="1" iyy="1" izz="1" ixy="0" ixz="0" iyz="0"/>
    </inertial>
    <visual>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
    </collision>
  </link>
</robot>
)";

  const auto file = makeTempFile(urdf, ".xml");
  ASSERT_FALSE(file.path.empty());

  const auto contents = SceneParser::readFile(file.path.string());

  EXPECT_TRUE(contents.worlds.empty());
  ASSERT_EQ(contents.skeletons.size(), 1u);
  EXPECT_EQ(contents.skeletons.front()->getName(), "temp_robot");
}

//==============================================================================
TEST(SceneParser, FormatHintFailureReturnsEmpty)
{
  SceneParser::Options options;
  options.mFormatHint = SceneParser::Format::Sdf;

  const auto contents
      = SceneParser::readFile("dart://sample/vsk/Nick01.vsk", options);

  EXPECT_TRUE(contents.empty());
}
