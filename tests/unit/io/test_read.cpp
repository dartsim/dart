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

#include <dart/config.hpp>

#include <dart/dynamics/free_joint.hpp>
#include <dart/dynamics/weld_joint.hpp>

#include <dart/common/local_resource_retriever.hpp>

#include <gtest/gtest.h>

using namespace dart;

//==============================================================================
TEST(ReadUnit, ReadsSkelSkeletonFromWorldFile)
{
  io::ReadOptions options;
  options.format = io::ModelFormat::Skel;
  const auto skeleton = io::readSkeleton(
      "dart://sample/skel/test/single_pendulum.skel", options);
  EXPECT_NE(skeleton, nullptr);
}

//==============================================================================
TEST(ReadUnit, ReturnsNullForMjcfSkeleton)
{
  const auto skeleton = io::readSkeleton("dart://sample/mjcf/openai/ant.xml");
  EXPECT_EQ(skeleton, nullptr);
}

#if DART_HAVE_SDFORMAT

namespace {

const char* kSingleBodySdfWorld
    = "dart://sample/sdf/test/single_bodynode_skeleton.world";

}

//==============================================================================
TEST(ReadUnit, ReadsSdfWorldWithFloatingRootByDefault)
{
  io::ReadOptions options;
  options.format = io::ModelFormat::Sdf;

  const auto world = io::readWorld(kSingleBodySdfWorld, options);
  ASSERT_NE(world, nullptr);
  ASSERT_EQ(world->getNumSkeletons(), 1u);

  const auto skeleton = world->getSkeleton(0);
  ASSERT_NE(skeleton, nullptr);
  ASSERT_EQ(skeleton->getNumJoints(), 1u);

  EXPECT_NE(dynamic_cast<dynamics::FreeJoint*>(skeleton->getJoint(0)), nullptr);
}

//==============================================================================
TEST(ReadUnit, ReadsSdfWorldWithFixedRootWhenRequested)
{
  io::ReadOptions options;
  options.format = io::ModelFormat::Sdf;
  options.sdfDefaultRootJointType = io::RootJointType::Fixed;

  const auto world = io::readWorld(kSingleBodySdfWorld, options);
  ASSERT_NE(world, nullptr);
  ASSERT_EQ(world->getNumSkeletons(), 1u);

  const auto skeleton = world->getSkeleton(0);
  ASSERT_NE(skeleton, nullptr);
  ASSERT_EQ(skeleton->getNumJoints(), 1u);

  EXPECT_NE(dynamic_cast<dynamics::WeldJoint*>(skeleton->getJoint(0)), nullptr);
}
#endif // DART_HAVE_SDFORMAT

#if DART_IO_HAS_URDF
//==============================================================================
TEST(ReadUnit, ReadsUrdfWithPackageDirectories)
{
  const common::Uri wamUri
      = common::Uri::createFromPath(config::dataPath("urdf/wam/wam.urdf"));
  const auto wamPackageDir = config::dataPath("urdf/wam");

  io::ReadOptions options;
  EXPECT_EQ(io::readSkeleton(wamUri, options), nullptr);

  options.addPackageDirectory("herb_description", "/does/not/exist");
  options.addPackageDirectory("herb_description", wamPackageDir);
  const auto skeleton = io::readSkeleton(wamUri, options);
  EXPECT_NE(skeleton, nullptr);
}
#endif

//==============================================================================
TEST(ReadUnit, ReturnsNullForNonExistentFile)
{
  const auto skeleton = io::readSkeleton("/nonexistent/path/robot.urdf");
  EXPECT_EQ(skeleton, nullptr);

  const auto world = io::readWorld("/nonexistent/path/world.sdf");
  EXPECT_EQ(world, nullptr);
}

//==============================================================================
TEST(ReadUnit, ReturnsNullForUnknownExtension)
{
  const auto skeleton = io::readSkeleton("/some/file.unknown");
  EXPECT_EQ(skeleton, nullptr);
}

//==============================================================================
TEST(ReadUnit, TryReadSkeletonReturnsErrorForInvalidPath)
{
  const auto result = io::tryReadSkeleton("/nonexistent/robot.urdf");
  EXPECT_FALSE(result.isOk());
  EXPECT_TRUE(result.isErr());
}

//==============================================================================
TEST(ReadUnit, TryReadWorldReturnsErrorForInvalidPath)
{
  const auto result = io::tryReadWorld("/nonexistent/world.sdf");
  EXPECT_FALSE(result.isOk());
  EXPECT_TRUE(result.isErr());
}

//==============================================================================
TEST(ReadUnit, TryReadSkeletonReturnsOkForValidFile)
{
  const auto result
      = io::tryReadSkeleton("dart://sample/skel/test/single_pendulum.skel");
  EXPECT_TRUE(result.isOk());
  EXPECT_FALSE(result.isErr());
  EXPECT_NE(result.value(), nullptr);
}

//==============================================================================
TEST(ReadUnit, TryReadWorldReturnsOkForValidFile)
{
  const auto result
      = io::tryReadWorld("dart://sample/skel/test/single_pendulum.skel");
  EXPECT_TRUE(result.isOk());
  EXPECT_FALSE(result.isErr());
  EXPECT_NE(result.value(), nullptr);
}

//==============================================================================
TEST(ReadUnit, ReadsMjcfWorldSuccessfully)
{
  const auto world = io::readWorld("dart://sample/mjcf/openai/ant.xml");
  EXPECT_NE(world, nullptr);
}

//==============================================================================
TEST(ReadUnit, ReadsSkelWorldSuccessfully)
{
  io::ReadOptions options;
  options.format = io::ModelFormat::Skel;
  const auto world
      = io::readWorld("dart://sample/skel/test/single_pendulum.skel", options);
  EXPECT_NE(world, nullptr);
}

//==============================================================================
TEST(ReadUnit, InfersFormatFromSkelExtension)
{
  const auto skeleton
      = io::readSkeleton("dart://sample/skel/test/single_pendulum.skel");
  EXPECT_NE(skeleton, nullptr);
}

//==============================================================================
TEST(ReadUnit, InfersFormatFromUrdfExtension)
{
#if DART_IO_HAS_URDF
  const auto skeleton
      = io::readSkeleton("dart://sample/urdf/KR5/KR5 sixx R650.urdf");
  EXPECT_NE(skeleton, nullptr);
#endif
}

//==============================================================================
TEST(ReadUnit, InfersFormatFromMjcfExtension)
{
  io::ReadOptions options;
  options.format = io::ModelFormat::Mjcf;
  const auto skeleton
      = io::readSkeleton("dart://sample/mjcf/openai/ant.xml", options);
  EXPECT_EQ(skeleton, nullptr);
}

//==============================================================================
TEST(ReadUnit, SkelWorldWithMultipleSkeletonsReturnsFirst)
{
  io::ReadOptions options;
  options.format = io::ModelFormat::Skel;
  const auto skeleton = io::readSkeleton(
      "dart://sample/skel/test/single_pendulum.skel", options);
  EXPECT_NE(skeleton, nullptr);
}

//==============================================================================
TEST(ReadUnit, InfersFormatFromMjcfFileExtension)
{
  const auto skeleton = io::readSkeleton("/nonexistent/model.mjcf");
  EXPECT_EQ(skeleton, nullptr);
}

//==============================================================================
TEST(ReadUnit, ReturnsNullForNoExtensionFile)
{
  const auto skeleton = io::readSkeleton("/some/path/file_without_ext");
  EXPECT_EQ(skeleton, nullptr);
}

//==============================================================================
TEST(ReadUnit, ReturnsNullForDotBeforeSlash)
{
  const auto skeleton = io::readSkeleton("/some/path.dir/no_ext");
  EXPECT_EQ(skeleton, nullptr);
}

//==============================================================================
TEST(ReadUnit, PassesExplicitRetrieverThrough)
{
  auto retriever = std::make_shared<common::LocalResourceRetriever>();
  io::ReadOptions options;
  options.resourceRetriever = retriever;
  options.format = io::ModelFormat::Skel;
  const auto skeleton = io::readSkeleton("/nonexistent/robot.skel", options);
  EXPECT_EQ(skeleton, nullptr);
}
