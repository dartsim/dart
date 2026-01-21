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

#include "dart/io/all.hpp"

#include <dart/config.hpp>

#include <dart/dynamics/free_joint.hpp>
#include <dart/dynamics/weld_joint.hpp>

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
