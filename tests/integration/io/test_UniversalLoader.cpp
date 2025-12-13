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

#include "dart/utils/All.hpp"

#include <gtest/gtest.h>

using namespace dart;
using namespace dart::utils;

//==============================================================================
TEST(UniversalLoader, AutoDetectsSkelWorld)
{
  const auto world = readWorld("dart://sample/skel/test/empty.skel");
  ASSERT_NE(world, nullptr);
  EXPECT_EQ(world->getNumSkeletons(), 0);
}

//==============================================================================
TEST(UniversalLoader, AutoDetectsSdfWorld)
{
  const auto world = readWorld("dart://sample/sdf/empty.world");
  ASSERT_NE(world, nullptr);
}

//==============================================================================
TEST(UniversalLoader, AutoDetectsMjcfWorldByXmlRoot)
{
  const auto world = readWorld("dart://sample/mjcf/openai/ant.xml");
  ASSERT_NE(world, nullptr);
  EXPECT_GT(world->getNumSkeletons(), 0);
}

//==============================================================================
TEST(UniversalLoader, ExplicitFormatOverridesAuto)
{
  ReadOptions options;
  options.format = ModelFormat::Skel;
  const auto skeleton
      = readSkeleton("dart://sample/skel/test/single_pendulum.skel", options);
  EXPECT_NE(skeleton, nullptr);
}

//==============================================================================
TEST(UniversalLoader, UrdfRequiresUtilsUrdfComponent)
{
  // This test intentionally links against dart-utils only (no dart-utils-urdf).
  const auto skeleton
      = readSkeleton("dart://sample/urdf/test/primitive_geometry.urdf");
  EXPECT_EQ(skeleton, nullptr);
}

