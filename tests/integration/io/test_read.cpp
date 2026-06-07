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

using namespace dart;

//==============================================================================
TEST(Read, AutoDetectsSkelSkeleton)
{
  const auto skeleton
      = io::readSkeleton("dart://sample/skel/test/single_pendulum.skel");
  EXPECT_NE(skeleton, nullptr);
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
TEST(Read, MjcfDoesNotExposeDirectSkeletonLoading)
{
  const auto skeleton = io::readSkeleton("dart://sample/mjcf/openai/ant.xml");
  EXPECT_EQ(skeleton, nullptr);
}

//==============================================================================
TEST(Read, ExplicitFormatOverridesAuto)
{
  io::ReadOptions options;
  options.format = io::ModelFormat::Skel;
  const auto skeleton = io::readSkeleton(
      "dart://sample/skel/test/single_pendulum.skel", options);
  EXPECT_NE(skeleton, nullptr);
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
