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

#include <iostream>
#include <gtest/gtest.h>
#include "dart/utils/urdf/DartLoader.h"

using kido::common::Uri;
using kido::utils::DartLoader;

TEST(DartLoader, parseSkeleton_NonExistantPathReturnsNull)
{
  DartLoader loader;
  EXPECT_EQ(nullptr,
    loader.parseSkeleton(KIDO_DATA_PATH"skel/test/does_not_exist.urdf"));
}

TEST(DartLoader, parseSkeleton_InvalidUrdfReturnsNull)
{
  DartLoader loader;
  EXPECT_EQ(nullptr,
    loader.parseSkeleton(KIDO_DATA_PATH"urdf/test/invalid.urdf)"));
}

TEST(DartLoader, parseSkeleton_MissingMeshReturnsNull)
{
  DartLoader loader;
  EXPECT_EQ(nullptr,
    loader.parseSkeleton(KIDO_DATA_PATH"urdf/test/missing_mesh.urdf"));
}

TEST(DartLoader, parseSkeleton_InvalidMeshReturnsNull)
{
  DartLoader loader;
  EXPECT_EQ(nullptr,
    loader.parseSkeleton(KIDO_DATA_PATH"urdf/test/invalid_mesh.urdf"));
}

TEST(DartLoader, parseSkeleton_MissingPackageReturnsNull)
{
  DartLoader loader;
  EXPECT_EQ(nullptr,
    loader.parseSkeleton(KIDO_DATA_PATH"urdf/test/missing_package.urdf"));
}

TEST(DartLoader, parseSkeleton_LoadsPrimitiveGeometry)
{
  DartLoader loader;
  EXPECT_TRUE(nullptr !=
    loader.parseSkeleton(KIDO_DATA_PATH"urdf/test/primitive_geometry.urdf"));
}

TEST(DartLoader, parseWorld)
{
  DartLoader loader;
  EXPECT_TRUE(nullptr !=
      loader.parseWorld(KIDO_DATA_PATH"urdf/test/testWorld.urdf"));
}

int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
