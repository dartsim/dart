/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
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

#include <dart/utils/C3D.hpp>

#include <gtest/gtest.h>

#include <vector>

using namespace dart::utils;

TEST(C3D, SaveRejectsFrameCountLargerThanData)
{
  std::vector<std::vector<Eigen::Vector3d>> data{
      {Eigen::Vector3d(1.0, 2.0, 3.0)}};

  EXPECT_FALSE(saveC3DFile("invalid_frame_count.c3d", data, 10000, 1, 120.0));
}

TEST(C3D, SaveRejectsMarkerCountLargerThanData)
{
  std::vector<std::vector<Eigen::Vector3d>> data{
      {Eigen::Vector3d(1.0, 2.0, 3.0)}};

  EXPECT_FALSE(saveC3DFile("invalid_marker_count.c3d", data, 1, 2, 120.0));
}

TEST(C3D, SaveRejectsNegativeDimensions)
{
  std::vector<std::vector<Eigen::Vector3d>> data{
      {Eigen::Vector3d(1.0, 2.0, 3.0)}};

  EXPECT_FALSE(saveC3DFile("negative_frame_count.c3d", data, -1, 1, 120.0));
  EXPECT_FALSE(saveC3DFile("negative_marker_count.c3d", data, 1, -1, 120.0));
}
