/*
 * Copyright (c) 2011-2019, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/master/LICENSE
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
#include <TestHelpers.hpp>
#include <dart/dart.hpp>
#include <dart/io/urdf/DartLoader.hpp>

//==============================================================================
TEST(Issue838, MaterialParsing)
{
  dart::io::DartLoader loader;
  dart::dynamics::SkeletonPtr skeleton =
      loader.parseSkeleton("dart://sample/urdf/test/issue838.urdf");
  EXPECT_TRUE(nullptr != skeleton);

  std::vector<Eigen::Vector4d> colors;
  colors.push_back(Eigen::Vector4d(0.0, 0.0, 0.8, 1.0));
  colors.push_back(Eigen::Vector4d(1.0, 0.0, 0.0, 1.0));
  colors.push_back(Eigen::Vector4d(1.0, 1.0, 0.0, 1.0));

  EXPECT_EQ(colors.size(), skeleton->getNumBodyNodes());

  for(size_t i=0; i < skeleton->getNumBodyNodes(); ++i)
  {
    const Eigen::Vector4d c = colors[i];
    const dart::dynamics::BodyNode* bn = skeleton->getBodyNode(i);
    for(size_t j=0; j < bn->getNumShapeNodes(); ++j)
    {
      const dart::dynamics::ShapeNode* sn = bn->getShapeNode(j);
      EXPECT_TRUE(equals(sn->getVisualAspect()->getRGBA(), c));
    }
  }
}

//==============================================================================
int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
