/*
 * Copyright (c) 2011-2018, The DART development contributors
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
#include <dart/math/Random.hpp>
#include "TestHelpers.hpp"

using namespace dart;

//==============================================================================
TEST(Random, UniformScalar)
{
  double mind = -5.0;
  double maxd = 10.0;

  float minf = -3.0f;
  float maxf = 4.0f;

  int mini = -5.0;
  int maxi = 10.0;

  double uniformd = math::Random::uniform(mind, maxd);
  EXPECT_GE(uniformd, mind);
  EXPECT_LE(uniformd, maxd);

  float uniformf = math::Random::uniform(minf, maxf);
  EXPECT_GE(uniformf, minf);
  EXPECT_LE(uniformf, maxf);

  int uniformi = math::Random::uniform(mini, maxi);
  EXPECT_GE(uniformi, mini);
  EXPECT_LE(uniformi, maxi);
}

//==============================================================================
TEST(Random, NormalScalar)
{
  double meand = 1.0;
  double sigmad = 10.0;

  float meanf = -3.0f;
  float sigmaf = 4.0f;

  int meani = 1;
  int sigmai = 10;

  // TODO(JS): Not sure what to test because the image of the random values
  // is infinite in theory.
  math::Random::normal(meand, sigmad);
  math::Random::normal(meanf, sigmaf);
  math::Random::normal(meani, sigmai);
}

//==============================================================================
TEST(Random, UniformVector)
{
  const int vectorSize = 5;

  Eigen::VectorXd mind = Eigen::VectorXd::Constant(vectorSize, -5.0);
  Eigen::VectorXd maxd = Eigen::VectorXd::Constant(vectorSize, 10.0);

  Eigen::VectorXf minf = Eigen::VectorXf::Constant(vectorSize, -3.0f);
  Eigen::VectorXf maxf = Eigen::VectorXf::Constant(vectorSize, 4.0f);

  Eigen::VectorXi mini = Eigen::VectorXi::Constant(vectorSize, -5);
  Eigen::VectorXi maxi = Eigen::VectorXi::Constant(vectorSize, 10);

  Eigen::VectorXd uniformd = math::Random::uniform(mind, maxd);
  EXPECT_TRUE((uniformd.array() >= mind.array()).all());
  EXPECT_TRUE((uniformd.array() <= maxd.array()).all());

  Eigen::VectorXf uniformf = math::Random::uniform(minf, maxf);
  EXPECT_TRUE((uniformf.array() >= minf.array()).all());
  EXPECT_TRUE((uniformf.array() <= maxf.array()).all());

  Eigen::VectorXi uniformi = math::Random::uniform(mini, maxi);
  EXPECT_TRUE((uniformi.array() >= mini.array()).all());
  EXPECT_TRUE((uniformi.array() <= maxi.array()).all());
}
