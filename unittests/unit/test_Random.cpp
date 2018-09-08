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
using namespace dart::math;

//==============================================================================
TEST(Random, SyntaxValidityCheck)
{
//  bool result;

  const int rows = 5;
  const int cols = 5;

  const int size = 5;

  int mini = -5.0;
  int maxi = 10.0;

  float minf = -3.0f;
  float maxf = 4.0f;

  double mind = -5.0;
  double maxd = 10.0;

  Eigen::VectorXi minVecXi = Eigen::VectorXi::Constant(size, mini);
  Eigen::VectorXi maxVecXi = Eigen::VectorXi::Constant(size, maxi);
  Eigen::VectorXf minVecXf = Eigen::VectorXf::Constant(size, minf);
  Eigen::VectorXf maxVecXf = Eigen::VectorXf::Constant(size, maxf);
  Eigen::VectorXd minVecXd = Eigen::VectorXd::Constant(size, mind);
  Eigen::VectorXd maxVecXd = Eigen::VectorXd::Constant(size, maxd);

  Eigen::Vector3i minVec3i = Eigen::Vector3i::Constant(mini);
  Eigen::Vector3i maxVec3i = Eigen::Vector3i::Constant(maxi);
  Eigen::Vector3f minVec3f = Eigen::Vector3f::Constant(minf);
  Eigen::Vector3f maxVec3f = Eigen::Vector3f::Constant(maxf);
  Eigen::Vector3d minVec3d = Eigen::Vector3d::Constant(mind);
  Eigen::Vector3d maxVec3d = Eigen::Vector3d::Constant(maxd);

  Eigen::MatrixXi minMatXi = Eigen::MatrixXi::Constant(rows, cols, mini);
  Eigen::MatrixXi maxMatXi = Eigen::MatrixXi::Constant(rows, cols, maxi);
  Eigen::MatrixXf minMatXf = Eigen::MatrixXf::Constant(rows, cols, minf);
  Eigen::MatrixXf maxMatXf = Eigen::MatrixXf::Constant(rows, cols, maxf);
  Eigen::MatrixXd minMatXd = Eigen::MatrixXd::Constant(rows, cols, mind);
  Eigen::MatrixXd maxMatXd = Eigen::MatrixXd::Constant(rows, cols, maxd);

  Eigen::Matrix3i minMat3i = Eigen::Matrix3i::Constant(mini);
  Eigen::Matrix3i maxMat3i = Eigen::Matrix3i::Constant(maxi);
  Eigen::Matrix3f minMat3f = Eigen::Matrix3f::Constant(minf);
  Eigen::Matrix3f maxMat3f = Eigen::Matrix3f::Constant(maxf);
  Eigen::Matrix3d minMat3d = Eigen::Matrix3d::Constant(mind);
  Eigen::Matrix3d maxMat3d = Eigen::Matrix3d::Constant(maxd);

  // -- Create random vectors without template parameters.
  //
  // The output type will be inferred from the arguments.

  // Create random scalars given scalar bounds
  Random::uniform(mini, maxi);
  Random::uniform(minf, maxf);
  Random::uniform(mind, maxd);

  // Create random vectors given dynamic size vector bounds
  Random::uniform(minVecXi, maxVecXi);
  Random::uniform(minVecXf, maxVecXf);
  Random::uniform(minVecXd, maxVecXd);

  // Create random vectors given fixed-size vector bounds
  Random::uniform(minVec3i, maxVec3i);
  Random::uniform(minVec3f, maxVec3f);
  Random::uniform(minVec3d, maxVec3d);

  // Create random matrices given dynamic size matrix bounds
  Random::uniform(minMatXi, maxMatXi);
  Random::uniform(minMatXf, maxMatXf);
  Random::uniform(minMatXd, maxMatXd);

  // Create random matrices given fixed-size matrix bounds
  Random::uniform(minMat3i, maxMat3i);
  Random::uniform(minMat3f, maxMat3f);
  Random::uniform(minMat3d, maxMat3d);

  // -- Create random vectors explicitly given template parameters

  // Create random scalars given scalar bounds
  Random::uniform<int>(mini, maxi);
  Random::uniform<float>(minf, maxf);
  Random::uniform<double>(mind, maxd);

  // Create random vectors given scalar bounds
  Random::uniform<Eigen::VectorXi>(size, mini, maxi);
  Random::uniform<Eigen::VectorXf>(size, minf, maxf);
  Random::uniform<Eigen::VectorXd>(size, mind, maxd);

  // Create random vectors given dynamic size vector bounds
  Random::uniform<Eigen::VectorXi>(minVecXi, maxVecXi);
  Random::uniform<Eigen::VectorXf>(minVecXf, maxVecXf);
  Random::uniform<Eigen::VectorXd>(minVecXd, maxVecXd);

  // Create random vectors given fixed-size vector bounds
  Random::uniform<Eigen::Vector3i>(minVec3i, maxVec3i);
  Random::uniform<Eigen::Vector3f>(minVec3f, maxVec3f);
  Random::uniform<Eigen::Vector3d>(minVec3d, maxVec3d);

  // Create random vectors given scalar bounds
  Random::uniform<Eigen::MatrixXi>(rows, cols, mini, maxi);
  Random::uniform<Eigen::MatrixXf>(rows, cols, minf, maxf);
  Random::uniform<Eigen::MatrixXd>(rows, cols, mind, maxd);

  // Create random matrices given dynamic size matrix bounds
  Random::uniform<Eigen::MatrixXi>(minMatXi, maxMatXi);
  Random::uniform<Eigen::MatrixXf>(minMatXf, maxMatXf);
  Random::uniform<Eigen::MatrixXd>(minMatXd, maxMatXd);

  // Create random matrices given fixed-size matrix bounds
  Random::uniform<Eigen::Matrix3i>(minMat3i, maxMat3i);
  Random::uniform<Eigen::Matrix3f>(minMat3f, maxMat3f);
  Random::uniform<Eigen::Matrix3d>(minMat3d, maxMat3d);
}

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

//==============================================================================
TEST(Random, SetSeed)
{
  unsigned int N = 10;

  int min = -10;
  int max = 10;

  std::vector<int> first;
  std::vector<int> second;
  std::vector<int> third;

  for (unsigned int i = 0; i < N; ++i)
  {
    math::Random::setSeed(i);
    first.push_back(math::Random::uniform(min, max));
    second.push_back(math::Random::uniform(min, max));
    third.push_back(math::Random::uniform(min, max));
  }

  for (unsigned int i = 0; i < N; ++i)
  {
    math::Random::setSeed(i);
    EXPECT_EQ(math::Random::getSeed(), static_cast<unsigned int>(i));
    EXPECT_EQ(first[i], math::Random::uniform(min, max));
    EXPECT_EQ(second[i], math::Random::uniform(min, max));
    EXPECT_EQ(third[i], math::Random::uniform(min, max));
  }
}
