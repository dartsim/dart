/*
 * Copyright (c) 2011-2023, The DART development contributors
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

#include <dart/test/math/GTestUtils.hpp>

#include <dart/math/Random.hpp>

#include <gtest/gtest.h>

using namespace dart;
using namespace dart::math;

//==============================================================================
TEST(Random, CheckSyntaxValidity)
{
  const int rows = 5;
  const int cols = 5;
  const int size = 5;

  short mins = -2;
  short maxs = 3;
  int mini = -1;
  int maxi = 10;
  long minl = -1l;
  long maxl = 10l;
  long long minll = -1ll;
  long long maxll = 10ll;
  unsigned short minus = 2;
  unsigned short maxus = 3;
  unsigned int minui = 1u;
  unsigned int maxui = 10u;
  unsigned long minul = 1ul;
  unsigned long maxul = 10ul;
  unsigned long long minull = 1ull;
  unsigned long long maxull = 10ull;
  float minf = -3.0f;
  float maxf = 4.0f;
  double mind = -5.0;
  double maxd = 10.0;
  long double minld = -5.0l;
  long double maxld = 10.0l;

  math::VectorXi minVecXi = math::VectorXi::Constant(size, mini);
  math::VectorXi maxVecXi = math::VectorXi::Constant(size, maxi);
  math::VectorXf minVecXf = math::VectorXf::Constant(size, minf);
  math::VectorXf maxVecXf = math::VectorXf::Constant(size, maxf);
  math::VectorXd minVecXd = math::VectorXd::Constant(size, mind);
  math::VectorXd maxVecXd = math::VectorXd::Constant(size, maxd);

  math::Vector3i minVec3i = math::Vector3i::Constant(mini);
  math::Vector3i maxVec3i = math::Vector3i::Constant(maxi);
  math::Vector3f minVec3f = math::Vector3f::Constant(minf);
  math::Vector3f maxVec3f = math::Vector3f::Constant(maxf);
  math::Vector3d minVec3d = math::Vector3d::Constant(mind);
  math::Vector3d maxVec3d = math::Vector3d::Constant(maxd);

  math::MatrixXi minMatXi = math::MatrixXi::Constant(rows, cols, mini);
  math::MatrixXi maxMatXi = math::MatrixXi::Constant(rows, cols, maxi);
  math::MatrixXf minMatXf = math::MatrixXf::Constant(rows, cols, minf);
  math::MatrixXf maxMatXf = math::MatrixXf::Constant(rows, cols, maxf);
  math::MatrixXd minMatXd = math::MatrixXd::Constant(rows, cols, mind);
  math::MatrixXd maxMatXd = math::MatrixXd::Constant(rows, cols, maxd);

  math::Matrix3i minMat3i = math::Matrix3i::Constant(mini);
  math::Matrix3i maxMat3i = math::Matrix3i::Constant(maxi);
  math::Matrix3f minMat3f = math::Matrix3f::Constant(minf);
  math::Matrix3f maxMat3f = math::Matrix3f::Constant(maxf);
  math::Matrix3d minMat3d = math::Matrix3d::Constant(mind);
  math::Matrix3d maxMat3d = math::Matrix3d::Constant(maxd);

  // -- Create random vectors without template parameters.
  //
  // The output type will be inferred from the arguments.

  // Create random scalars given scalar bounds
  Random::uniform(mins, maxs);
  Random::uniform(mini, maxi);
  Random::uniform(minl, maxl);
  Random::uniform(minll, maxll);
  Random::uniform(minus, maxus);
  Random::uniform(minui, maxui);
  Random::uniform(minul, maxul);
  Random::uniform(minull, maxull);
  Random::uniform(minf, maxf);
  Random::uniform(mind, maxd);
  Random::uniform(minld, maxld);

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
  Random::uniform<short>(mins, maxs);
  Random::uniform<int>(mini, maxi);
  Random::uniform<long>(minl, maxl);
  Random::uniform<long long>(minll, maxll);
  Random::uniform<unsigned short>(minus, maxus);
  Random::uniform<unsigned int>(minui, maxui);
  Random::uniform<unsigned long>(minul, maxul);
  Random::uniform<unsigned long long>(minull, maxull);
  Random::uniform<float>(minf, maxf);
  Random::uniform<double>(mind, maxd);
  Random::uniform<long double>(minld, maxld);

  // Create random vectors given scalar bounds
  Random::uniform<math::VectorXi>(size, mini, maxi);
  Random::uniform<math::VectorXf>(size, minf, maxf);
  Random::uniform<math::VectorXd>(size, mind, maxd);

  // Create random vectors given dynamic size vector bounds
  Random::uniform<math::VectorXi>(minVecXi, maxVecXi);
  Random::uniform<math::VectorXf>(minVecXf, maxVecXf);
  Random::uniform<math::VectorXd>(minVecXd, maxVecXd);

  // Create random vectors given fixed-size vector bounds
  Random::uniform<math::Vector3i>(minVec3i, maxVec3i);
  Random::uniform<math::Vector3f>(minVec3f, maxVec3f);
  Random::uniform<math::Vector3d>(minVec3d, maxVec3d);

  // Create random vectors given scalar bounds
  Random::uniform<math::MatrixXi>(rows, cols, mini, maxi);
  Random::uniform<math::MatrixXf>(rows, cols, minf, maxf);
  Random::uniform<math::MatrixXd>(rows, cols, mind, maxd);

  // Create random matrices given dynamic size matrix bounds
  Random::uniform<math::MatrixXi>(minMatXi, maxMatXi);
  Random::uniform<math::MatrixXf>(minMatXf, maxMatXf);
  Random::uniform<math::MatrixXd>(minMatXd, maxMatXd);

  // Create random matrices given fixed-size matrix bounds
  Random::uniform<math::Matrix3i>(minMat3i, maxMat3i);
  Random::uniform<math::Matrix3f>(minMat3f, maxMat3f);
  Random::uniform<math::Matrix3d>(minMat3d, maxMat3d);
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
template <typename I>
bool testClosedEnds(int maxTry, I min = 0, I max = 2)
{
  bool foundMin = false;
  bool foundMax = false;

  for (int i = 0; i < maxTry; ++i) {
    const I val = math::Random::uniform<I>(min, max);

    if (val == min)
      foundMin = true;

    if (val == max)
      foundMax = true;

    if (foundMin && foundMax)
      return true;
  }

  return false;
}

//==============================================================================
TEST(Random, UniformClosedEnd)
{
  const int maxTry = 100000;

  EXPECT_TRUE(testClosedEnds<short>(maxTry));
  EXPECT_TRUE(testClosedEnds<int>(maxTry));
  EXPECT_TRUE(testClosedEnds<long>(maxTry));
  EXPECT_TRUE(testClosedEnds<long long>(maxTry));

  EXPECT_TRUE(testClosedEnds<unsigned short>(maxTry));
  EXPECT_TRUE(testClosedEnds<unsigned int>(maxTry));
  EXPECT_TRUE(testClosedEnds<unsigned long>(maxTry));
  EXPECT_TRUE(testClosedEnds<unsigned long long>(maxTry));

  EXPECT_TRUE(testClosedEnds<std::size_t>(maxTry));
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

  math::VectorXd mind = math::VectorXd::Constant(vectorSize, -5.0);
  math::VectorXd maxd = math::VectorXd::Constant(vectorSize, 10.0);

  math::VectorXf minf = math::VectorXf::Constant(vectorSize, -3.0f);
  math::VectorXf maxf = math::VectorXf::Constant(vectorSize, 4.0f);

  math::VectorXi mini = math::VectorXi::Constant(vectorSize, -5);
  math::VectorXi maxi = math::VectorXi::Constant(vectorSize, 10);

  math::VectorXd uniformd = math::Random::uniform(mind, maxd);
  EXPECT_TRUE((uniformd.array() >= mind.array()).all());
  EXPECT_TRUE((uniformd.array() <= maxd.array()).all());

  math::VectorXf uniformf = math::Random::uniform(minf, maxf);
  EXPECT_TRUE((uniformf.array() >= minf.array()).all());
  EXPECT_TRUE((uniformf.array() <= maxf.array()).all());

  math::VectorXi uniformi = math::Random::uniform(mini, maxi);
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

  for (unsigned int i = 0; i < N; ++i) {
    math::Random::setSeed(i);
    first.push_back(math::Random::uniform(min, max));
    second.push_back(math::Random::uniform(min, max));
    third.push_back(math::Random::uniform(min, max));
  }

  for (unsigned int i = 0; i < N; ++i) {
    math::Random::setSeed(i);
    EXPECT_EQ(math::Random::getSeed(), static_cast<unsigned int>(i));
    EXPECT_EQ(first[i], math::Random::uniform(min, max));
    EXPECT_EQ(second[i], math::Random::uniform(min, max));
    EXPECT_EQ(third[i], math::Random::uniform(min, max));
  }
}
