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

#include <dart/simd/simd.hpp>

#include <gtest/gtest.h>

#include <limits>

#include <cmath>

using namespace dart::simd;

class SimdEdgeCasesFloat4 : public ::testing::Test
{
protected:
  static constexpr float inf = std::numeric_limits<float>::infinity();
  static constexpr float nan_val = std::numeric_limits<float>::quiet_NaN();
  static constexpr float denorm = std::numeric_limits<float>::denorm_min();
  static constexpr float max_val = std::numeric_limits<float>::max();
  static constexpr float min_val = std::numeric_limits<float>::min();
};

TEST_F(SimdEdgeCasesFloat4, Infinity)
{
  alignas(16) float data[4] = {1.0f, inf, -inf, 2.0f};
  auto v = Vec<float, 4>::load(data);

  auto sum = v + Vec<float, 4>::broadcast(1.0f);
  EXPECT_FLOAT_EQ(sum[0], 2.0f);
  EXPECT_TRUE(std::isinf(sum[1]) && sum[1] > 0);
  EXPECT_TRUE(std::isinf(sum[2]) && sum[2] < 0);
  EXPECT_FLOAT_EQ(sum[3], 3.0f);
}

TEST_F(SimdEdgeCasesFloat4, NaN)
{
  alignas(16) float data[4] = {1.0f, nan_val, 3.0f, nan_val};
  auto v = Vec<float, 4>::load(data);

  auto sum = v + Vec<float, 4>::broadcast(1.0f);
  EXPECT_FLOAT_EQ(sum[0], 2.0f);
  EXPECT_TRUE(std::isnan(sum[1]));
  EXPECT_FLOAT_EQ(sum[2], 4.0f);
  EXPECT_TRUE(std::isnan(sum[3]));
}

TEST_F(SimdEdgeCasesFloat4, DenormalizedNumbers)
{
  alignas(16) float data[4] = {denorm, denorm * 2, denorm * 3, denorm * 4};
  auto v = Vec<float, 4>::load(data);

  auto doubled = v + v;
  EXPECT_NEAR(doubled[0], denorm * 2, denorm);
  EXPECT_NEAR(doubled[1], denorm * 4, denorm);
}

TEST_F(SimdEdgeCasesFloat4, MaxValues)
{
  alignas(16) float data[4] = {max_val, max_val / 2, -max_val, -max_val / 2};
  auto v = Vec<float, 4>::load(data);

  auto halved = v / Vec<float, 4>::broadcast(2.0f);
  EXPECT_FLOAT_EQ(halved[0], max_val / 2);
  EXPECT_FLOAT_EQ(halved[1], max_val / 4);
}

TEST_F(SimdEdgeCasesFloat4, MinValues)
{
  alignas(16) float data[4] = {min_val, min_val * 2, min_val * 4, min_val * 8};
  auto v = Vec<float, 4>::load(data);

  auto squared = v * v;
  EXPECT_GE(squared[0], 0.0f);
  EXPECT_GE(squared[1], 0.0f);
}

TEST_F(SimdEdgeCasesFloat4, ZeroDivision)
{
  auto a = Vec<float, 4>::broadcast(1.0f);
  auto b = Vec<float, 4>::zero();

  auto result = a / b;
  EXPECT_TRUE(std::isinf(result[0]));
  EXPECT_TRUE(std::isinf(result[1]));
  EXPECT_TRUE(std::isinf(result[2]));
  EXPECT_TRUE(std::isinf(result[3]));
}

TEST_F(SimdEdgeCasesFloat4, NegativeZero)
{
  alignas(16) float data[4] = {0.0f, -0.0f, 1.0f, -1.0f};
  auto v = Vec<float, 4>::load(data);

  auto neg = -v;
  EXPECT_FLOAT_EQ(neg[0], -0.0f);
  EXPECT_FLOAT_EQ(neg[2], -1.0f);
  EXPECT_FLOAT_EQ(neg[3], 1.0f);
}

class SimdEdgeCasesDouble2 : public ::testing::Test
{
protected:
  static constexpr double inf = std::numeric_limits<double>::infinity();
  static constexpr double nan_val = std::numeric_limits<double>::quiet_NaN();
  static constexpr double denorm = std::numeric_limits<double>::denorm_min();
};

TEST_F(SimdEdgeCasesDouble2, Infinity)
{
  alignas(16) double data[2] = {inf, -inf};
  auto v = Vec<double, 2>::load(data);

  auto sum = v + Vec<double, 2>::broadcast(1.0);
  EXPECT_TRUE(std::isinf(sum[0]) && sum[0] > 0);
  EXPECT_TRUE(std::isinf(sum[1]) && sum[1] < 0);
}

TEST_F(SimdEdgeCasesDouble2, NaN)
{
  alignas(16) double data[2] = {nan_val, 1.0};
  auto v = Vec<double, 2>::load(data);

  auto prod = v * Vec<double, 2>::broadcast(2.0);
  EXPECT_TRUE(std::isnan(prod[0]));
  EXPECT_DOUBLE_EQ(prod[1], 2.0);
}

class SimdAlignmentTest : public ::testing::Test
{
};

TEST_F(SimdAlignmentTest, AlignedVector)
{
  aligned_vector<float> vec(16, 1.0f);
  EXPECT_TRUE(is_aligned(vec.data(), 32));

  for (std::size_t i = 0; i < 16; i += 4) {
    auto v = Vec<float, 4>::load(&vec[i]);
    EXPECT_FLOAT_EQ(v[0], 1.0f);
    EXPECT_FLOAT_EQ(v[1], 1.0f);
    EXPECT_FLOAT_EQ(v[2], 1.0f);
    EXPECT_FLOAT_EQ(v[3], 1.0f);
  }
}

TEST_F(SimdAlignmentTest, AlignedVectorResize)
{
  aligned_vector<double> vec;
  vec.resize(100, 2.5);
  EXPECT_TRUE(is_aligned(vec.data(), default_vector_alignment));
  EXPECT_EQ(vec.size(), 100u);
  EXPECT_DOUBLE_EQ(vec[50], 2.5);
}

TEST_F(SimdAlignmentTest, UnalignedLoadStore)
{
  std::vector<float> unaligned(10, 0.0f);
  for (int i = 0; i < 10; ++i) {
    unaligned[i] = static_cast<float>(i);
  }

  auto v = Vec<float, 4>::loadu(&unaligned[1]);
  EXPECT_FLOAT_EQ(v[0], 1.0f);
  EXPECT_FLOAT_EQ(v[1], 2.0f);
  EXPECT_FLOAT_EQ(v[2], 3.0f);
  EXPECT_FLOAT_EQ(v[3], 4.0f);

  v.storeu(&unaligned[5]);
  EXPECT_FLOAT_EQ(unaligned[5], 1.0f);
  EXPECT_FLOAT_EQ(unaligned[6], 2.0f);
  EXPECT_FLOAT_EQ(unaligned[7], 3.0f);
  EXPECT_FLOAT_EQ(unaligned[8], 4.0f);
}

TEST_F(SimdAlignmentTest, IsAlignedCheck)
{
  alignas(64) float aligned64[16];
  alignas(32) float aligned32[8];
  alignas(16) float aligned16[4];

  EXPECT_TRUE(is_aligned(aligned64, 16));
  EXPECT_TRUE(is_aligned(aligned64, 32));
  EXPECT_TRUE(is_aligned(aligned64, 64));

  EXPECT_TRUE(is_aligned(aligned32, 16));
  EXPECT_TRUE(is_aligned(aligned32, 32));

  EXPECT_TRUE(is_aligned(aligned16, 16));
}

class SimdMathEdgeCases : public ::testing::Test
{
};

TEST_F(SimdMathEdgeCases, SqrtZero)
{
  auto v = Vec<float, 4>::zero();
  auto result = sqrt(v);
  EXPECT_FLOAT_EQ(result[0], 0.0f);
  EXPECT_FLOAT_EQ(result[1], 0.0f);
  EXPECT_FLOAT_EQ(result[2], 0.0f);
  EXPECT_FLOAT_EQ(result[3], 0.0f);
}

TEST_F(SimdMathEdgeCases, SqrtNegative)
{
  alignas(16) float data[4] = {-1.0f, -4.0f, -9.0f, -16.0f};
  auto v = Vec<float, 4>::load(data);
  auto result = sqrt(v);
  EXPECT_TRUE(std::isnan(result[0]));
  EXPECT_TRUE(std::isnan(result[1]));
  EXPECT_TRUE(std::isnan(result[2]));
  EXPECT_TRUE(std::isnan(result[3]));
}

TEST_F(SimdMathEdgeCases, RsqrtSmallValues)
{
  alignas(16) float data[4] = {1.0f, 4.0f, 9.0f, 16.0f};
  auto v = Vec<float, 4>::load(data);
  auto result = rsqrt(v);

  EXPECT_NEAR(result[0], 1.0f, 0.001f);
  EXPECT_NEAR(result[1], 0.5f, 0.001f);
  EXPECT_NEAR(result[2], 1.0f / 3.0f, 0.001f);
  EXPECT_NEAR(result[3], 0.25f, 0.001f);
}

TEST_F(SimdMathEdgeCases, RcpAccuracy)
{
  alignas(16) float data[4] = {1.0f, 2.0f, 4.0f, 8.0f};
  auto v = Vec<float, 4>::load(data);
  auto result = rcp(v);

  EXPECT_NEAR(result[0], 1.0f, 0.001f);
  EXPECT_NEAR(result[1], 0.5f, 0.001f);
  EXPECT_NEAR(result[2], 0.25f, 0.001f);
  EXPECT_NEAR(result[3], 0.125f, 0.001f);
}

TEST_F(SimdMathEdgeCases, SelectAllTrue)
{
  auto if_true = Vec<float, 4>::broadcast(1.0f);
  auto if_false = Vec<float, 4>::broadcast(2.0f);
  auto mask = VecMask<float, 4>(true);

  auto result = select(mask, if_true, if_false);
  EXPECT_FLOAT_EQ(result[0], 1.0f);
  EXPECT_FLOAT_EQ(result[1], 1.0f);
  EXPECT_FLOAT_EQ(result[2], 1.0f);
  EXPECT_FLOAT_EQ(result[3], 1.0f);
}

TEST_F(SimdMathEdgeCases, SelectAllFalse)
{
  auto if_true = Vec<float, 4>::broadcast(1.0f);
  auto if_false = Vec<float, 4>::broadcast(2.0f);
  auto mask = VecMask<float, 4>(false);

  auto result = select(mask, if_true, if_false);
  EXPECT_FLOAT_EQ(result[0], 2.0f);
  EXPECT_FLOAT_EQ(result[1], 2.0f);
  EXPECT_FLOAT_EQ(result[2], 2.0f);
  EXPECT_FLOAT_EQ(result[3], 2.0f);
}

TEST_F(SimdMathEdgeCases, HsumSingleElement)
{
  alignas(16) double data[2] = {3.0, 7.0};
  auto v = Vec<double, 2>::load(data);
  EXPECT_DOUBLE_EQ(hsum(v), 10.0);
}

TEST_F(SimdMathEdgeCases, HminHmaxEqual)
{
  auto v = Vec<float, 4>::broadcast(5.0f);
  EXPECT_FLOAT_EQ(hmin(v), 5.0f);
  EXPECT_FLOAT_EQ(hmax(v), 5.0f);
}

TEST_F(SimdMathEdgeCases, FloorCeilRound)
{
  alignas(16) float data[4] = {1.2f, 1.5f, 1.7f, -1.5f};
  auto v = Vec<float, 4>::load(data);

  auto fl = floor(v);
  EXPECT_FLOAT_EQ(fl[0], 1.0f);
  EXPECT_FLOAT_EQ(fl[1], 1.0f);
  EXPECT_FLOAT_EQ(fl[2], 1.0f);
  EXPECT_FLOAT_EQ(fl[3], -2.0f);

  auto ce = ceil(v);
  EXPECT_FLOAT_EQ(ce[0], 2.0f);
  EXPECT_FLOAT_EQ(ce[1], 2.0f);
  EXPECT_FLOAT_EQ(ce[2], 2.0f);
  EXPECT_FLOAT_EQ(ce[3], -1.0f);

  auto tr = trunc(v);
  EXPECT_FLOAT_EQ(tr[0], 1.0f);
  EXPECT_FLOAT_EQ(tr[1], 1.0f);
  EXPECT_FLOAT_EQ(tr[2], 1.0f);
  EXPECT_FLOAT_EQ(tr[3], -1.0f);
}

TEST_F(SimdMathEdgeCases, FmaAccuracy)
{
  alignas(16) float a_data[4] = {1.0f, 2.0f, 3.0f, 4.0f};
  alignas(16) float b_data[4] = {2.0f, 3.0f, 4.0f, 5.0f};
  alignas(16) float c_data[4] = {3.0f, 4.0f, 5.0f, 6.0f};

  auto a = Vec<float, 4>::load(a_data);
  auto b = Vec<float, 4>::load(b_data);
  auto c = Vec<float, 4>::load(c_data);

  auto result = fmadd(a, b, c);
  EXPECT_FLOAT_EQ(result[0], 1.0f * 2.0f + 3.0f);
  EXPECT_FLOAT_EQ(result[1], 2.0f * 3.0f + 4.0f);
  EXPECT_FLOAT_EQ(result[2], 3.0f * 4.0f + 5.0f);
  EXPECT_FLOAT_EQ(result[3], 4.0f * 5.0f + 6.0f);
}
