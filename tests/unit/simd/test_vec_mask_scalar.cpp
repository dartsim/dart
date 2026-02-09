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

namespace dart::simd {

template <typename MaskType>
class VecMaskScalarTest : public ::testing::Test
{
protected:
  using mask_type = MaskType;
  using scalar_type = typename MaskType::scalar_type;
  static constexpr std::size_t width = MaskType::width;
};

using MaskTypes = ::testing::Types<
    VecMask<float, 4>,
    VecMask<double, 4>,
    VecMask<float, 8>,
    VecMask<double, 2>,
    VecMask<std::int32_t, 4>,
    VecMask<std::int32_t, 8>>;

TYPED_TEST_SUITE(VecMaskScalarTest, MaskTypes);

TYPED_TEST(VecMaskScalarTest, AllTrue)
{
  using mask_type = typename TestFixture::mask_type;
  constexpr auto width = TestFixture::width;

  mask_type m(true);
  EXPECT_TRUE(m.all());
  EXPECT_TRUE(m.any());
  EXPECT_FALSE(m.none());
  EXPECT_EQ(m.popcount(), static_cast<int>(width));

  for (std::size_t i = 0; i < width; ++i) {
    EXPECT_TRUE(m[i]);
  }
}

TYPED_TEST(VecMaskScalarTest, AllFalse)
{
  using mask_type = typename TestFixture::mask_type;
  constexpr auto width = TestFixture::width;

  mask_type m(false);
  EXPECT_FALSE(m.all());
  EXPECT_FALSE(m.any());
  EXPECT_TRUE(m.none());
  EXPECT_EQ(m.popcount(), 0);

  for (std::size_t i = 0; i < width; ++i) {
    EXPECT_FALSE(m[i]);
  }
}

TYPED_TEST(VecMaskScalarTest, XorOperations)
{
  using mask_type = typename TestFixture::mask_type;
  constexpr auto width = TestFixture::width;

  mask_type all_true(true);
  mask_type all_false(false);

  mask_type true_xor_false = all_true ^ all_false;
  EXPECT_TRUE(true_xor_false.all());
  EXPECT_TRUE(true_xor_false.any());
  EXPECT_FALSE(true_xor_false.none());
  EXPECT_EQ(true_xor_false.popcount(), static_cast<int>(width));

  mask_type true_xor_true = all_true ^ all_true;
  EXPECT_FALSE(true_xor_true.all());
  EXPECT_FALSE(true_xor_true.any());
  EXPECT_TRUE(true_xor_true.none());
  EXPECT_EQ(true_xor_true.popcount(), 0);
}

TYPED_TEST(VecMaskScalarTest, Bitmask)
{
  using mask_type = typename TestFixture::mask_type;
  constexpr auto width = TestFixture::width;

  mask_type all_true(true);
  std::uint32_t expected_all = (1u << width) - 1u;
  EXPECT_EQ(all_true.bitmask(), expected_all);

  mask_type all_false(false);
  EXPECT_EQ(all_false.bitmask(), 0u);
}

TYPED_TEST(VecMaskScalarTest, LogicalAnd)
{
  using mask_type = typename TestFixture::mask_type;

  mask_type a(true);
  mask_type b(false);
  auto result = a & b;

  EXPECT_FALSE(result.all());
  EXPECT_FALSE(result.any());
  EXPECT_TRUE(result.none());

  auto result2 = a & a;
  EXPECT_TRUE(result2.all());

  auto result3 = a && b;
  EXPECT_EQ(result.bitmask(), result3.bitmask());
}

TYPED_TEST(VecMaskScalarTest, LogicalOr)
{
  using mask_type = typename TestFixture::mask_type;

  mask_type a(true);
  mask_type b(false);
  auto result = a | b;

  EXPECT_TRUE(result.all());
  EXPECT_TRUE(result.any());
  EXPECT_FALSE(result.none());

  auto result2 = b | b;
  EXPECT_FALSE(result2.any());

  auto result3 = a || b;
  EXPECT_EQ(result.bitmask(), result3.bitmask());
}

TYPED_TEST(VecMaskScalarTest, LogicalXor)
{
  using mask_type = typename TestFixture::mask_type;

  mask_type a(true);
  mask_type b(false);
  auto result = a ^ b;

  EXPECT_TRUE(result.all());

  auto result2 = a ^ a;
  EXPECT_FALSE(result2.any());

  auto result3 = b ^ b;
  EXPECT_FALSE(result3.any());
}

TYPED_TEST(VecMaskScalarTest, LogicalNot)
{
  using mask_type = typename TestFixture::mask_type;

  mask_type a(true);
  auto result = ~a;

  EXPECT_FALSE(result.all());
  EXPECT_FALSE(result.any());
  EXPECT_TRUE(result.none());

  mask_type b(false);
  auto result2 = ~b;

  EXPECT_TRUE(result2.all());
  EXPECT_TRUE(result2.any());
  EXPECT_FALSE(result2.none());
}

TYPED_TEST(VecMaskScalarTest, CompoundAssignment)
{
  using mask_type = typename TestFixture::mask_type;

  mask_type a(true);
  mask_type b(false);

  mask_type test = a;
  test &= b;
  EXPECT_FALSE(test.any());

  test = a;
  test |= b;
  EXPECT_TRUE(test.all());

  test = a;
  test ^= b;
  EXPECT_TRUE(test.all());

  test = a;
  test ^= a;
  EXPECT_FALSE(test.any());
}

TYPED_TEST(VecMaskScalarTest, Equality)
{
  using mask_type = typename TestFixture::mask_type;

  mask_type a(true);
  mask_type b(true);
  mask_type c(false);

  EXPECT_TRUE(a == b);
  EXPECT_FALSE(a == c);
  EXPECT_FALSE(a != b);
  EXPECT_TRUE(a != c);
}

TYPED_TEST(VecMaskScalarTest, FromVecComparison)
{
  using scalar_type = typename TestFixture::scalar_type;
  constexpr auto width = TestFixture::width;

  using vec_type = Vec<scalar_type, width>;

  std::vector<scalar_type> data_a(width), data_b(width);
  for (std::size_t i = 0; i < width; ++i) {
    data_a[i] = static_cast<scalar_type>(i);
    data_b[i] = static_cast<scalar_type>(width / 2);
  }

  auto a = vec_type::loadu(data_a.data());
  auto b = vec_type::loadu(data_b.data());

  auto mask = (a == b);
  for (std::size_t i = 0; i < width; ++i) {
    EXPECT_EQ(mask[i], data_a[i] == data_b[i]);
  }
}

} // namespace dart::simd
