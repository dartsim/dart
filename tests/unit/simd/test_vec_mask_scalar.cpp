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

TYPED_TEST(VecMaskScalarTest, MixedValues)
{
  using mask_type = typename TestFixture::mask_type;
  constexpr auto width = TestFixture::width;

  std::array<bool, width> arr;
  int expected_popcount = 0;
  for (std::size_t i = 0; i < width; ++i) {
    arr[i] = (i % 2 == 0);
    if (arr[i])
      ++expected_popcount;
  }

  mask_type m(arr);
  EXPECT_FALSE(m.all());
  EXPECT_TRUE(m.any());
  EXPECT_FALSE(m.none());
  EXPECT_EQ(m.popcount(), expected_popcount);

  for (std::size_t i = 0; i < width; ++i) {
    EXPECT_EQ(m[i], arr[i]);
  }
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

  std::array<bool, width> alt;
  std::uint32_t expected_alt = 0;
  for (std::size_t i = 0; i < width; ++i) {
    alt[i] = (i % 2 == 0);
    if (alt[i])
      expected_alt |= (1u << i);
  }
  mask_type m_alt(alt);
  EXPECT_EQ(m_alt.bitmask(), expected_alt);
}

TYPED_TEST(VecMaskScalarTest, LogicalAnd)
{
  using mask_type = typename TestFixture::mask_type;
  constexpr auto width = TestFixture::width;

  std::array<bool, width> arr_a, arr_b;
  for (std::size_t i = 0; i < width; ++i) {
    arr_a[i] = (i < width / 2);
    arr_b[i] = (i % 2 == 0);
  }

  mask_type a(arr_a);
  mask_type b(arr_b);
  auto result = a & b;

  for (std::size_t i = 0; i < width; ++i) {
    EXPECT_EQ(result[i], arr_a[i] && arr_b[i]) << "at index " << i;
  }

  auto result2 = a && b;
  EXPECT_EQ(result.bitmask(), result2.bitmask());
}

TYPED_TEST(VecMaskScalarTest, LogicalOr)
{
  using mask_type = typename TestFixture::mask_type;
  constexpr auto width = TestFixture::width;

  std::array<bool, width> arr_a, arr_b;
  for (std::size_t i = 0; i < width; ++i) {
    arr_a[i] = (i < width / 2);
    arr_b[i] = (i % 2 == 0);
  }

  mask_type a(arr_a);
  mask_type b(arr_b);
  auto result = a | b;

  for (std::size_t i = 0; i < width; ++i) {
    EXPECT_EQ(result[i], arr_a[i] || arr_b[i]) << "at index " << i;
  }

  auto result2 = a || b;
  EXPECT_EQ(result.bitmask(), result2.bitmask());
}

TYPED_TEST(VecMaskScalarTest, LogicalXor)
{
  using mask_type = typename TestFixture::mask_type;
  constexpr auto width = TestFixture::width;

  std::array<bool, width> arr_a, arr_b;
  for (std::size_t i = 0; i < width; ++i) {
    arr_a[i] = (i < width / 2);
    arr_b[i] = (i % 2 == 0);
  }

  mask_type a(arr_a);
  mask_type b(arr_b);
  auto result = a ^ b;

  for (std::size_t i = 0; i < width; ++i) {
    EXPECT_EQ(result[i], arr_a[i] != arr_b[i]) << "at index " << i;
  }
}

TYPED_TEST(VecMaskScalarTest, LogicalNot)
{
  using mask_type = typename TestFixture::mask_type;
  constexpr auto width = TestFixture::width;

  std::array<bool, width> arr;
  for (std::size_t i = 0; i < width; ++i) {
    arr[i] = (i % 2 == 0);
  }

  mask_type m(arr);
  auto result = ~m;

  for (std::size_t i = 0; i < width; ++i) {
    EXPECT_EQ(result[i], !arr[i]) << "at index " << i;
  }
}

TYPED_TEST(VecMaskScalarTest, CompoundAssignment)
{
  using mask_type = typename TestFixture::mask_type;
  constexpr auto width = TestFixture::width;

  std::array<bool, width> arr_a, arr_b;
  for (std::size_t i = 0; i < width; ++i) {
    arr_a[i] = (i < width / 2);
    arr_b[i] = (i % 2 == 0);
  }

  mask_type a(arr_a);
  mask_type b(arr_b);

  mask_type test = a;
  test &= b;
  for (std::size_t i = 0; i < width; ++i) {
    EXPECT_EQ(test[i], arr_a[i] && arr_b[i]);
  }

  test = mask_type(arr_a);
  test |= b;
  for (std::size_t i = 0; i < width; ++i) {
    EXPECT_EQ(test[i], arr_a[i] || arr_b[i]);
  }

  test = mask_type(arr_a);
  test ^= b;
  for (std::size_t i = 0; i < width; ++i) {
    EXPECT_EQ(test[i], arr_a[i] != arr_b[i]);
  }
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

  auto a = vec_type::load(data_a.data());
  auto b = vec_type::load(data_b.data());

  auto mask = (a == b);
  for (std::size_t i = 0; i < width; ++i) {
    EXPECT_EQ(mask[i], data_a[i] == data_b[i]);
  }
}

} // namespace dart::simd
