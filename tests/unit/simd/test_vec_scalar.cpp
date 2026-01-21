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

namespace dart::simd {

template <typename VecType>
class VecScalarTest : public ::testing::Test
{
protected:
  using vec_type = VecType;
  using scalar_type = typename VecType::scalar_type;
  static constexpr std::size_t width = VecType::width;

  static constexpr scalar_type tolerance()
  {
    if constexpr (std::is_same_v<scalar_type, float>) {
      return 1e-6f;
    } else if constexpr (std::is_same_v<scalar_type, double>) {
      return 1e-15;
    } else {
      return scalar_type{0};
    }
  }

  void expect_near(scalar_type a, scalar_type b)
  {
    if constexpr (std::is_floating_point_v<scalar_type>) {
      EXPECT_NEAR(a, b, tolerance());
    } else {
      EXPECT_EQ(a, b);
    }
  }
};

using VecTypes = ::testing::Types<
    Vec<float, 4>,
    Vec<double, 4>,
    Vec<float, 8>,
    Vec<double, 2>,
    Vec<std::int32_t, 4>,
    Vec<std::int32_t, 8>>;

TYPED_TEST_SUITE(VecScalarTest, VecTypes);

TYPED_TEST(VecScalarTest, Zero)
{
  using vec_type = typename TestFixture::vec_type;
  using scalar_type = typename TestFixture::scalar_type;
  constexpr auto width = TestFixture::width;

  auto v = vec_type::zero();
  for (std::size_t i = 0; i < width; ++i) {
    EXPECT_EQ(v[i], scalar_type{0});
  }
}

TYPED_TEST(VecScalarTest, Broadcast)
{
  using vec_type = typename TestFixture::vec_type;
  using scalar_type = typename TestFixture::scalar_type;
  constexpr auto width = TestFixture::width;

  scalar_type val = scalar_type{42};
  auto v = vec_type::broadcast(val);
  for (std::size_t i = 0; i < width; ++i) {
    this->expect_near(v[i], val);
  }
}

TYPED_TEST(VecScalarTest, LoadStore)
{
  using vec_type = typename TestFixture::vec_type;
  using scalar_type = typename TestFixture::scalar_type;
  constexpr auto width = TestFixture::width;

  std::vector<scalar_type> data(width);
  for (std::size_t i = 0; i < width; ++i) {
    data[i] = static_cast<scalar_type>(i + 1);
  }

  auto v = vec_type::load(data.data());
  for (std::size_t i = 0; i < width; ++i) {
    this->expect_near(v[i], data[i]);
  }

  std::vector<scalar_type> out(width);
  v.store(out.data());
  for (std::size_t i = 0; i < width; ++i) {
    this->expect_near(out[i], data[i]);
  }
}

TYPED_TEST(VecScalarTest, Addition)
{
  using vec_type = typename TestFixture::vec_type;
  using scalar_type = typename TestFixture::scalar_type;
  constexpr auto width = TestFixture::width;

  auto a = vec_type::broadcast(scalar_type{10});
  auto b = vec_type::broadcast(scalar_type{32});
  auto c = a + b;

  for (std::size_t i = 0; i < width; ++i) {
    this->expect_near(c[i], scalar_type{42});
  }
}

TYPED_TEST(VecScalarTest, Subtraction)
{
  using vec_type = typename TestFixture::vec_type;
  using scalar_type = typename TestFixture::scalar_type;
  constexpr auto width = TestFixture::width;

  auto a = vec_type::broadcast(scalar_type{50});
  auto b = vec_type::broadcast(scalar_type{8});
  auto c = a - b;

  for (std::size_t i = 0; i < width; ++i) {
    this->expect_near(c[i], scalar_type{42});
  }
}

TYPED_TEST(VecScalarTest, Multiplication)
{
  using vec_type = typename TestFixture::vec_type;
  using scalar_type = typename TestFixture::scalar_type;
  constexpr auto width = TestFixture::width;

  auto a = vec_type::broadcast(scalar_type{6});
  auto b = vec_type::broadcast(scalar_type{7});
  auto c = a * b;

  for (std::size_t i = 0; i < width; ++i) {
    this->expect_near(c[i], scalar_type{42});
  }
}

TYPED_TEST(VecScalarTest, Division)
{
  using vec_type = typename TestFixture::vec_type;
  using scalar_type = typename TestFixture::scalar_type;
  constexpr auto width = TestFixture::width;

  auto a = vec_type::broadcast(scalar_type{84});
  auto b = vec_type::broadcast(scalar_type{2});
  auto c = a / b;

  for (std::size_t i = 0; i < width; ++i) {
    this->expect_near(c[i], scalar_type{42});
  }
}

TYPED_TEST(VecScalarTest, Negation)
{
  using vec_type = typename TestFixture::vec_type;
  using scalar_type = typename TestFixture::scalar_type;
  constexpr auto width = TestFixture::width;

  auto a = vec_type::broadcast(scalar_type{42});
  auto b = -a;

  for (std::size_t i = 0; i < width; ++i) {
    this->expect_near(b[i], scalar_type{-42});
  }
}

TYPED_TEST(VecScalarTest, CompoundAssignment)
{
  using vec_type = typename TestFixture::vec_type;
  using scalar_type = typename TestFixture::scalar_type;
  constexpr auto width = TestFixture::width;

  auto a = vec_type::broadcast(scalar_type{10});
  auto b = vec_type::broadcast(scalar_type{5});

  a += b;
  for (std::size_t i = 0; i < width; ++i) {
    this->expect_near(a[i], scalar_type{15});
  }

  a -= b;
  for (std::size_t i = 0; i < width; ++i) {
    this->expect_near(a[i], scalar_type{10});
  }

  a *= vec_type::broadcast(scalar_type{2});
  for (std::size_t i = 0; i < width; ++i) {
    this->expect_near(a[i], scalar_type{20});
  }

  a /= vec_type::broadcast(scalar_type{2});
  for (std::size_t i = 0; i < width; ++i) {
    this->expect_near(a[i], scalar_type{10});
  }
}

TYPED_TEST(VecScalarTest, Comparison)
{
  using vec_type = typename TestFixture::vec_type;
  using scalar_type = typename TestFixture::scalar_type;
  constexpr auto width = TestFixture::width;

  std::vector<scalar_type> data_a(width);
  std::vector<scalar_type> data_b(width);
  for (std::size_t i = 0; i < width; ++i) {
    data_a[i] = static_cast<scalar_type>(i);
    data_b[i] = static_cast<scalar_type>(width / 2);
  }

  auto a = vec_type::load(data_a.data());
  auto b = vec_type::load(data_b.data());

  auto lt = (a < b);
  auto gt = (a > b);
  auto eq = (a == b);

  for (std::size_t i = 0; i < width; ++i) {
    scalar_type av = data_a[i];
    scalar_type bv = data_b[i];
    EXPECT_EQ(lt[i], av < bv) << "at index " << i;
    EXPECT_EQ(gt[i], av > bv) << "at index " << i;
    EXPECT_EQ(eq[i], av == bv) << "at index " << i;
  }
}

TYPED_TEST(VecScalarTest, ElementAccess)
{
  using vec_type = typename TestFixture::vec_type;
  using scalar_type = typename TestFixture::scalar_type;
  constexpr auto width = TestFixture::width;

  vec_type v;
  for (std::size_t i = 0; i < width; ++i) {
    v[i] = static_cast<scalar_type>(i * 10);
  }

  for (std::size_t i = 0; i < width; ++i) {
    this->expect_near(v[i], static_cast<scalar_type>(i * 10));
  }
}

TYPED_TEST(VecScalarTest, ArrayConstruction)
{
  using vec_type = typename TestFixture::vec_type;
  using scalar_type = typename TestFixture::scalar_type;
  constexpr auto width = TestFixture::width;

  std::array<scalar_type, width> arr;
  for (std::size_t i = 0; i < width; ++i) {
    arr[i] = static_cast<scalar_type>(i + 1);
  }

  vec_type v(arr);
  for (std::size_t i = 0; i < width; ++i) {
    this->expect_near(v[i], arr[i]);
  }
}

template <typename VecType>
class VecFloatOnlyTest : public VecScalarTest<VecType>
{
};

using FloatVecTypes = ::testing::
    Types<Vec<float, 4>, Vec<double, 4>, Vec<float, 8>, Vec<double, 2>>;

TYPED_TEST_SUITE(VecFloatOnlyTest, FloatVecTypes);

TYPED_TEST(VecFloatOnlyTest, LoadStoreUnaligned)
{
  using vec_type = typename TestFixture::vec_type;
  using scalar_type = typename TestFixture::scalar_type;
  constexpr auto width = TestFixture::width;

  std::vector<scalar_type> data(width + 1);
  for (std::size_t i = 0; i < width; ++i) {
    data[i + 1] = static_cast<scalar_type>(i + 1);
  }

  auto v = vec_type::loadu(data.data() + 1);
  for (std::size_t i = 0; i < width; ++i) {
    this->expect_near(v[i], data[i + 1]);
  }

  std::vector<scalar_type> out(width + 1);
  v.storeu(out.data() + 1);
  for (std::size_t i = 0; i < width; ++i) {
    this->expect_near(out[i + 1], data[i + 1]);
  }
}

} // namespace dart::simd
