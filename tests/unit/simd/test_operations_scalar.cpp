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

#include <cmath>

namespace dart::simd {

template <typename VecType>
class OperationsScalarTest : public ::testing::Test
{
protected:
  using vec_type = VecType;
  using scalar_type = typename VecType::scalar_type;
  using mask_type = typename VecType::mask_type;
  static constexpr std::size_t width = VecType::width;

  static constexpr scalar_type tolerance()
  {
    if constexpr (std::is_same_v<scalar_type, float>) {
      return 1e-5f;
    } else {
      return 1e-12;
    }
  }

  void expect_near(scalar_type a, scalar_type b)
  {
    EXPECT_NEAR(a, b, tolerance());
  }
};

using FloatVecTypes = ::testing::
    Types<Vec<float, 4>, Vec<double, 4>, Vec<float, 8>, Vec<double, 2>>;

TYPED_TEST_SUITE(OperationsScalarTest, FloatVecTypes);

TYPED_TEST(OperationsScalarTest, Abs)
{
  using vec_type = typename TestFixture::vec_type;
  using scalar_type = typename TestFixture::scalar_type;
  constexpr auto width = TestFixture::width;

  std::vector<scalar_type> data(width);
  for (std::size_t i = 0; i < width; ++i) {
    data[i] = static_cast<scalar_type>((i % 2 == 0) ? -(i + 1) : (i + 1));
  }

  auto v = vec_type::loadu(data.data());
  auto result = abs(v);

  for (std::size_t i = 0; i < width; ++i) {
    this->expect_near(result[i], std::abs(data[i]));
  }
}

TYPED_TEST(OperationsScalarTest, Sqrt)
{
  using vec_type = typename TestFixture::vec_type;
  using scalar_type = typename TestFixture::scalar_type;
  constexpr auto width = TestFixture::width;

  std::vector<scalar_type> data(width);
  for (std::size_t i = 0; i < width; ++i) {
    data[i] = static_cast<scalar_type>((i + 1) * (i + 1));
  }

  auto v = vec_type::loadu(data.data());
  auto result = sqrt(v);

  for (std::size_t i = 0; i < width; ++i) {
    this->expect_near(result[i], std::sqrt(data[i]));
  }
}

TYPED_TEST(OperationsScalarTest, Rsqrt)
{
  using vec_type = typename TestFixture::vec_type;
  using scalar_type = typename TestFixture::scalar_type;
  constexpr auto width = TestFixture::width;

  std::vector<scalar_type> data(width);
  for (std::size_t i = 0; i < width; ++i) {
    data[i] = static_cast<scalar_type>((i + 1) * (i + 1));
  }

  auto v = vec_type::loadu(data.data());
  auto result = rsqrt(v);

  for (std::size_t i = 0; i < width; ++i) {
    this->expect_near(result[i], scalar_type{1} / std::sqrt(data[i]));
  }
}

TYPED_TEST(OperationsScalarTest, Rcp)
{
  using vec_type = typename TestFixture::vec_type;
  using scalar_type = typename TestFixture::scalar_type;
  constexpr auto width = TestFixture::width;

  std::vector<scalar_type> data(width);
  for (std::size_t i = 0; i < width; ++i) {
    data[i] = static_cast<scalar_type>(i + 1);
  }

  auto v = vec_type::loadu(data.data());
  auto result = rcp(v);

  for (std::size_t i = 0; i < width; ++i) {
    this->expect_near(result[i], scalar_type{1} / data[i]);
  }
}

TYPED_TEST(OperationsScalarTest, MinMax)
{
  using vec_type = typename TestFixture::vec_type;
  using scalar_type = typename TestFixture::scalar_type;
  constexpr auto width = TestFixture::width;

  std::vector<scalar_type> data_a(width), data_b(width);
  for (std::size_t i = 0; i < width; ++i) {
    data_a[i] = static_cast<scalar_type>(i * 2);
    data_b[i] = static_cast<scalar_type>(width - i);
  }

  auto a = vec_type::loadu(data_a.data());
  auto b = vec_type::loadu(data_b.data());

  auto min_result = min(a, b);
  auto max_result = max(a, b);

  for (std::size_t i = 0; i < width; ++i) {
    this->expect_near(min_result[i], std::min(data_a[i], data_b[i]));
    this->expect_near(max_result[i], std::max(data_a[i], data_b[i]));
  }
}

TYPED_TEST(OperationsScalarTest, Clamp)
{
  using vec_type = typename TestFixture::vec_type;
  using scalar_type = typename TestFixture::scalar_type;
  constexpr auto width = TestFixture::width;

  std::vector<scalar_type> data(width);
  for (std::size_t i = 0; i < width; ++i) {
    data[i] = static_cast<scalar_type>(i) - scalar_type{1};
  }

  auto v = vec_type::loadu(data.data());
  auto lo = vec_type::broadcast(scalar_type{0});
  auto hi = vec_type::broadcast(static_cast<scalar_type>(width - 2));

  auto result = clamp(v, lo, hi);

  for (std::size_t i = 0; i < width; ++i) {
    scalar_type expected = std::clamp(
        data[i], scalar_type{0}, static_cast<scalar_type>(width - 2));
    this->expect_near(result[i], expected);
  }
}

TYPED_TEST(OperationsScalarTest, Fmadd)
{
  using vec_type = typename TestFixture::vec_type;
  using scalar_type = typename TestFixture::scalar_type;
  constexpr auto width = TestFixture::width;

  auto a = vec_type::broadcast(scalar_type{2});
  auto b = vec_type::broadcast(scalar_type{3});
  auto c = vec_type::broadcast(scalar_type{1});

  auto result = fmadd(a, b, c);
  for (std::size_t i = 0; i < width; ++i) {
    this->expect_near(result[i], scalar_type{7});
  }
}

TYPED_TEST(OperationsScalarTest, Fmsub)
{
  using vec_type = typename TestFixture::vec_type;
  using scalar_type = typename TestFixture::scalar_type;
  constexpr auto width = TestFixture::width;

  auto a = vec_type::broadcast(scalar_type{2});
  auto b = vec_type::broadcast(scalar_type{3});
  auto c = vec_type::broadcast(scalar_type{1});

  auto result = fmsub(a, b, c);
  for (std::size_t i = 0; i < width; ++i) {
    this->expect_near(result[i], scalar_type{5});
  }
}

TYPED_TEST(OperationsScalarTest, Fnmadd)
{
  using vec_type = typename TestFixture::vec_type;
  using scalar_type = typename TestFixture::scalar_type;
  constexpr auto width = TestFixture::width;

  auto a = vec_type::broadcast(scalar_type{2});
  auto b = vec_type::broadcast(scalar_type{3});
  auto c = vec_type::broadcast(scalar_type{10});

  auto result = fnmadd(a, b, c);
  for (std::size_t i = 0; i < width; ++i) {
    this->expect_near(result[i], scalar_type{4});
  }
}

TYPED_TEST(OperationsScalarTest, Floor)
{
  using vec_type = typename TestFixture::vec_type;
  using scalar_type = typename TestFixture::scalar_type;
  constexpr auto width = TestFixture::width;

  std::vector<scalar_type> data(width);
  for (std::size_t i = 0; i < width; ++i) {
    data[i] = static_cast<scalar_type>(i) + scalar_type{0.7};
    if (i % 2 == 1) {
      data[i] = -data[i];
    }
  }

  auto v = vec_type::loadu(data.data());
  auto result = floor(v);

  for (std::size_t i = 0; i < width; ++i) {
    this->expect_near(result[i], std::floor(data[i]));
  }
}

TYPED_TEST(OperationsScalarTest, Ceil)
{
  using vec_type = typename TestFixture::vec_type;
  using scalar_type = typename TestFixture::scalar_type;
  constexpr auto width = TestFixture::width;

  std::vector<scalar_type> data(width);
  for (std::size_t i = 0; i < width; ++i) {
    data[i] = static_cast<scalar_type>(i) + scalar_type{0.3};
    if (i % 2 == 1) {
      data[i] = -data[i];
    }
  }

  auto v = vec_type::loadu(data.data());
  auto result = ceil(v);

  for (std::size_t i = 0; i < width; ++i) {
    this->expect_near(result[i], std::ceil(data[i]));
  }
}

TYPED_TEST(OperationsScalarTest, Trunc)
{
  using vec_type = typename TestFixture::vec_type;
  using scalar_type = typename TestFixture::scalar_type;
  constexpr auto width = TestFixture::width;

  std::vector<scalar_type> data(width);
  for (std::size_t i = 0; i < width; ++i) {
    data[i] = static_cast<scalar_type>(i) + scalar_type{0.9};
    if (i % 2 == 1) {
      data[i] = -data[i];
    }
  }

  auto v = vec_type::loadu(data.data());
  auto result = trunc(v);

  for (std::size_t i = 0; i < width; ++i) {
    this->expect_near(result[i], std::trunc(data[i]));
  }
}

TYPED_TEST(OperationsScalarTest, Hsum)
{
  using vec_type = typename TestFixture::vec_type;
  using scalar_type = typename TestFixture::scalar_type;
  constexpr auto width = TestFixture::width;

  std::vector<scalar_type> data(width);
  scalar_type expected = scalar_type{0};
  for (std::size_t i = 0; i < width; ++i) {
    data[i] = static_cast<scalar_type>(i + 1);
    expected += data[i];
  }

  auto v = vec_type::loadu(data.data());
  this->expect_near(hsum(v), expected);
}

TYPED_TEST(OperationsScalarTest, Hmin)
{
  using vec_type = typename TestFixture::vec_type;
  using scalar_type = typename TestFixture::scalar_type;
  constexpr auto width = TestFixture::width;

  std::vector<scalar_type> data(width);
  for (std::size_t i = 0; i < width; ++i) {
    data[i] = static_cast<scalar_type>((i + 1) * ((i % 2 == 0) ? 1 : 2));
  }

  auto v = vec_type::loadu(data.data());
  scalar_type expected = *std::min_element(data.begin(), data.end());
  this->expect_near(hmin(v), expected);
}

TYPED_TEST(OperationsScalarTest, Hmax)
{
  using vec_type = typename TestFixture::vec_type;
  using scalar_type = typename TestFixture::scalar_type;
  constexpr auto width = TestFixture::width;

  std::vector<scalar_type> data(width);
  for (std::size_t i = 0; i < width; ++i) {
    data[i] = static_cast<scalar_type>((i + 1) * ((i % 2 == 0) ? 1 : 2));
  }

  auto v = vec_type::loadu(data.data());
  scalar_type expected = *std::max_element(data.begin(), data.end());
  this->expect_near(hmax(v), expected);
}

TYPED_TEST(OperationsScalarTest, Hprod)
{
  using vec_type = typename TestFixture::vec_type;
  using scalar_type = typename TestFixture::scalar_type;
  constexpr auto width = TestFixture::width;

  std::vector<scalar_type> data(width);
  scalar_type expected = scalar_type{1};
  for (std::size_t i = 0; i < width; ++i) {
    data[i] = static_cast<scalar_type>(i + 1);
    expected *= data[i];
  }

  auto v = vec_type::loadu(data.data());
  this->expect_near(hprod(v), expected);
}

TYPED_TEST(OperationsScalarTest, Select)
{
  using vec_type = typename TestFixture::vec_type;
  using scalar_type = typename TestFixture::scalar_type;
  constexpr auto width = TestFixture::width;

  std::vector<scalar_type> data_a(width), data_b(width), threshold_data(width);
  for (std::size_t i = 0; i < width; ++i) {
    data_a[i] = static_cast<scalar_type>(i + 1);
    data_b[i] = static_cast<scalar_type>((i + 1) * 10);
    threshold_data[i] = static_cast<scalar_type>(width / 2);
  }

  auto a = vec_type::loadu(data_a.data());
  auto b = vec_type::loadu(data_b.data());
  auto threshold = vec_type::loadu(threshold_data.data());
  auto mask = (a < threshold);

  auto result = select(mask, a, b);
  for (std::size_t i = 0; i < width; ++i) {
    bool cond = data_a[i] < threshold_data[i];
    scalar_type expected = cond ? data_a[i] : data_b[i];
    this->expect_near(result[i], expected);
  }
}

} // namespace dart::simd
