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

#include <dart/math/constants.hpp>

#include <dart/simd/simd.hpp>

#include <gtest/gtest.h>

#include <limits>
#include <random>
#include <vector>

#include <cmath>

namespace dart::simd {

// =============================================================================
// Test Fixture
// =============================================================================

template <typename VecType>
class MathTest : public ::testing::Test
{
protected:
  using vec_type = VecType;
  using scalar_type = typename VecType::scalar_type;
  using mask_type = typename VecType::mask_type;
  static constexpr std::size_t width = VecType::width;

  // Tolerance for transcendental functions (looser than basic ops)
  static constexpr scalar_type tolerance()
  {
    if constexpr (std::is_same_v<scalar_type, float>) {
      return 1e-5f;
    } else {
      return 1e-12;
    }
  }

  // Even looser tolerance for functions with higher error accumulation
  static constexpr scalar_type looseTolerance()
  {
    if constexpr (std::is_same_v<scalar_type, float>) {
      return 1e-4f;
    } else {
      return 1e-10;
    }
  }

  void expectNear(scalar_type a, scalar_type b, scalar_type tol = tolerance())
  {
    if (std::isnan(a) && std::isnan(b)) {
      return; // Both NaN is considered equal
    }
    if (std::isinf(a) && std::isinf(b)) {
      EXPECT_EQ(std::signbit(a), std::signbit(b));
      return;
    }
    EXPECT_NEAR(a, b, tol);
  }

  // Generate random values in a given range
  std::vector<scalar_type> randomValues(
      std::size_t count, scalar_type lo, scalar_type hi, int seed = 42)
  {
    std::mt19937 gen(seed);
    std::uniform_real_distribution<scalar_type> dist(lo, hi);
    std::vector<scalar_type> values(count);
    for (auto& v : values) {
      v = dist(gen);
    }
    return values;
  }
};

using FloatVecTypes = ::testing::
    Types<Vec<float, 4>, Vec<double, 4>, Vec<float, 8>, Vec<double, 2>>;

TYPED_TEST_SUITE(MathTest, FloatVecTypes);

// =============================================================================
// Sign manipulation tests
// =============================================================================

TYPED_TEST(MathTest, Sign)
{
  using vec_type = typename TestFixture::vec_type;
  using scalar_type = typename TestFixture::scalar_type;
  constexpr auto width = TestFixture::width;

  std::vector<scalar_type> data(width);
  for (std::size_t i = 0; i < width; ++i) {
    data[i] = static_cast<scalar_type>(
        (i % 3 == 0)   ? -(i + 1)
        : (i % 3 == 1) ? (i + 1)
                       : 0);
  }

  auto v = vec_type::loadu(data.data());
  auto result = sign(v);

  for (std::size_t i = 0; i < width; ++i) {
    scalar_type expected
        = (data[i] > 0) ? scalar_type{1}
                        : ((data[i] < 0) ? scalar_type{-1} : scalar_type{0});
    this->expectNear(result[i], expected);
  }
}

TYPED_TEST(MathTest, Copysign)
{
  using vec_type = typename TestFixture::vec_type;
  using scalar_type = typename TestFixture::scalar_type;
  constexpr auto width = TestFixture::width;

  std::vector<scalar_type> magnitudes(width), signs(width);
  for (std::size_t i = 0; i < width; ++i) {
    magnitudes[i] = static_cast<scalar_type>(i + 1);
    signs[i] = (i % 2 == 0) ? scalar_type{-1} : scalar_type{1};
  }

  auto mag = vec_type::loadu(magnitudes.data());
  auto sgn = vec_type::loadu(signs.data());
  auto result = copysign(mag, sgn);

  for (std::size_t i = 0; i < width; ++i) {
    this->expectNear(result[i], std::copysign(magnitudes[i], signs[i]));
  }
}

TYPED_TEST(MathTest, Mulsign)
{
  using vec_type = typename TestFixture::vec_type;
  using scalar_type = typename TestFixture::scalar_type;
  constexpr auto width = TestFixture::width;

  std::vector<scalar_type> values(width), signs(width);
  for (std::size_t i = 0; i < width; ++i) {
    values[i] = static_cast<scalar_type>(i % 2 == 0 ? i + 1 : -(i + 1));
    signs[i] = static_cast<scalar_type>(i % 3 == 0 ? -1 : 1);
  }

  auto val = vec_type::loadu(values.data());
  auto sgn = vec_type::loadu(signs.data());
  auto result = mulsign(val, sgn);

  for (std::size_t i = 0; i < width; ++i) {
    scalar_type expected = std::copysign(values[i], values[i] * signs[i]);
    this->expectNear(result[i], expected);
  }
}

// =============================================================================
// FP Classification tests
// =============================================================================

TYPED_TEST(MathTest, IsNaN)
{
  using vec_type = typename TestFixture::vec_type;
  using scalar_type = typename TestFixture::scalar_type;
  constexpr auto width = TestFixture::width;

  std::vector<scalar_type> data(width);
  for (std::size_t i = 0; i < width; ++i) {
    if (i % 3 == 0) {
      data[i] = std::numeric_limits<scalar_type>::quiet_NaN();
    } else {
      data[i] = static_cast<scalar_type>(i);
    }
  }

  auto v = vec_type::loadu(data.data());
  auto result = isnan(v);

  for (std::size_t i = 0; i < width; ++i) {
    EXPECT_EQ(result[i], std::isnan(data[i]));
  }
}

TYPED_TEST(MathTest, IsInf)
{
  using vec_type = typename TestFixture::vec_type;
  using scalar_type = typename TestFixture::scalar_type;
  constexpr auto width = TestFixture::width;

  std::vector<scalar_type> data(width);
  for (std::size_t i = 0; i < width; ++i) {
    if (i % 4 == 0) {
      data[i] = std::numeric_limits<scalar_type>::infinity();
    } else if (i % 4 == 1) {
      data[i] = -std::numeric_limits<scalar_type>::infinity();
    } else {
      data[i] = static_cast<scalar_type>(i);
    }
  }

  auto v = vec_type::loadu(data.data());
  auto result = isinf(v);

  for (std::size_t i = 0; i < width; ++i) {
    EXPECT_EQ(result[i], std::isinf(data[i]));
  }
}

TYPED_TEST(MathTest, IsFinite)
{
  using vec_type = typename TestFixture::vec_type;
  using scalar_type = typename TestFixture::scalar_type;
  constexpr auto width = TestFixture::width;

  std::vector<scalar_type> data(width);
  for (std::size_t i = 0; i < width; ++i) {
    if (i % 3 == 0) {
      data[i] = std::numeric_limits<scalar_type>::quiet_NaN();
    } else if (i % 3 == 1) {
      data[i] = std::numeric_limits<scalar_type>::infinity();
    } else {
      data[i] = static_cast<scalar_type>(i);
    }
  }

  auto v = vec_type::loadu(data.data());
  auto result = isfinite(v);

  for (std::size_t i = 0; i < width; ++i) {
    EXPECT_EQ(result[i], std::isfinite(data[i]));
  }
}

// =============================================================================
// Exponential and logarithmic tests
// =============================================================================

TYPED_TEST(MathTest, Exp)
{
  using vec_type = typename TestFixture::vec_type;
  using scalar_type = typename TestFixture::scalar_type;
  constexpr auto width = TestFixture::width;

  auto data = this->randomValues(width, scalar_type{-5}, scalar_type{5});
  auto v = vec_type::loadu(data.data());
  auto result = exp(v);

  for (std::size_t i = 0; i < width; ++i) {
    this->expectNear(result[i], std::exp(data[i]));
  }
}

TYPED_TEST(MathTest, Exp2)
{
  using vec_type = typename TestFixture::vec_type;
  using scalar_type = typename TestFixture::scalar_type;
  constexpr auto width = TestFixture::width;

  auto data = this->randomValues(width, scalar_type{-10}, scalar_type{10});
  auto v = vec_type::loadu(data.data());
  auto result = exp2(v);

  for (std::size_t i = 0; i < width; ++i) {
    this->expectNear(result[i], std::exp2(data[i]));
  }
}

TYPED_TEST(MathTest, Log)
{
  using vec_type = typename TestFixture::vec_type;
  using scalar_type = typename TestFixture::scalar_type;
  constexpr auto width = TestFixture::width;

  auto data = this->randomValues(width, scalar_type{0.01}, scalar_type{100});
  auto v = vec_type::loadu(data.data());
  auto result = log(v);

  for (std::size_t i = 0; i < width; ++i) {
    this->expectNear(result[i], std::log(data[i]));
  }
}

TYPED_TEST(MathTest, Log2)
{
  using vec_type = typename TestFixture::vec_type;
  using scalar_type = typename TestFixture::scalar_type;
  constexpr auto width = TestFixture::width;

  auto data = this->randomValues(width, scalar_type{0.01}, scalar_type{100});
  auto v = vec_type::loadu(data.data());
  auto result = log2(v);

  for (std::size_t i = 0; i < width; ++i) {
    this->expectNear(result[i], std::log2(data[i]));
  }
}

TYPED_TEST(MathTest, Log10)
{
  using vec_type = typename TestFixture::vec_type;
  using scalar_type = typename TestFixture::scalar_type;
  constexpr auto width = TestFixture::width;

  auto data = this->randomValues(width, scalar_type{0.01}, scalar_type{100});
  auto v = vec_type::loadu(data.data());
  auto result = log10(v);

  for (std::size_t i = 0; i < width; ++i) {
    this->expectNear(result[i], std::log10(data[i]));
  }
}

TYPED_TEST(MathTest, Pow)
{
  using vec_type = typename TestFixture::vec_type;
  using scalar_type = typename TestFixture::scalar_type;
  constexpr auto width = TestFixture::width;

  auto bases = this->randomValues(width, scalar_type{0.5}, scalar_type{5}, 1);
  auto exponents
      = this->randomValues(width, scalar_type{-2}, scalar_type{2}, 2);

  auto base = vec_type::loadu(bases.data());
  auto exp = vec_type::loadu(exponents.data());
  auto result = pow(base, exp);

  for (std::size_t i = 0; i < width; ++i) {
    this->expectNear(
        result[i], std::pow(bases[i], exponents[i]), this->looseTolerance());
  }
}

TYPED_TEST(MathTest, Cbrt)
{
  using vec_type = typename TestFixture::vec_type;
  using scalar_type = typename TestFixture::scalar_type;
  constexpr auto width = TestFixture::width;

  auto data = this->randomValues(width, scalar_type{-100}, scalar_type{100});
  auto v = vec_type::loadu(data.data());
  auto result = cbrt(v);

  for (std::size_t i = 0; i < width; ++i) {
    this->expectNear(result[i], std::cbrt(data[i]));
  }
}

// =============================================================================
// Trigonometric tests
// =============================================================================

TYPED_TEST(MathTest, Sin)
{
  using vec_type = typename TestFixture::vec_type;
  using scalar_type = typename TestFixture::scalar_type;
  constexpr auto width = TestFixture::width;

  constexpr auto pi = dart::math::pi_v<scalar_type>;
  auto data
      = this->randomValues(width, -scalar_type{10} * pi, scalar_type{10} * pi);
  auto v = vec_type::loadu(data.data());
  auto result = sin(v);

  for (std::size_t i = 0; i < width; ++i) {
    this->expectNear(result[i], std::sin(data[i]));
  }
}

TYPED_TEST(MathTest, Cos)
{
  using vec_type = typename TestFixture::vec_type;
  using scalar_type = typename TestFixture::scalar_type;
  constexpr auto width = TestFixture::width;

  constexpr auto pi = dart::math::pi_v<scalar_type>;
  auto data
      = this->randomValues(width, -scalar_type{10} * pi, scalar_type{10} * pi);
  auto v = vec_type::loadu(data.data());
  auto result = cos(v);

  for (std::size_t i = 0; i < width; ++i) {
    this->expectNear(result[i], std::cos(data[i]));
  }
}

TYPED_TEST(MathTest, SinCos)
{
  using vec_type = typename TestFixture::vec_type;
  using scalar_type = typename TestFixture::scalar_type;
  constexpr auto width = TestFixture::width;

  constexpr auto pi = dart::math::pi_v<scalar_type>;
  auto data
      = this->randomValues(width, -scalar_type{10} * pi, scalar_type{10} * pi);
  auto v = vec_type::loadu(data.data());
  auto [s, c] = sincos(v);

  for (std::size_t i = 0; i < width; ++i) {
    this->expectNear(s[i], std::sin(data[i]));
    this->expectNear(c[i], std::cos(data[i]));
  }
}

TYPED_TEST(MathTest, Tan)
{
  using vec_type = typename TestFixture::vec_type;
  using scalar_type = typename TestFixture::scalar_type;
  constexpr auto width = TestFixture::width;

  // Avoid values near pi/2 where tan approaches infinity
  auto data = this->randomValues(width, scalar_type{-1.5}, scalar_type{1.5});
  auto v = vec_type::loadu(data.data());
  auto result = tan(v);

  for (std::size_t i = 0; i < width; ++i) {
    this->expectNear(result[i], std::tan(data[i]), this->looseTolerance());
  }
}

// =============================================================================
// Inverse trigonometric tests
// =============================================================================

TYPED_TEST(MathTest, Asin)
{
  using vec_type = typename TestFixture::vec_type;
  using scalar_type = typename TestFixture::scalar_type;
  constexpr auto width = TestFixture::width;

  // asin domain: [-1, 1]
  auto data = this->randomValues(width, scalar_type{-0.99}, scalar_type{0.99});
  auto v = vec_type::loadu(data.data());
  auto result = asin(v);

  for (std::size_t i = 0; i < width; ++i) {
    this->expectNear(result[i], std::asin(data[i]));
  }
}

TYPED_TEST(MathTest, Acos)
{
  using vec_type = typename TestFixture::vec_type;
  using scalar_type = typename TestFixture::scalar_type;
  constexpr auto width = TestFixture::width;

  // acos domain: [-1, 1]
  auto data = this->randomValues(width, scalar_type{-0.99}, scalar_type{0.99});
  auto v = vec_type::loadu(data.data());
  auto result = acos(v);

  for (std::size_t i = 0; i < width; ++i) {
    this->expectNear(result[i], std::acos(data[i]));
  }
}

TYPED_TEST(MathTest, Atan)
{
  using vec_type = typename TestFixture::vec_type;
  using scalar_type = typename TestFixture::scalar_type;
  constexpr auto width = TestFixture::width;

  auto data = this->randomValues(width, scalar_type{-10}, scalar_type{10});
  auto v = vec_type::loadu(data.data());
  auto result = atan(v);

  for (std::size_t i = 0; i < width; ++i) {
    this->expectNear(result[i], std::atan(data[i]));
  }
}

TYPED_TEST(MathTest, Atan2)
{
  using vec_type = typename TestFixture::vec_type;
  using scalar_type = typename TestFixture::scalar_type;
  constexpr auto width = TestFixture::width;

  auto yData = this->randomValues(width, scalar_type{-10}, scalar_type{10}, 1);
  auto xData = this->randomValues(width, scalar_type{-10}, scalar_type{10}, 2);

  auto y = vec_type::loadu(yData.data());
  auto x = vec_type::loadu(xData.data());
  auto result = atan2(y, x);

  for (std::size_t i = 0; i < width; ++i) {
    this->expectNear(result[i], std::atan2(yData[i], xData[i]));
  }
}

// =============================================================================
// Hyperbolic tests
// =============================================================================

TYPED_TEST(MathTest, Sinh)
{
  using vec_type = typename TestFixture::vec_type;
  using scalar_type = typename TestFixture::scalar_type;
  constexpr auto width = TestFixture::width;

  auto data = this->randomValues(width, scalar_type{-5}, scalar_type{5});
  auto v = vec_type::loadu(data.data());
  auto result = sinh(v);

  for (std::size_t i = 0; i < width; ++i) {
    this->expectNear(result[i], std::sinh(data[i]));
  }
}

TYPED_TEST(MathTest, Cosh)
{
  using vec_type = typename TestFixture::vec_type;
  using scalar_type = typename TestFixture::scalar_type;
  constexpr auto width = TestFixture::width;

  auto data = this->randomValues(width, scalar_type{-5}, scalar_type{5});
  auto v = vec_type::loadu(data.data());
  auto result = cosh(v);

  for (std::size_t i = 0; i < width; ++i) {
    this->expectNear(result[i], std::cosh(data[i]));
  }
}

TYPED_TEST(MathTest, Tanh)
{
  using vec_type = typename TestFixture::vec_type;
  using scalar_type = typename TestFixture::scalar_type;
  constexpr auto width = TestFixture::width;

  auto data = this->randomValues(width, scalar_type{-5}, scalar_type{5});
  auto v = vec_type::loadu(data.data());
  auto result = tanh(v);

  for (std::size_t i = 0; i < width; ++i) {
    this->expectNear(result[i], std::tanh(data[i]));
  }
}

TYPED_TEST(MathTest, SinCosh)
{
  using vec_type = typename TestFixture::vec_type;
  using scalar_type = typename TestFixture::scalar_type;
  constexpr auto width = TestFixture::width;

  auto data = this->randomValues(width, scalar_type{-5}, scalar_type{5});
  auto v = vec_type::loadu(data.data());
  auto [s, c] = sincosh(v);

  for (std::size_t i = 0; i < width; ++i) {
    this->expectNear(s[i], std::sinh(data[i]));
    this->expectNear(c[i], std::cosh(data[i]));
  }
}

// =============================================================================
// Inverse hyperbolic tests
// =============================================================================

TYPED_TEST(MathTest, Asinh)
{
  using vec_type = typename TestFixture::vec_type;
  using scalar_type = typename TestFixture::scalar_type;
  constexpr auto width = TestFixture::width;

  auto data = this->randomValues(width, scalar_type{-100}, scalar_type{100});
  auto v = vec_type::loadu(data.data());
  auto result = asinh(v);

  for (std::size_t i = 0; i < width; ++i) {
    this->expectNear(result[i], std::asinh(data[i]));
  }
}

TYPED_TEST(MathTest, Acosh)
{
  using vec_type = typename TestFixture::vec_type;
  using scalar_type = typename TestFixture::scalar_type;
  constexpr auto width = TestFixture::width;

  // acosh domain: [1, inf)
  auto data = this->randomValues(width, scalar_type{1.01}, scalar_type{100});
  auto v = vec_type::loadu(data.data());
  auto result = acosh(v);

  for (std::size_t i = 0; i < width; ++i) {
    this->expectNear(result[i], std::acosh(data[i]));
  }
}

TYPED_TEST(MathTest, Atanh)
{
  using vec_type = typename TestFixture::vec_type;
  using scalar_type = typename TestFixture::scalar_type;
  constexpr auto width = TestFixture::width;

  // atanh domain: (-1, 1)
  auto data = this->randomValues(width, scalar_type{-0.99}, scalar_type{0.99});
  auto v = vec_type::loadu(data.data());
  auto result = atanh(v);

  for (std::size_t i = 0; i < width; ++i) {
    this->expectNear(result[i], std::atanh(data[i]));
  }
}

// =============================================================================
// Special functions tests
// =============================================================================

TYPED_TEST(MathTest, Erf)
{
  using vec_type = typename TestFixture::vec_type;
  using scalar_type = typename TestFixture::scalar_type;
  constexpr auto width = TestFixture::width;

  auto data = this->randomValues(width, scalar_type{-3}, scalar_type{3});
  auto v = vec_type::loadu(data.data());
  auto result = erf(v);

  for (std::size_t i = 0; i < width; ++i) {
    this->expectNear(result[i], std::erf(data[i]));
  }
}

// =============================================================================
// Mantissa/exponent manipulation tests
// =============================================================================

TYPED_TEST(MathTest, Frexp)
{
  using vec_type = typename TestFixture::vec_type;
  using scalar_type = typename TestFixture::scalar_type;
  constexpr auto width = TestFixture::width;

  auto data = this->randomValues(width, scalar_type{0.1}, scalar_type{1000});
  auto v = vec_type::loadu(data.data());
  auto [mantissa, exponent] = frexp(v);

  for (std::size_t i = 0; i < width; ++i) {
    int exp;
    scalar_type expectedMantissa = std::frexp(data[i], &exp);
    this->expectNear(mantissa[i], expectedMantissa);
    this->expectNear(exponent[i], static_cast<scalar_type>(exp));
  }
}

TYPED_TEST(MathTest, Ldexp)
{
  using vec_type = typename TestFixture::vec_type;
  using scalar_type = typename TestFixture::scalar_type;
  constexpr auto width = TestFixture::width;

  auto mantissas
      = this->randomValues(width, scalar_type{0.5}, scalar_type{1.0}, 1);
  auto exponents
      = this->randomValues(width, scalar_type{-10}, scalar_type{10}, 2);

  auto man = vec_type::loadu(mantissas.data());
  auto exp = vec_type::loadu(exponents.data());
  auto result = ldexp(man, exp);

  for (std::size_t i = 0; i < width; ++i) {
    scalar_type expected
        = std::ldexp(mantissas[i], static_cast<int>(exponents[i]));
    this->expectNear(result[i], expected);
  }
}

// =============================================================================
// Edge case tests
// =============================================================================

TYPED_TEST(MathTest, ExpEdgeCases)
{
  using vec_type = typename TestFixture::vec_type;
  using scalar_type = typename TestFixture::scalar_type;
  constexpr auto width = TestFixture::width;

  std::vector<scalar_type> data(width);
  data[0] = scalar_type{0}; // exp(0) = 1
  data[1] = scalar_type{1}; // exp(1) = e
  if (width > 2) {
    data[2] = -std::numeric_limits<scalar_type>::infinity(); // exp(-inf) = 0
  }
  if (width > 3) {
    data[3] = std::numeric_limits<scalar_type>::infinity(); // exp(inf) = inf
  }
  for (std::size_t i = 4; i < width; ++i) {
    data[i] = scalar_type{0};
  }

  auto v = vec_type::loadu(data.data());
  auto result = exp(v);

  this->expectNear(result[0], scalar_type{1});
  this->expectNear(result[1], std::exp(scalar_type{1}));
  if (width > 2) {
    this->expectNear(result[2], scalar_type{0});
  }
  if (width > 3) {
    EXPECT_TRUE(std::isinf(result[3]) && result[3] > 0);
  }
}

TYPED_TEST(MathTest, LogEdgeCases)
{
  using vec_type = typename TestFixture::vec_type;
  using scalar_type = typename TestFixture::scalar_type;
  constexpr auto width = TestFixture::width;

  std::vector<scalar_type> data(width);
  data[0] = scalar_type{1};           // log(1) = 0
  data[1] = std::exp(scalar_type{1}); // log(e) = 1
  if (width > 2) {
    data[2] = std::numeric_limits<scalar_type>::infinity(); // log(inf) = inf
  }
  if (width > 3) {
    data[3] = scalar_type{0}; // log(0) = -inf
  }
  for (std::size_t i = 4; i < width; ++i) {
    data[i] = scalar_type{1};
  }

  auto v = vec_type::loadu(data.data());
  auto result = log(v);

  this->expectNear(result[0], scalar_type{0});
  this->expectNear(result[1], scalar_type{1});
  if (width > 2) {
    EXPECT_TRUE(std::isinf(result[2]) && result[2] > 0);
  }
  if (width > 3) {
    EXPECT_TRUE(std::isinf(result[3]) && result[3] < 0);
  }
}

TYPED_TEST(MathTest, TrigEdgeCases)
{
  using vec_type = typename TestFixture::vec_type;
  using scalar_type = typename TestFixture::scalar_type;
  constexpr auto width = TestFixture::width;

  constexpr auto pi = dart::math::pi_v<scalar_type>;
  std::vector<scalar_type> data(width);
  data[0] = scalar_type{0};                   // sin(0) = 0, cos(0) = 1
  data[1] = static_cast<scalar_type>(pi / 2); // sin(pi/2) = 1, cos(pi/2) = 0
  if (width > 2) {
    data[2] = static_cast<scalar_type>(pi); // sin(pi) = 0, cos(pi) = -1
  }
  if (width > 3) {
    data[3] = static_cast<scalar_type>(3 * pi / 2); // sin = -1, cos = 0
  }
  for (std::size_t i = 4; i < width; ++i) {
    data[i] = scalar_type{0};
  }

  auto v = vec_type::loadu(data.data());
  auto sinResult = sin(v);
  auto cosResult = cos(v);

  this->expectNear(sinResult[0], scalar_type{0});
  this->expectNear(cosResult[0], scalar_type{1});
  this->expectNear(sinResult[1], scalar_type{1}, this->looseTolerance());
  this->expectNear(cosResult[1], scalar_type{0}, this->looseTolerance());
  if (width > 2) {
    this->expectNear(sinResult[2], scalar_type{0}, this->looseTolerance());
    this->expectNear(cosResult[2], scalar_type{-1}, this->looseTolerance());
  }
}

// =============================================================================
// Accuracy bounds tests (sanity checks for transcendental function errors)
// =============================================================================

TEST(AccuracyTest, SinCosConsistency)
{
  // Verify sin^2(x) + cos^2(x) = 1 for various x values
  using vec_type = Vec<float, 4>;
  std::vector<float> data = {0.0f, 0.5f, 1.0f, 2.0f};
  auto v = vec_type::loadu(data.data());
  auto [s, c] = sincos(v);
  auto identity = s * s + c * c;

  for (std::size_t i = 0; i < 4; ++i) {
    EXPECT_NEAR(identity[i], 1.0f, 1e-5f)
        << "sin^2(x) + cos^2(x) != 1 at x=" << data[i];
  }
}

TEST(AccuracyTest, ExpLogInverse)
{
  // Verify log(exp(x)) = x for values in a safe range
  using vec_type = Vec<float, 4>;
  std::vector<float> data = {0.0f, 1.0f, 2.0f, 3.0f};
  auto v = vec_type::loadu(data.data());
  auto expV = exp(v);
  auto logExpV = log(expV);

  for (std::size_t i = 0; i < 4; ++i) {
    EXPECT_NEAR(logExpV[i], data[i], 1e-5f)
        << "log(exp(x)) != x at x=" << data[i];
  }
}

TEST(AccuracyTest, SinhCoshConsistency)
{
  // Verify cosh^2(x) - sinh^2(x) = 1 for various x values
  using vec_type = Vec<float, 4>;
  std::vector<float> data = {0.0f, 0.5f, 1.0f, 2.0f};
  auto v = vec_type::loadu(data.data());
  auto [sh, ch] = sincosh(v);
  auto identity = ch * ch - sh * sh;

  for (std::size_t i = 0; i < 4; ++i) {
    EXPECT_NEAR(identity[i], 1.0f, 1e-4f)
        << "cosh^2(x) - sinh^2(x) != 1 at x=" << data[i];
  }
}

} // namespace dart::simd
