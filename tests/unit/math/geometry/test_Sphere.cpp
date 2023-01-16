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

#include "dart/math/math.hpp"
#include "dart/test/math/GTestUtils.hpp"

#include <gtest/gtest.h>

using namespace dart;
using namespace math;

template <typename S>
struct SphereTest : public testing::Test
{
  using Scalar = S;
};

using Types = testing::Types<float, double>;
TYPED_TEST_SUITE(SphereTest, Types);

//==============================================================================
TYPED_TEST(SphereTest, ComputeSurfaceArea)
{
  using S = typename TestFixture::Scalar;

  S radius = 2.0;
  S expected_surface_area = 4 * pi<S>() * radius * radius;
  S surface_area = Sphere<S>::ComputeSurfaceArea(radius);

  EXPECT_NEAR(surface_area, expected_surface_area, 1e-4);
}

//==============================================================================
TYPED_TEST(SphereTest, ComputeVolume)
{
  using S = typename TestFixture::Scalar;

  S radius = 2.0;
  S expected_volume = 33.5103;
  S volume = Sphere<S>::ComputeVolume(radius);

  EXPECT_NEAR(volume, expected_volume, 1e-4);
}

//==============================================================================
TYPED_TEST(SphereTest, ComputeInertiaFromMass)
{
  using S = typename TestFixture::Scalar;

  S radius = 2.0;
  S mass = 2.0;
  Matrix3<S> expected_inertia;
  expected_inertia << (2.0 / 5.0) * mass * radius * radius, 0.0, 0.0, 0.0,
      (2.0 / 5.0) * mass * radius * radius, 0.0, 0.0, 0.0,
      (2.0 / 5.0) * mass * radius * radius;
  Matrix3<S> inertia = Sphere<S>::ComputeInertiaFromMass(radius, mass);

  EXPECT_TRUE(inertia.isApprox(expected_inertia))
      << "inertia: \n"
      << inertia << "\nexpected_inertia: \n"
      << expected_inertia;
}

//==============================================================================
TYPED_TEST(SphereTest, ComputeInertiaFromDensity)
{
  using S = typename TestFixture::Scalar;

  S radius = 2.0;
  S density = 2.0;
  S mass = density * (4.0 / 3.0) * pi<S>() * radius * radius * radius;
  Matrix3<S> expected_inertia;
  expected_inertia << (2.0 / 5.0) * mass * radius * radius, 0.0, 0.0, 0.0,
      (2.0 / 5.0) * mass * radius * radius, 0.0, 0.0, 0.0,
      (2.0 / 5.0) * mass * radius * radius;
  Matrix3<S> inertia = Sphere<S>::ComputeInertiaFromDensity(radius, density);

  EXPECT_TRUE(inertia.isApprox(expected_inertia));
}

//==============================================================================
TYPED_TEST(SphereTest, DefaultConstructor)
{
  using S = typename TestFixture::Scalar;

  Sphere<S> sphere;
  EXPECT_S_EQ(sphere.getRadius(), 0.5);
}

//==============================================================================
TYPED_TEST(SphereTest, GetVolume)
{
  using S = typename TestFixture::Scalar;

  S radius = 2.0;
  Sphere<S> sphere(radius);
  S expected_volume = 33.5103;
  S volume = sphere.getVolume();

  EXPECT_NEAR(volume, expected_volume, 1e-4);
}
