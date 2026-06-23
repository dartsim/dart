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

#include "dart/math/Constants.hpp"
#include "dart/math/Helpers.hpp"

#include <gtest/gtest.h>

using namespace dart;
using namespace dart::math::suffixes;

//==============================================================================
TEST(Helpers, AngleLiteralOperatorsFloat)
{
  // The floating-point (long double) overloads of the user-defined literal
  // operators.
  EXPECT_DOUBLE_EQ(1.0_pi, math::constantsd::pi());
  EXPECT_DOUBLE_EQ(0.5_pi, math::constantsd::pi() / 2.0);
  EXPECT_DOUBLE_EQ(2.0_pi, 2.0 * math::constantsd::pi());

  EXPECT_DOUBLE_EQ(1.5_rad, 1.5);
  EXPECT_DOUBLE_EQ(0.0_rad, 0.0);

  EXPECT_DOUBLE_EQ(90.0_deg, math::constantsd::pi() / 2.0);
  EXPECT_DOUBLE_EQ(180.0_deg, math::constantsd::pi());
  EXPECT_DOUBLE_EQ(0.0_deg, 0.0);
}

//==============================================================================
TEST(Helpers, AngleLiteralOperatorsInteger)
{
  // The integer (unsigned long long int) overloads of the user-defined literal
  // operators. These forward to the long double overloads, so the results must
  // match the floating-point equivalents exactly.
  EXPECT_DOUBLE_EQ(0_pi, 0.0);
  EXPECT_DOUBLE_EQ(1_pi, math::constantsd::pi());
  EXPECT_DOUBLE_EQ(2_pi, 2.0 * math::constantsd::pi());
  EXPECT_DOUBLE_EQ(1_pi, 1.0_pi);

  EXPECT_DOUBLE_EQ(0_rad, 0.0);
  EXPECT_DOUBLE_EQ(2_rad, 2.0);
  EXPECT_DOUBLE_EQ(2_rad, 2.0_rad);

  EXPECT_DOUBLE_EQ(0_deg, 0.0);
  EXPECT_DOUBLE_EQ(90_deg, math::constantsd::pi() / 2.0);
  EXPECT_DOUBLE_EQ(180_deg, math::constantsd::pi());
  EXPECT_DOUBLE_EQ(360_deg, 2.0 * math::constantsd::pi());
  EXPECT_DOUBLE_EQ(90_deg, 90.0_deg);
}
