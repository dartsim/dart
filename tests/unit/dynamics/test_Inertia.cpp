/*
 * Copyright (c) 2011-2021, The DART development contributors
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

#include <dart/dynamics/Inertia.hpp>
#include <dart/math/Random.hpp>
#include <gtest/gtest.h>
#include "dart/test/TestHelpers.hpp"

using namespace dart;

//==============================================================================
TEST(Inertia, Verification)
{
  const int numIter = 10;

  for (int i = 0; i < numIter; ++i)
  {
    const auto mass = math::Random::uniform<double>(0.1, 10.0);
    const auto com = math::Random::uniform<Eigen::Vector3d>(-5, 5);
    const auto i_xx = math::Random::uniform<double>(0.1, 1);
    const auto i_yy = math::Random::uniform<double>(0.1, 1);
    const auto i_zz = math::Random::uniform<double>(0.1, 1);
    const auto i_xy = math::Random::uniform<double>(-1, 1);
    const auto i_xz = math::Random::uniform<double>(-1, 1);
    const auto i_yz = math::Random::uniform<double>(-1, 1);

    const dynamics::Inertia inertia(
        mass, com[0], com[1], com[2], i_xx, i_yy, i_zz, i_xy, i_xz, i_yz);

    EXPECT_TRUE(inertia.verify());
  }
}
