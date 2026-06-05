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

#include "TestHelpers.hpp"

#include <dart/dynamics/Inertia.hpp>

#include <dart/math/Random.hpp>

#include <gtest/gtest.h>

#include <limits>

using namespace dart;

//==============================================================================
TEST(Inertia, Verification)
{
  const int numIter = 10;

  for (int i = 0; i < numIter; ++i) {
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

//==============================================================================
// A non-finite moment of inertia passed to the constructor must leave the
// object in a valid (finite) state. setMoment() rejects the non-finite matrix
// and returns early, so the constructor must first establish a valid default
// rather than leaving mMoment / the spatial tensor indeterminate.
// See https://github.com/gazebosim/gz-physics/issues/854
TEST(Inertia, NonFiniteMomentConstructorLeavesValidState)
{
  Eigen::Matrix3d nanMoment = Eigen::Matrix3d::Identity();
  nanMoment(0, 0) = std::numeric_limits<double>::quiet_NaN();
  const dynamics::Inertia nanInertia(1.0, Eigen::Vector3d::Zero(), nanMoment);
  EXPECT_TRUE(nanInertia.getMoment().allFinite());
  EXPECT_TRUE(nanInertia.getSpatialTensor().allFinite());

  Eigen::Matrix3d infMoment = Eigen::Matrix3d::Identity();
  infMoment(2, 2) = std::numeric_limits<double>::infinity();
  const dynamics::Inertia infInertia(1.0, Eigen::Vector3d::Zero(), infMoment);
  EXPECT_TRUE(infInertia.getMoment().allFinite());
  EXPECT_TRUE(infInertia.getSpatialTensor().allFinite());
}
