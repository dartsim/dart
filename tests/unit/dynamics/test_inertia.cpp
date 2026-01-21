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

#include <dart/dynamics/inertia.hpp>

#include <dart/math/geometry.hpp>
#include <dart/math/random.hpp>

#include <gtest/gtest.h>

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
TEST(Inertia, Transformations)
{
  const int numIter = 25;
  const double tol = 1e-9;

  for (int i = 0; i < numIter; ++i) {
    const auto mass = math::Random::uniform<double>(0.1, 10.0);
    const auto com = math::Random::uniform<Eigen::Vector3d>(-1.0, 1.0);
    const auto moment
        = Eigen::Matrix3d::Identity() * math::Random::uniform<double>(0.1, 2.0);
    dynamics::Inertia inertia(mass, com, moment);

    Eigen::Vector3d axis = math::Random::uniform<Eigen::Vector3d>(-1.0, 1.0);
    if (axis.norm() < 1e-6) {
      axis = Eigen::Vector3d::UnitX();
    }
    axis.normalize();
    const double angle = math::Random::uniform<double>(-3.14, 3.14);
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.linear() = Eigen::AngleAxisd(angle, axis).toRotationMatrix();
    T.translation() = math::Random::uniform<Eigen::Vector3d>(-0.5, 0.5);

    const dynamics::Inertia transformed = inertia.transformed(T);
    dynamics::Inertia inPlace = inertia;
    dynamics::Inertia& inPlaceRef = inPlace.transform(T);
    EXPECT_EQ(&inPlaceRef, &inPlace);

    const Eigen::Matrix6d adjoint = math::getAdTMatrix(T.inverse());
    const Eigen::Matrix6d expected
        = adjoint.transpose() * inertia.getSpatialTensor() * adjoint;
    EXPECT_TRUE(expected.isApprox(transformed.getSpatialTensor(), tol));
    EXPECT_TRUE(expected.isApprox(inPlace.getSpatialTensor(), tol));
    EXPECT_NEAR(transformed.getMass(), inertia.getMass(), tol);
    EXPECT_NEAR(inPlace.getMass(), inertia.getMass(), tol);

    const Eigen::Vector3d expectedCom
        = T.linear() * inertia.getLocalCOM() + T.translation();
    EXPECT_TRUE(transformed.getLocalCOM().isApprox(expectedCom, tol));
    EXPECT_TRUE(inPlace.getLocalCOM().isApprox(expectedCom, tol));

    const dynamics::Inertia identityTransformed
        = inertia.transformed(Eigen::Isometry3d::Identity());
    EXPECT_TRUE(identityTransformed.getSpatialTensor().isApprox(
        inertia.getSpatialTensor(), tol));
  }
}
