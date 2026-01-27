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

//==============================================================================
TEST(Inertia, SetAndGetParameter)
{
  dynamics::Inertia inertia;

  inertia.setParameter(dynamics::Inertia::MASS, 5.0);
  EXPECT_DOUBLE_EQ(inertia.getParameter(dynamics::Inertia::MASS), 5.0);
  EXPECT_DOUBLE_EQ(inertia.getMass(), 5.0);

  inertia.setParameter(dynamics::Inertia::COM_X, 1.0);
  inertia.setParameter(dynamics::Inertia::COM_Y, 2.0);
  inertia.setParameter(dynamics::Inertia::COM_Z, 3.0);
  EXPECT_DOUBLE_EQ(inertia.getParameter(dynamics::Inertia::COM_X), 1.0);
  EXPECT_DOUBLE_EQ(inertia.getParameter(dynamics::Inertia::COM_Y), 2.0);
  EXPECT_DOUBLE_EQ(inertia.getParameter(dynamics::Inertia::COM_Z), 3.0);
  EXPECT_TRUE(inertia.getLocalCOM().isApprox(Eigen::Vector3d(1.0, 2.0, 3.0)));
}

//==============================================================================
TEST(Inertia, SetMomentWithScalars)
{
  dynamics::Inertia inertia;

  inertia.setMoment(1.0, 2.0, 3.0, 0.1, 0.2, 0.3);

  Eigen::Matrix3d moment = inertia.getMoment();
  EXPECT_DOUBLE_EQ(moment(0, 0), 1.0);
  EXPECT_DOUBLE_EQ(moment(1, 1), 2.0);
  EXPECT_DOUBLE_EQ(moment(2, 2), 3.0);
  EXPECT_DOUBLE_EQ(moment(0, 1), 0.1);
  EXPECT_DOUBLE_EQ(moment(0, 2), 0.2);
  EXPECT_DOUBLE_EQ(moment(1, 2), 0.3);
}

//==============================================================================
TEST(Inertia, SetSpatialTensor)
{
  Eigen::Matrix6d spatial = Eigen::Matrix6d::Zero();
  spatial(0, 0) = 1.0;
  spatial(1, 1) = 1.0;
  spatial(2, 2) = 1.0;
  spatial(3, 3) = 2.0;
  spatial(4, 4) = 2.0;
  spatial(5, 5) = 2.0;

  dynamics::Inertia inertia(spatial);

  EXPECT_DOUBLE_EQ(inertia.getMass(), 2.0);
}

//==============================================================================
TEST(Inertia, VerifyMoment)
{
  Eigen::Matrix3d validMoment = Eigen::Matrix3d::Identity();
  EXPECT_TRUE(dynamics::Inertia::verifyMoment(validMoment, false));

  Eigen::Matrix3d negativeDiag = Eigen::Matrix3d::Identity();
  negativeDiag(0, 0) = -1.0;
  EXPECT_FALSE(dynamics::Inertia::verifyMoment(negativeDiag, false));

  Eigen::Matrix3d asymmetric = Eigen::Matrix3d::Identity();
  asymmetric(0, 1) = 0.5;
  asymmetric(1, 0) = 0.6;
  EXPECT_FALSE(dynamics::Inertia::verifyMoment(asymmetric, false));
}

//==============================================================================
TEST(Inertia, VerifySpatialTensor)
{
  Eigen::Matrix6d validSpatial = Eigen::Matrix6d::Identity();
  EXPECT_TRUE(dynamics::Inertia::verifySpatialTensor(validSpatial, false));

  Eigen::Matrix6d negativeDiag = Eigen::Matrix6d::Identity();
  negativeDiag(0, 0) = -1.0;
  EXPECT_FALSE(dynamics::Inertia::verifySpatialTensor(negativeDiag, false));

  Eigen::Matrix6d asymmetricTopLeft = Eigen::Matrix6d::Identity();
  asymmetricTopLeft(0, 1) = 0.5;
  asymmetricTopLeft(1, 0) = 0.6;
  EXPECT_FALSE(
      dynamics::Inertia::verifySpatialTensor(asymmetricTopLeft, false));

  Eigen::Matrix6d nonZeroBottomRight = Eigen::Matrix6d::Identity();
  nonZeroBottomRight(4, 5) = 0.1;
  EXPECT_FALSE(
      dynamics::Inertia::verifySpatialTensor(nonZeroBottomRight, false));
}

//==============================================================================
TEST(Inertia, Equality)
{
  dynamics::Inertia inertia1(
      1.0, Eigen::Vector3d::Zero(), Eigen::Matrix3d::Identity());
  dynamics::Inertia inertia2(
      1.0, Eigen::Vector3d::Zero(), Eigen::Matrix3d::Identity());
  dynamics::Inertia inertia3(
      2.0, Eigen::Vector3d::Zero(), Eigen::Matrix3d::Identity());

  EXPECT_TRUE(inertia1 == inertia2);
  EXPECT_FALSE(inertia1 == inertia3);
}

//==============================================================================
TEST(Inertia, SetLocalCOM)
{
  dynamics::Inertia inertia;
  Eigen::Vector3d com(1.0, 2.0, 3.0);

  inertia.setLocalCOM(com);

  EXPECT_TRUE(inertia.getLocalCOM().isApprox(com));
}

//==============================================================================
TEST(Inertia, SetMass)
{
  dynamics::Inertia inertia;

  inertia.setMass(10.0);

  EXPECT_DOUBLE_EQ(inertia.getMass(), 10.0);
}

//==============================================================================
TEST(Inertia, SetMassEdgeCases)
{
  // Test very small mass
  dynamics::Inertia inertia1;
  inertia1.setMass(1e-10);
  EXPECT_DOUBLE_EQ(inertia1.getMass(), 1e-10);

  // Test very large mass
  dynamics::Inertia inertia2;
  inertia2.setMass(1e10);
  EXPECT_DOUBLE_EQ(inertia2.getMass(), 1e10);

  // Test mass update preserves COM
  dynamics::Inertia inertia3;
  Eigen::Vector3d com(1.0, 2.0, 3.0);
  inertia3.setLocalCOM(com);
  inertia3.setMass(5.0);
  EXPECT_TRUE(inertia3.getLocalCOM().isApprox(com));
  EXPECT_DOUBLE_EQ(inertia3.getMass(), 5.0);
}

//==============================================================================
TEST(Inertia, SetMomentWithMatrix)
{
  dynamics::Inertia inertia;

  Eigen::Matrix3d moment;
  moment << 2.0, 0.1, 0.2, 0.1, 3.0, 0.3, 0.2, 0.3, 4.0;

  inertia.setMoment(moment);

  Eigen::Matrix3d retrieved = inertia.getMoment();
  EXPECT_DOUBLE_EQ(retrieved(0, 0), 2.0);
  EXPECT_DOUBLE_EQ(retrieved(1, 1), 3.0);
  EXPECT_DOUBLE_EQ(retrieved(2, 2), 4.0);
  EXPECT_DOUBLE_EQ(retrieved(0, 1), 0.1);
  EXPECT_DOUBLE_EQ(retrieved(0, 2), 0.2);
  EXPECT_DOUBLE_EQ(retrieved(1, 2), 0.3);
  // Check symmetry
  EXPECT_DOUBLE_EQ(retrieved(1, 0), 0.1);
  EXPECT_DOUBLE_EQ(retrieved(2, 0), 0.2);
  EXPECT_DOUBLE_EQ(retrieved(2, 1), 0.3);
}

//==============================================================================
TEST(Inertia, GetSpatialTensorStructure)
{
  const double mass = 2.0;
  const Eigen::Vector3d com(0.0, 0.0, 0.0);
  const Eigen::Matrix3d moment = Eigen::Matrix3d::Identity() * 1.5;

  dynamics::Inertia inertia(mass, com, moment);
  const Eigen::Matrix6d& spatial = inertia.getSpatialTensor();

  // With COM at origin, spatial tensor has specific structure:
  // Top-left 3x3: moment of inertia
  // Bottom-right 3x3: mass * Identity
  // Off-diagonal blocks: zero (when COM is at origin)

  // Check bottom-right block (mass * I)
  EXPECT_DOUBLE_EQ(spatial(3, 3), mass);
  EXPECT_DOUBLE_EQ(spatial(4, 4), mass);
  EXPECT_DOUBLE_EQ(spatial(5, 5), mass);

  // Check top-left block (moment of inertia when COM at origin)
  EXPECT_DOUBLE_EQ(spatial(0, 0), 1.5);
  EXPECT_DOUBLE_EQ(spatial(1, 1), 1.5);
  EXPECT_DOUBLE_EQ(spatial(2, 2), 1.5);
}

//==============================================================================
TEST(Inertia, GetSpatialTensorWithCOM)
{
  const double mass = 3.0;
  const Eigen::Vector3d com(1.0, 0.0, 0.0);
  const Eigen::Matrix3d moment = Eigen::Matrix3d::Identity();

  dynamics::Inertia inertia(mass, com, moment);
  const Eigen::Matrix6d& spatial = inertia.getSpatialTensor();

  // Mass should still be in bottom-right diagonal
  EXPECT_DOUBLE_EQ(spatial(3, 3), mass);
  EXPECT_DOUBLE_EQ(spatial(4, 4), mass);
  EXPECT_DOUBLE_EQ(spatial(5, 5), mass);

  // Spatial tensor should be valid
  EXPECT_TRUE(inertia.verify(false));
}

//==============================================================================
TEST(Inertia, VerifyWithValidInertia)
{
  // Default inertia should be valid
  dynamics::Inertia defaultInertia;
  EXPECT_TRUE(defaultInertia.verify(false));

  // Inertia with positive mass and valid moment should be valid
  dynamics::Inertia validInertia(
      5.0, Eigen::Vector3d(1.0, 2.0, 3.0), Eigen::Matrix3d::Identity() * 2.0);
  EXPECT_TRUE(validInertia.verify(false));
}

//==============================================================================
TEST(Inertia, VerifyWithCustomTolerance)
{
  dynamics::Inertia inertia(
      1.0, Eigen::Vector3d::Zero(), Eigen::Matrix3d::Identity());

  // Should pass with default tolerance
  EXPECT_TRUE(inertia.verify(false, 1e-8));

  // Should pass with larger tolerance
  EXPECT_TRUE(inertia.verify(false, 1e-3));
}

//==============================================================================
TEST(Inertia, IdentityMomentOfInertia)
{
  // Test with identity moment of inertia
  dynamics::Inertia inertia(
      1.0, Eigen::Vector3d::Zero(), Eigen::Matrix3d::Identity());

  Eigen::Matrix3d moment = inertia.getMoment();
  EXPECT_TRUE(moment.isApprox(Eigen::Matrix3d::Identity()));
  EXPECT_DOUBLE_EQ(inertia.getMass(), 1.0);
  EXPECT_TRUE(inertia.getLocalCOM().isApprox(Eigen::Vector3d::Zero()));
  EXPECT_TRUE(inertia.verify(false));
}

//==============================================================================
TEST(Inertia, DiagonalMomentOfInertia)
{
  // Test with diagonal (but non-identity) moment of inertia
  Eigen::Matrix3d diagonalMoment = Eigen::Matrix3d::Zero();
  diagonalMoment(0, 0) = 1.0;
  diagonalMoment(1, 1) = 2.0;
  diagonalMoment(2, 2) = 3.0;

  dynamics::Inertia inertia(2.0, Eigen::Vector3d::Zero(), diagonalMoment);

  Eigen::Matrix3d retrieved = inertia.getMoment();
  EXPECT_DOUBLE_EQ(retrieved(0, 0), 1.0);
  EXPECT_DOUBLE_EQ(retrieved(1, 1), 2.0);
  EXPECT_DOUBLE_EQ(retrieved(2, 2), 3.0);
  EXPECT_DOUBLE_EQ(retrieved(0, 1), 0.0);
  EXPECT_DOUBLE_EQ(retrieved(0, 2), 0.0);
  EXPECT_DOUBLE_EQ(retrieved(1, 2), 0.0);
  EXPECT_TRUE(inertia.verify(false));
}

//==============================================================================
TEST(Inertia, ConstructorWithScalars)
{
  // Test the scalar constructor
  dynamics::Inertia inertia(
      5.0,  // mass
      1.0,  // comX
      2.0,  // comY
      3.0,  // comZ
      1.0,  // Ixx
      2.0,  // Iyy
      3.0,  // Izz
      0.1,  // Ixy
      0.2,  // Ixz
      0.3); // Iyz

  EXPECT_DOUBLE_EQ(inertia.getMass(), 5.0);
  EXPECT_TRUE(inertia.getLocalCOM().isApprox(Eigen::Vector3d(1.0, 2.0, 3.0)));

  Eigen::Matrix3d moment = inertia.getMoment();
  EXPECT_DOUBLE_EQ(moment(0, 0), 1.0);
  EXPECT_DOUBLE_EQ(moment(1, 1), 2.0);
  EXPECT_DOUBLE_EQ(moment(2, 2), 3.0);
  EXPECT_DOUBLE_EQ(moment(0, 1), 0.1);
  EXPECT_DOUBLE_EQ(moment(0, 2), 0.2);
  EXPECT_DOUBLE_EQ(moment(1, 2), 0.3);
}

//==============================================================================
TEST(Inertia, DefaultConstructor)
{
  // Test default constructor values
  dynamics::Inertia inertia;

  EXPECT_DOUBLE_EQ(inertia.getMass(), 1.0);
  EXPECT_TRUE(inertia.getLocalCOM().isApprox(Eigen::Vector3d::Zero()));
  EXPECT_TRUE(inertia.getMoment().isApprox(Eigen::Matrix3d::Identity()));
  EXPECT_TRUE(inertia.verify(false));
}

//==============================================================================
TEST(Inertia, SetSpatialTensorAndRetrieveParameters)
{
  // Create a valid spatial tensor
  Eigen::Matrix6d spatial = Eigen::Matrix6d::Zero();
  // Top-left: moment of inertia (identity)
  spatial.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
  // Bottom-right: mass * identity
  spatial.block<3, 3>(3, 3) = 3.0 * Eigen::Matrix3d::Identity();

  dynamics::Inertia inertia(spatial);

  EXPECT_DOUBLE_EQ(inertia.getMass(), 3.0);
  EXPECT_TRUE(inertia.getLocalCOM().isApprox(Eigen::Vector3d::Zero(), 1e-10));
}
