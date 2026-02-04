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

#ifndef DART_TESTS_LCPTESTFIXTURES_HPP_
#define DART_TESTS_LCPTESTFIXTURES_HPP_

#include <dart/math/lcp/lcp_types.hpp>

#include <Eigen/Core>

#include <limits>
#include <string>
#include <vector>

namespace dart::test {

enum class LcpFixtureKind
{
  Standard,
  Boxed,
  BoxedFriction
};

struct LcpFixture
{
  std::string name;
  dart::math::LcpProblem problem;
  Eigen::VectorXd expected;
  LcpFixtureKind kind;
};

inline std::vector<LcpFixture> getStandardBoxedFixtures()
{
  const double kInf = std::numeric_limits<double>::infinity();
  std::vector<LcpFixture> fixtures;

  {
    Eigen::Matrix2d A;
    A << 4.0, 1.0, 1.0, 3.0;
    const Eigen::Vector2d xStar(0.5, 0.25);
    const Eigen::Vector2d b = A * xStar;

    fixtures.push_back(
        LcpFixture{
            "standard_2d_spd",
            dart::math::LcpProblem(
                A,
                b,
                Eigen::Vector2d::Zero(),
                Eigen::Vector2d::Constant(kInf),
                Eigen::Vector2i::Constant(-1)),
            xStar,
            LcpFixtureKind::Standard});
  }

  {
    Eigen::Matrix2d A;
    A << 2.0, 0.5, 0.5, 1.5;
    Eigen::Vector2d lo;
    lo << -1.0, 0.0;
    Eigen::Vector2d hi;
    hi << 1.0, 0.2;
    Eigen::Vector2d xStar;
    xStar << 0.1, 0.2;
    Eigen::Vector2d w;
    w << 0.0, -0.3;
    const Eigen::Vector2d b = A * xStar - w;

    fixtures.push_back(
        LcpFixture{
            "boxed_active_upper",
            dart::math::LcpProblem(A, b, lo, hi, Eigen::Vector2i::Constant(-1)),
            xStar,
            LcpFixtureKind::Boxed});
  }

  {
    Eigen::Matrix3d A;
    A << 3.0, 0.2, 0.0, 0.2, 2.5, 0.1, 0.0, 0.1, 2.0;
    Eigen::Vector3d lo;
    lo << -0.2, -0.5, -0.1;
    Eigen::Vector3d hi;
    hi << 0.0, 0.5, 0.5;
    Eigen::Vector3d xStar;
    xStar << 0.0, -0.5, 0.3;
    Eigen::Vector3d w;
    w << -0.1, 0.2, 0.0;
    const Eigen::Vector3d b = A * xStar - w;

    fixtures.push_back(
        LcpFixture{
            "boxed_mixed_bounds",
            dart::math::LcpProblem(A, b, lo, hi, Eigen::Vector3i::Constant(-1)),
            xStar,
            LcpFixtureKind::Boxed});
  }

  return fixtures;
}

inline std::vector<LcpFixture> getFrictionIndexFixtures()
{
  const double kInf = std::numeric_limits<double>::infinity();
  std::vector<LcpFixture> fixtures;

  {
    Eigen::Matrix3d A;
    A << 4.0, 0.5, 0.0, 0.5, 3.0, 0.25, 0.0, 0.25, 2.5;
    Eigen::Vector3d xStar;
    xStar << 1.0, 0.2, -0.1;
    const Eigen::Vector3d b = A * xStar;

    const double mu = 0.5;
    Eigen::Vector3d lo;
    lo << 0.0, -mu, -mu;
    Eigen::Vector3d hi;
    hi << kInf, mu, mu;
    Eigen::Vector3i findex;
    findex << -1, 0, 0;

    fixtures.push_back(
        LcpFixture{
            "findex_single_contact_interior",
            dart::math::LcpProblem(A, b, lo, hi, findex),
            xStar,
            LcpFixtureKind::BoxedFriction});
  }

  {
    Eigen::Matrix3d A;
    A << 3.5, 0.4, 0.1, 0.4, 2.8, 0.0, 0.1, 0.0, 2.2;
    Eigen::Vector3d xStar;
    xStar << 0.8, 0.4, -0.4;
    const Eigen::Vector3d b = A * xStar;

    const double mu = 0.5;
    Eigen::Vector3d lo;
    lo << 0.0, -mu, -mu;
    Eigen::Vector3d hi;
    hi << kInf, mu, mu;
    Eigen::Vector3i findex;
    findex << -1, 0, 0;

    fixtures.push_back(
        LcpFixture{
            "findex_single_contact_at_bounds",
            dart::math::LcpProblem(A, b, lo, hi, findex),
            xStar,
            LcpFixtureKind::BoxedFriction});
  }

  {
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(6, 6);
    Eigen::Matrix3d A1;
    A1 << 3.0, 0.1, 0.0, 0.1, 2.5, 0.2, 0.0, 0.2, 2.2;
    Eigen::Matrix3d A2;
    A2 << 4.0, 0.3, 0.0, 0.3, 3.5, 0.2, 0.0, 0.2, 3.0;
    A.block(0, 0, 3, 3) = A1;
    A.block(3, 3, 3, 3) = A2;

    Eigen::VectorXd xStar = Eigen::VectorXd::Zero(6);
    xStar[3] = 1.2;
    xStar[4] = 0.2;
    xStar[5] = -0.1;
    const Eigen::VectorXd b = A * xStar;

    const double mu1 = 0.6;
    const double mu2 = 0.3;
    Eigen::VectorXd lo = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd hi = Eigen::VectorXd::Zero(6);
    Eigen::VectorXi findex = Eigen::VectorXi::Constant(6, -1);

    lo[0] = 0.0;
    hi[0] = kInf;
    lo[1] = -mu1;
    hi[1] = mu1;
    findex[1] = 0;
    lo[2] = -mu1;
    hi[2] = mu1;
    findex[2] = 0;

    lo[3] = 0.0;
    hi[3] = kInf;
    lo[4] = -mu2;
    hi[4] = mu2;
    findex[4] = 3;
    lo[5] = -mu2;
    hi[5] = mu2;
    findex[5] = 3;

    fixtures.push_back(
        LcpFixture{
            "findex_two_contacts_mixed",
            dart::math::LcpProblem(A, b, lo, hi, findex),
            xStar,
            LcpFixtureKind::BoxedFriction});
  }

  return fixtures;
}

} // namespace dart::test

#endif // DART_TESTS_LCPTESTFIXTURES_HPP_
