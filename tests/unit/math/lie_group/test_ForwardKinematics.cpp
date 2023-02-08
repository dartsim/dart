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

#include "dart/math/LieGroups.hpp"
#include "dart/test/math/GTestUtils.hpp"

#include <gtest/gtest.h>

using namespace dart;
using namespace math;

namespace {

template <typename Scalar_>
class Joint
{
public:
  using Scalar = Scalar_;
  Joint() = default;
  virtual ~Joint() = default;
};

template <typename Scalar_>
class JointRevolute : public Joint<Scalar_>
{
public:
  using Scalar = Scalar_;
  JointRevolute() = default;
  ~JointRevolute() override = default;

private:
  Vector3<Scalar> m_axis{Vector3<Scalar>::UnitX()};
  Vector6<Scalar> m_jacobian;
};

template <typename Scalar_>
class MultiBody
{
public:
  using Scalar = Scalar_;

  MultiBody(size_t numLinks = 0)
  {
    m_transforms.resize(numLinks);
    m_spatial_velocities.resize(numLinks);
    m_spatial_accelerations.resize(numLinks);

    m_local_transforms.resize(numLinks);
    m_local_jacobians.resize(numLinks, Matrix<Scalar, 6, 1>::Zero()); // TODO

    m_positions.setZero(numLinks);
    m_velocities.setZero(numLinks);
    m_accelerations.setZero(numLinks);
  }

  [[nodiscard]] auto getNumLinks() const
  {
    return m_transforms.size();
  }

  [[nodiscard]] auto getNumJoints() const
  {
    return getNumLinks();
  }

  [[nodiscard]] const auto& getLinkTransform(size_t index) const
  {
    return m_transforms[index];
  }

  [[nodiscard]] const auto& getLinkSpatialVelocity(size_t index) const
  {
    return m_spatial_velocities[index];
  }

  [[nodiscard]] const auto& getLinkSpatialAcceleration(size_t index) const
  {
    return m_spatial_accelerations[index];
  }

  void computeForwardKinematics()
  {
    // Link properties
    auto& X = m_transforms;
    auto& V = m_spatial_velocities;
    auto& dV = m_spatial_accelerations;

    // Joint properties
    auto& T = m_local_transforms;
    auto& S = m_local_jacobians;
    // auto& q = m_positions;
    auto& dq = m_velocities;
    auto& ddq = m_accelerations;

    if (getNumLinks() == 0) {
      return;
    }

    for (auto i = 1u; i < getNumLinks(); ++i) {
      // Update local transforms
      // TODO

      if (i == 0) {
        // Update global transforms
        X[0] = T[0];

        // global spatial velocities
        V[i] = S[i] * dq[i];

        // global spatial accelerations
        dV[i] = ad(V[i], S[i] * dq[i]) + S[i] * ddq[i];
      } else {
        // Update global transforms
        X[i] = X[i - 1] * T[i];

        // global spatial velocities
        V[i] = Ad(T[i], V[i - 1]) + S[i] * dq[i];

        // global spatial accelerations
        dV[i] = Ad(T[i], dV[i - 1]) + ad(V[i], S[i] * dq[i]) + S[i] * ddq[i];
      }
    }
  }

private:
  // Link properties
  std::vector<SE3<Scalar>> m_transforms;
  std::vector<SE3Tangent<Scalar>> m_spatial_velocities;
  std::vector<SE3Tangent<Scalar>> m_spatial_accelerations;

  // Joint properties
  std::vector<SE3<Scalar>> m_local_transforms;
  std::vector<Matrix<Scalar, 6, 1>> m_local_jacobians;
  VectorX<Scalar> m_positions;
  VectorX<Scalar> m_velocities;
  VectorX<Scalar> m_accelerations;
};

} // namespace

template <typename S>
struct ForwardKinematicsTest : public testing::Test
{
  using Scalar = S;
};

using Types = testing::Types<float, double>;
TYPED_TEST_SUITE(ForwardKinematicsTest, Types);

//==============================================================================
TYPED_TEST(ForwardKinematicsTest, DefaultConstructor)
{
  using S = typename TestFixture::Scalar;

  auto mb = MultiBody<S>();
  EXPECT_EQ(mb.getNumLinks(), 0);
  EXPECT_EQ(mb.getNumJoints(), 0);
}

//==============================================================================
TYPED_TEST(ForwardKinematicsTest, InitialValues)
{
  using S = typename TestFixture::Scalar;

  auto mb = MultiBody<S>(10);
  EXPECT_EQ(mb.getNumLinks(), 10);
  EXPECT_EQ(mb.getNumJoints(), 10);

  for (auto i = 0u; i < mb.getNumLinks(); ++i) {
    EXPECT_TRUE(mb.getLinkTransform(i).isIdentity());
    EXPECT_TRUE(mb.getLinkSpatialVelocity(i).isZero());
    EXPECT_TRUE(mb.getLinkSpatialAcceleration(i).isZero());
  }
}

//==============================================================================
TYPED_TEST(ForwardKinematicsTest, FwdKin)
{
  using S = typename TestFixture::Scalar;

  auto mb = MultiBody<S>(10);
  mb.computeForwardKinematics();
}
