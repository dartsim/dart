/*
 * Copyright (c) 2011-2021, The DART development contributors:
 * https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <gtest/gtest.h>

#include "dart/multibody/all.hpp"

using namespace dart;
using namespace multibody;

//==============================================================================
template <typename T>
struct FrameTest : public testing::Test
{
  using Type = T;
};

//==============================================================================
using Types = testing::Types<float, double>;

//==============================================================================
TYPED_TEST_SUITE(FrameTest, Types);

//==============================================================================
TYPED_TEST(FrameTest, InertialFrame)
{
  using Scalar = typename TestFixture::Type;

  auto inertial_frame = Frame<Scalar>::GetInertialFrame();
  ASSERT_NE(inertial_frame, nullptr);

  // Properties
  EXPECT_TRUE(inertial_frame->is_inertial_frame());
  EXPECT_EQ(inertial_frame->get_name(), InertialFrame<Scalar>::GetName());

  // Position == 0
  EXPECT_TRUE(inertial_frame->get_pose().is_identity());
  EXPECT_TRUE(inertial_frame->get_orientation().is_identity());
  EXPECT_TRUE(inertial_frame->get_position().is_zero());

  // Velocity == 0
  EXPECT_TRUE(inertial_frame->get_spatial_velocity().is_zero());
  EXPECT_TRUE(inertial_frame->get_angular_velocity().is_zero());
  EXPECT_TRUE(inertial_frame->get_linear_velocity().is_zero());

  // Acceleration == 0
  EXPECT_TRUE(inertial_frame->get_spatial_acceleration().is_zero());
  EXPECT_TRUE(inertial_frame->get_angular_acceleration().is_zero());
  EXPECT_TRUE(inertial_frame->get_linear_acceleration().is_zero());
}

//==============================================================================
TYPED_TEST(FrameTest, FreeFrame)
{
  using Scalar = typename TestFixture::Type;

  auto frame = FreeFrame<Scalar>();

  // Check default values
  EXPECT_EQ(frame.get_name(), "");
  EXPECT_TRUE(frame.get_pose().is_identity());
  EXPECT_TRUE(frame.get_spatial_velocity().is_zero());
  EXPECT_TRUE(frame.get_spatial_acceleration().is_zero());

  // Pose
  const math::SE3<Scalar> random_pose = math::SE3<Scalar>::Random();
  frame.set_pose(random_pose);
  EXPECT_EQ(frame.get_pose(), random_pose);

  // Orientation
  const math::SO3<Scalar> random_orientation = math::SO3<Scalar>::Random();
  frame.set_orientation(random_orientation);
  EXPECT_EQ(frame.get_orientation(), random_orientation);

  // Position
  const math::R3<Scalar> random_position = math::R3<Scalar>::Random();
  frame.set_position(random_position);
  EXPECT_EQ(frame.get_position(), random_position);

  // Spatial velocity
  const math::SE3Tangent<Scalar> random_spatial_velocity
      = math::SE3Tangent<Scalar>::Random();
  frame.set_spatial_velocity(random_spatial_velocity);
}

//==============================================================================
TYPED_TEST(FrameTest, FreeRelativeFrame)
{
  using Scalar = typename TestFixture::Type;

  auto frame = FreeRelativeFrame<Scalar>();
  EXPECT_EQ(frame.get_name(), "");
  EXPECT_TRUE(frame.get_pose().is_identity());
  EXPECT_TRUE(frame.get_spatial_velocity().is_zero());
  EXPECT_TRUE(frame.get_spatial_acceleration().is_zero());
}
