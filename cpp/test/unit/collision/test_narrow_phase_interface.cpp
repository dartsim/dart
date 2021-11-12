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

#include "dart/collision/all.hpp"
#include "dart/collision/fcl/fcl_engine.hpp"

using namespace dart;

//==============================================================================
template <typename T>
struct NarrowPhaseTest : public testing::Test
{
  using Type = T;
};

//==============================================================================
using Types = testing::Types</*double, */ float>;

//==============================================================================
TYPED_TEST_SUITE(NarrowPhaseTest, Types);

//==============================================================================
template <typename EngineT>
void test_collide(const EngineT& engine)
{
  using Scalar = typename EngineT::element_type::Scalar;

  if (!engine) {
    return;
  }

  auto sphere1 = engine->create_sphere_object(0.5);
  auto sphere2 = engine->create_sphere_object(0.5);
  ASSERT_TRUE(sphere1);
  ASSERT_TRUE(sphere2);

  sphere1->set_position(math::Vector3<Scalar>(-1, 0, 0));
  sphere2->set_position(math::Vector3<Scalar>(1, 0, 0));
  EXPECT_FALSE(collision::collide(sphere1, sphere2));

  sphere1->set_position(math::Vector3<Scalar>(-0.25, 0, 0));
  sphere2->set_position(math::Vector3<Scalar>(0.25, 0, 0));
  EXPECT_TRUE(collision::collide(sphere1, sphere2));

  collision::CollisionOption<Scalar> option;
  option.enable_contact = true;
  option.max_num_contacts = 10;
  collision::CollisionResult<Scalar> result;
  sphere1->set_position(math::Vector3<Scalar>(-0.25, 0, 0));
  sphere2->set_position(math::Vector3<Scalar>(0.25, 0, 0));
  EXPECT_TRUE(collision::collide(sphere1, sphere2, option, &result));
}

//==============================================================================
TYPED_TEST(NarrowPhaseTest, Collide)
{
  using Scalar = typename TestFixture::Type;

  test_collide(collision::DartEngine<Scalar>::Create());
#if DART_HAVE_fcl
  test_collide(collision::FclEngine<Scalar>::Create());
#endif
#if DART_HAVE_ODE
  test_collide(collision::OdeEngine<Scalar>::Create());
#endif
#if DART_HAVE_Bullet
  test_collide(collision::BulletEngine<Scalar>::Create());
#endif
}
