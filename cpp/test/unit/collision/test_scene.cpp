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
#if DART_HAVE_fcl
  #include "dart/collision/fcl/fcl_engine.hpp"
#endif

using namespace dart;
using namespace collision;

//==============================================================================
template <typename T>
struct SceneTest : public testing::Test
{
  using Type = T;
};

//==============================================================================
using Types = testing::Types<float, double>;

//==============================================================================
TYPED_TEST_SUITE(SceneTest, Types);

//==============================================================================
template <typename EngineT>
void test_scene(const EngineT& engine)
{
  using Scalar = typename EngineT::element_type::Scalar;

  if (!engine) {
    return;
  }

  auto scene = engine->create_scene();
  ASSERT_TRUE(scene);

  auto sphere1 = scene->create_sphere_object(0.5);
  auto sphere2 = scene->create_sphere_object(0.5);
  ASSERT_TRUE(sphere1);
  ASSERT_TRUE(sphere2);

  EXPECT_EQ(sphere1->get_id(), 0);
  EXPECT_EQ(sphere2->get_id(), 1);

  scene->update();

  sphere1->set_position(math::Vector3<Scalar>::Random());
  sphere2->set_position(math::Vector3<Scalar>::Random());

  scene->update();

  engine->print();
}

//==============================================================================
TYPED_TEST(SceneTest, Collide)
{
  using Scalar = typename TestFixture::Type;

  test_scene(collision::DartEngine<Scalar>::Create());
#if DART_HAVE_fcl
  test_scene(collision::FclEngine<Scalar>::Create());
#endif
#if DART_HAVE_ODE
  test_scene(collision::OdeEngine<Scalar>::Create());
#endif
#if DART_HAVE_Bullet
  test_scene(collision::BulletEngine<Scalar>::Create());
#endif
}

//==============================================================================
TYPED_TEST(SceneTest, MemoryManager)
{
  common::set_log_level_to_debug();

  using Scalar = typename TestFixture::Type;

  auto engine = DartEngine<Scalar>::Create();
  auto scene = engine->create_scene();
  (void)scene;

  engine->destroy_scene(scene);
}
