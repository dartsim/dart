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

#include <gtest/gtest.h>

#include <dart/collision/collision.hpp>
#include <dart/math/math.hpp>

using namespace dart;

//==============================================================================
template <typename EngineT>
void test_collide(const EngineT& engine)
{
  if (!engine)
  {
    return;
  }

  auto sphere1 = engine->create_sphere_object();
  auto sphere2 = engine->create_sphere_object();
  ASSERT_TRUE(sphere1);
  ASSERT_TRUE(sphere2);

  sphere1->set_position(math::Vector3<double>(-1, 0, 0));
  sphere2->set_position(math::Vector3<double>(1, 0, 0));
  EXPECT_FALSE(collision::collide(sphere1, sphere2));

  sphere1->set_position(math::Vector3<double>(-0.25, 0, 0));
  sphere2->set_position(math::Vector3<double>(0.25, 0, 0));
  EXPECT_TRUE(collision::collide(sphere1, sphere2));
}

//==============================================================================
TEST(NarrowPhaseTest, Collide)
{
  test_collide(collision::Engine<double>::create("fcl"));
}
