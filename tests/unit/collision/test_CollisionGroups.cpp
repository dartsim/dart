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
#include <dart/test/GTestUtils.hpp>

using namespace dart;

//==============================================================================
template <class T>
class CollisionGroupsTest : public ::testing::Test
{
};

template <class T>
struct Test;

template <class... T>
struct Test<std::tuple<T...>>
{
  using Types = ::testing::Types<T...>;
};

//==============================================================================
template <typename S>
collision2::CollisionDetectorPtr<S> createCollisionDetector(
    const std::string& name)
{
#if !DART_HAVE_BULLET
  if (engineName == "bullet")
  {
    return nullptr;
  }
#endif

  auto cd = collision2::CollisionDetector<S>::create(name);
  EXPECT_NE(cd, nullptr);

  return cd;
}

//==============================================================================
// using TestTypes = Test<test::Combinations_t<
//  std::tuple<double, float>, "dart", "fcl", "bullet", "ode">::Types;

// TYPED_TEST_CASE(CollisionGroupsTest, TestTypes);

////==============================================================================
// TYPED_TEST(CollisionGroupsTest, Constructor)
//{
//  using S = typename TypeParam::type;
//  const std::string engineName = TypeParam::GetParam();

//  if (engineName != "fcl")
//    return;

//  auto cd = createCollisionDetector<S>(engineName);
//  if (cd == nullptr)
//    return;

//  auto g1 = cd->createCollisionGroup();
//  EXPECT_NE(g1, nullptr);
//  EXPECT_EQ(g1->getCollisionDetector(), cd.get());

//  auto o1 = g1->createCollisionObject<math::Sphere<S>>(1);
//  ASSERT_NE(o1, nullptr);
//}

////==============================================================================
// INSTANTIATE_TEST_CASE_P(
//    CollisionEngine,
//    CollisionGroupsTest,
//    testing::Values("dart", "fcl", "bullet", "ode"));
