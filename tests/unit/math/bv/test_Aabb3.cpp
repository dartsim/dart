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

#include "dart/math/math.hpp"
#include "dart/test/math/GTestUtils.hpp"

#include <gtest/gtest.h>

using namespace dart;
using namespace math;

template <typename S>
struct Aabb3Test : public testing::Test
{
  using Scalar = S;
};

using Types = testing::Types<float, double>;
TYPED_TEST_SUITE(Aabb3Test, Types);

//==============================================================================
TYPED_TEST(Aabb3Test, DefaultConstructor)
{
  using S = typename TestFixture::Scalar;

  Aabb3<S> aabb3;
  EXPECT_S_EQ(aabb3.getMin().x(), std::numeric_limits<S>::max());
  EXPECT_S_EQ(aabb3.getMin().y(), std::numeric_limits<S>::max());
  EXPECT_S_EQ(aabb3.getMin().z(), std::numeric_limits<S>::max());
  EXPECT_S_EQ(aabb3.getMax().x(), -std::numeric_limits<S>::max());
  EXPECT_S_EQ(aabb3.getMax().y(), -std::numeric_limits<S>::max());
  EXPECT_S_EQ(aabb3.getMax().z(), -std::numeric_limits<S>::max());
}

//==============================================================================
TYPED_TEST(Aabb3Test, CopyConstructor)
{
  using S = typename TestFixture::Scalar;

  Aabb3<S> aabb3_1;
  aabb3_1.merge(Vector3<S>(0, 0, 0));
  aabb3_1.merge(Vector3<S>(1, 1, 1));

  Aabb3<S> aabb3_2(aabb3_1);
  EXPECT_S_EQ(aabb3_2.getMin().x(), 0);
  EXPECT_S_EQ(aabb3_2.getMin().y(), 0);
  EXPECT_S_EQ(aabb3_2.getMin().z(), 0);
  EXPECT_S_EQ(aabb3_2.getMax().x(), 1);
  EXPECT_S_EQ(aabb3_2.getMax().y(), 1);
  EXPECT_S_EQ(aabb3_2.getMax().z(), 1);
}

//==============================================================================
TYPED_TEST(Aabb3Test, MoveConstructor)
{
  using S = typename TestFixture::Scalar;

  Aabb3<S> aabb3_1;
  aabb3_1.merge(Vector3<S>(0, 0, 0));
  aabb3_1.merge(Vector3<S>(1, 1, 1));

  Aabb3<S> aabb3_2(std::move(aabb3_1));
  EXPECT_S_EQ(aabb3_2.getMin().x(), 0);
  EXPECT_S_EQ(aabb3_2.getMin().y(), 0);
  EXPECT_S_EQ(aabb3_2.getMin().z(), 0);
  EXPECT_S_EQ(aabb3_2.getMax().x(), 1);
  EXPECT_S_EQ(aabb3_2.getMax().y(), 1);
  EXPECT_S_EQ(aabb3_2.getMax().z(), 1);
}

//==============================================================================
TYPED_TEST(Aabb3Test, MergeWithAabb3)
{
  using S = typename TestFixture::Scalar;

  Aabb3<S> aabb1(Vector3<S>(-1, -1, -1), Vector3<S>(1, 1, 1));
  Aabb3<S> aabb2(Vector3<S>(0, 0, 0), Vector3<S>(2, 2, 2));
  aabb1.merge(aabb2);
  EXPECT_EQ(aabb1.getMin(), Vector3<S>(-1, -1, -1));
  EXPECT_EQ(aabb1.getMax(), Vector3<S>(2, 2, 2));
}

//==============================================================================
TYPED_TEST(Aabb3Test, MergeWithVector3)
{
  using S = typename TestFixture::Scalar;

  // Create a bounding box that covers the unit cube centered at the origin.
  Aabb3<S> box1;
  box1.merge({-0.5, -0.5, -0.5});
  box1.merge({0.5, 0.5, 0.5});

  // Create a bounding box that covers the unit sphere centered at (1, 1, 1).
  Aabb3<S> box2;
  box2.merge({0.5, 0.5, 0.5});
  box2.merge({1.5, 1.5, 1.5});

  // Merge the two boxes and check that the result covers the union of the two
  // original boxes.
  box1.merge(box2);
  EXPECT_EQ(box1.getMin(), Vector3<S>(-0.5, -0.5, -0.5));
  EXPECT_EQ(box1.getMax(), Vector3<S>(1.5, 1.5, 1.5));
}

//==============================================================================
TYPED_TEST(Aabb3Test, Overlaps)
{
  using S = typename TestFixture::Scalar;

  // Create a bounding box that covers the unit cube centered at the origin.
  Aabb3<S> box1;
  box1.merge({-0.5, -0.5, -0.5});
  box1.merge({0.5, 0.5, 0.5});

  // Create a bounding box that covers the unit sphere centered at (1, 1, 1).
  Aabb3<S> box2;
  box2.merge({0.5, 0.5, 0.5});
  box2.merge({1.5, 1.5, 1.5});

  // Create a bounding box that covers the unit sphere centered at (2, 2, 2).
  Aabb3<S> box3;
  box3.merge({1.5, 1.5, 1.5});
  box3.merge({2.5, 2.5, 2.5});

  // Check that the first two boxes overlap but the third box does not overlap
  // with the first box.
  EXPECT_TRUE(box1.overlaps(box2));
  EXPECT_TRUE(box2.overlaps(box1));
  EXPECT_FALSE(box1.overlaps(box3));
  EXPECT_FALSE(box3.overlaps(box1));
}

//==============================================================================
TYPED_TEST(Aabb3Test, Contains)
{
  using S = typename TestFixture::Scalar;

  Aabb3<S> aabb1(Vector3<S>(-1, -1, -1), Vector3<S>(1, 1, 1));
  Aabb3<S> aabb2(Vector3<S>(-0.5, -0.5, -0.5), Vector3<S>(0.5, 0.5, 0.5));
  Aabb3<S> aabb3(Vector3<S>(-2, -2, -2), Vector3<S>(-1, -1, -1));
  EXPECT_TRUE(aabb1.contains(aabb2));
  EXPECT_FALSE(aabb1.contains(aabb3));
}
