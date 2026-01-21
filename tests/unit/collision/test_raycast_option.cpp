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

#include <dart/collision/RaycastOption.hpp>

#include <gtest/gtest.h>

using namespace dart::collision;

TEST(RaycastOption, DefaultValues)
{
  RaycastOption option;

  EXPECT_FALSE(option.mEnableAllHits);
  EXPECT_FALSE(option.mSortByClosest);
  EXPECT_EQ(option.mFilter, nullptr);
}

TEST(RaycastOption, CustomValues)
{
  auto filter = [](const CollisionObject*) {
    return true;
  };
  RaycastOption option(true, true, filter);

  EXPECT_TRUE(option.mEnableAllHits);
  EXPECT_TRUE(option.mSortByClosest);
  EXPECT_NE(option.mFilter, nullptr);
}

TEST(RaycastOption, PassesFilterWithNoFilter)
{
  RaycastOption option;
  EXPECT_TRUE(option.passesFilter(nullptr));
}

TEST(RaycastOption, PassesFilterWithAllowingFilter)
{
  auto allowAll = [](const CollisionObject*) {
    return true;
  };
  RaycastOption option(false, false, allowAll);

  EXPECT_TRUE(option.passesFilter(nullptr));
}

TEST(RaycastOption, PassesFilterWithBlockingFilter)
{
  auto blockAll = [](const CollisionObject*) {
    return false;
  };
  RaycastOption option(false, false, blockAll);

  EXPECT_FALSE(option.passesFilter(nullptr));
}

TEST(RaycastOption, CopyAssignment)
{
  auto filter = [](const CollisionObject*) {
    return true;
  };
  RaycastOption original(true, true, filter);
  RaycastOption copy = original;

  EXPECT_EQ(copy.mEnableAllHits, original.mEnableAllHits);
  EXPECT_EQ(copy.mSortByClosest, original.mSortByClosest);
}
