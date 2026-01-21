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

#include <dart/collision/collision_filter.hpp>
#include <dart/collision/collision_option.hpp>

#include <gtest/gtest.h>

using namespace dart::collision;

TEST(CollisionOption, DefaultValues)
{
  CollisionOption option;

  EXPECT_TRUE(option.enableContact);
  EXPECT_EQ(option.maxNumContacts, 1000u);
  EXPECT_FALSE(option.allowNegativePenetrationDepthContacts);
  EXPECT_EQ(option.collisionFilter, nullptr);
}

TEST(CollisionOption, CustomValues)
{
  auto filter = std::make_shared<BodyNodeCollisionFilter>();

  CollisionOption option(false, 500u, filter, true);

  EXPECT_FALSE(option.enableContact);
  EXPECT_EQ(option.maxNumContacts, 500u);
  EXPECT_TRUE(option.allowNegativePenetrationDepthContacts);
  EXPECT_EQ(option.collisionFilter, filter);
}

TEST(CollisionOption, BinaryCheck)
{
  CollisionOption option(true, 1u);

  EXPECT_TRUE(option.enableContact);
  EXPECT_EQ(option.maxNumContacts, 1u);
}

TEST(CollisionOption, ZeroMaxContacts)
{
  CollisionOption option(true, 0u);

  EXPECT_EQ(option.maxNumContacts, 0u);
}

TEST(CollisionOption, CopyAssignment)
{
  auto filter = std::make_shared<BodyNodeCollisionFilter>();
  CollisionOption original(false, 42u, filter, true);

  CollisionOption copy = original;

  EXPECT_EQ(copy.enableContact, original.enableContact);
  EXPECT_EQ(copy.maxNumContacts, original.maxNumContacts);
  EXPECT_EQ(
      copy.allowNegativePenetrationDepthContacts,
      original.allowNegativePenetrationDepthContacts);
  EXPECT_EQ(copy.collisionFilter, original.collisionFilter);
}
