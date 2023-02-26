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

#include <dart/common/ecs/entity.hpp>

#include <gtest/gtest.h>

using namespace dart;
using namespace common;

template <typename DataT>
class EntityForTest : public EntityT<DataT>
{
public:
  using Base = EntityT<DataT>;
  using Base::DATA_BITS;
  using Base::ID_BITS;
  using Base::ID_MASK;
  using Base::VERSION_BITS;
  using Base::VERSION_MASK;
};

GTEST_TEST(EntityTest, StaticProperties)
{
  EXPECT_EQ(EntityForTest<std::uint16_t>::DATA_BITS, 16);
  EXPECT_EQ(EntityForTest<std::uint16_t>::ID_BITS, 8);
  EXPECT_EQ(EntityForTest<std::uint16_t>::VERSION_BITS, 8);

  EXPECT_EQ(EntityForTest<std::uint32_t>::DATA_BITS, 32);
  EXPECT_EQ(EntityForTest<std::uint32_t>::ID_BITS, 24);
  EXPECT_EQ(EntityForTest<std::uint32_t>::VERSION_BITS, 8);

  EXPECT_EQ(EntityForTest<std::uint64_t>::DATA_BITS, 64);
  EXPECT_EQ(EntityForTest<std::uint64_t>::ID_BITS, 56);
  EXPECT_EQ(EntityForTest<std::uint64_t>::VERSION_BITS, 8);
}

GTEST_TEST(EntityTest, DefaultConstruction)
{
  EntityT entity(0, 0);

  EXPECT_EQ(entity.getId(), 0);
  EXPECT_EQ(entity.getVersion(), 0);
}

GTEST_TEST(EntityTest, CustomConstruction)
{
  EntityT entity(123, 45);

  EXPECT_EQ(entity.getId(), 123);
  EXPECT_EQ(entity.getVersion(), 45);
}

GTEST_TEST(EntityTest, Equality)
{
  EntityT entity1(123, 45);
  EntityT entity2(123, 45);
  EntityT entity3(45, 123);

  EXPECT_EQ(entity1, entity2);
  EXPECT_NE(entity1, entity3);
}

GTEST_TEST(EntityTest, Bitmasks)
{
  EntityT<std::uint32_t> entity(0xFF, 0xFF);

  EXPECT_EQ(entity.getId(), 0xFF);
  EXPECT_EQ(entity.getVersion(), 0xFF);
}
