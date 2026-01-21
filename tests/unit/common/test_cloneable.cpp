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

#include <dart/common/cloneable.hpp>

#include <gtest/gtest.h>

#include <string>

using namespace dart::common;

namespace {

struct SimpleData
{
  int value = 0;
  std::string name;

  bool operator==(const SimpleData& other) const
  {
    return value == other.value && name == other.name;
  }
};

class SimpleBase : public Cloneable<SimpleBase>
{
public:
  virtual ~SimpleBase() = default;
};

using SimpleMakeCloneable = MakeCloneable<SimpleBase, SimpleData>;

} // namespace

//==============================================================================
TEST(MakeCloneable, DefaultConstruction)
{
  SimpleMakeCloneable cloneable;

  EXPECT_EQ(cloneable.value, 0);
  EXPECT_TRUE(cloneable.name.empty());
}

//==============================================================================
TEST(MakeCloneable, ConstructFromMixin)
{
  SimpleData data{42, "test"};
  SimpleMakeCloneable cloneable(data);

  EXPECT_EQ(cloneable.value, 42);
  EXPECT_EQ(cloneable.name, "test");
}

//==============================================================================
TEST(MakeCloneable, ConstructFromMixinRvalue)
{
  SimpleData data{99, "moved"};
  SimpleMakeCloneable cloneable(std::move(data));

  EXPECT_EQ(cloneable.value, 99);
  EXPECT_EQ(cloneable.name, "moved");
}

//==============================================================================
TEST(MakeCloneable, CopyConstruction)
{
  SimpleMakeCloneable original;
  original.value = 123;
  original.name = "original";

  SimpleMakeCloneable copy(original);

  EXPECT_EQ(copy.value, 123);
  EXPECT_EQ(copy.name, "original");
}

//==============================================================================
TEST(MakeCloneable, MoveConstruction)
{
  SimpleMakeCloneable original;
  original.value = 456;
  original.name = "to_move";

  SimpleMakeCloneable moved(std::move(original));

  EXPECT_EQ(moved.value, 456);
  EXPECT_EQ(moved.name, "to_move");
}

//==============================================================================
TEST(MakeCloneable, Clone)
{
  SimpleMakeCloneable original;
  original.value = 789;
  original.name = "cloned";

  auto cloned = original.clone();

  ASSERT_NE(cloned, nullptr);
  auto* casted = dynamic_cast<SimpleMakeCloneable*>(cloned.get());
  ASSERT_NE(casted, nullptr);
  EXPECT_EQ(casted->value, 789);
  EXPECT_EQ(casted->name, "cloned");
}

//==============================================================================
TEST(MakeCloneable, CopyMethod)
{
  SimpleMakeCloneable original;
  original.value = 111;
  original.name = "source";

  SimpleMakeCloneable target;
  target.copy(original);

  EXPECT_EQ(target.value, 111);
  EXPECT_EQ(target.name, "source");
}

//==============================================================================
TEST(MakeCloneable, AssignmentFromMixin)
{
  SimpleMakeCloneable cloneable;
  SimpleData data{555, "assigned"};

  cloneable = data;

  EXPECT_EQ(cloneable.value, 555);
  EXPECT_EQ(cloneable.name, "assigned");
}

//==============================================================================
TEST(MakeCloneable, AssignmentFromMixinRvalue)
{
  SimpleMakeCloneable cloneable;
  SimpleData data{666, "rvalue_assigned"};

  cloneable = std::move(data);

  EXPECT_EQ(cloneable.value, 666);
  EXPECT_EQ(cloneable.name, "rvalue_assigned");
}

//==============================================================================
TEST(MakeCloneable, CopyAssignment)
{
  SimpleMakeCloneable original;
  original.value = 333;
  original.name = "copy_src";

  SimpleMakeCloneable target;
  target = original;

  EXPECT_EQ(target.value, 333);
  EXPECT_EQ(target.name, "copy_src");
}

//==============================================================================
TEST(MakeCloneable, MoveAssignment)
{
  SimpleMakeCloneable original;
  original.value = 444;
  original.name = "move_src";

  SimpleMakeCloneable target;
  target = std::move(original);

  EXPECT_EQ(target.value, 444);
  EXPECT_EQ(target.name, "move_src");
}

//==============================================================================
TEST(MakeCloneable, CloneIsIndependent)
{
  SimpleMakeCloneable original;
  original.value = 100;
  original.name = "original";

  auto cloned = original.clone();
  auto* casted = dynamic_cast<SimpleMakeCloneable*>(cloned.get());
  ASSERT_NE(casted, nullptr);

  casted->value = 200;
  casted->name = "modified";

  EXPECT_EQ(original.value, 100);
  EXPECT_EQ(original.name, "original");
  EXPECT_EQ(casted->value, 200);
  EXPECT_EQ(casted->name, "modified");
}

//==============================================================================
TEST(MakeCloneable, CopyOverwritesExisting)
{
  SimpleMakeCloneable source;
  source.value = 50;
  source.name = "source";

  SimpleMakeCloneable target;
  target.value = 999;
  target.name = "will_be_overwritten";

  target.copy(source);

  EXPECT_EQ(target.value, 50);
  EXPECT_EQ(target.name, "source");
}
