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

#include <dart/all.hpp>

#include <gtest/gtest.h>

#include <map>
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

using SimplePtr = std::unique_ptr<SimpleBase>;

struct ProxyOwner
{
  SimpleData data;
};

void setProxyData(ProxyOwner* owner, const SimpleData& data)
{
  owner->data = data;
}

SimpleData getProxyData(const ProxyOwner* owner)
{
  return owner->data;
}

class ProxyBase : public Cloneable<ProxyBase>
{
public:
  ~ProxyBase() override = default;
};

class CopyableProxyBase
{
public:
  virtual ~CopyableProxyBase() = default;

  virtual std::unique_ptr<CopyableProxyBase> clone() const
  {
    return std::make_unique<CopyableProxyBase>(*this);
  }

  virtual void copy(const CopyableProxyBase& other)
  {
    (void)other;
  }
};

using SimpleProxy = ProxyCloneable<
    ProxyBase,
    ProxyOwner,
    SimpleData,
    setProxyData,
    getProxyData>;

using CopyableProxy = ProxyCloneable<
    CopyableProxyBase,
    ProxyOwner,
    SimpleData,
    setProxyData,
    getProxyData>;

SimplePtr makeSimpleCloneable(int value, const std::string& name)
{
  auto cloneable = std::make_unique<SimpleMakeCloneable>();
  cloneable->value = value;
  cloneable->name = name;
  return cloneable;
}

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

//==============================================================================
TEST(CloneableVector, MoveAndCopy)
{
  std::vector<SimplePtr> raw;
  raw.push_back(makeSimpleCloneable(1, "one"));
  raw.push_back(nullptr);
  raw.push_back(makeSimpleCloneable(2, "two"));

  CloneableVector<SimplePtr> moved(std::move(raw));
  EXPECT_EQ(moved.getVector().size(), 3u);

  CloneableVector<SimplePtr> copied(moved);
  EXPECT_EQ(copied.getVector().size(), 3u);

  const auto& copiedVector = copied.getVector();
  ASSERT_NE(copiedVector[0], nullptr);
  EXPECT_EQ(
      static_cast<const SimpleMakeCloneable*>(copiedVector[0].get())->value, 1);
  EXPECT_EQ(
      static_cast<const SimpleMakeCloneable*>(copiedVector[2].get())->value, 2);
  EXPECT_EQ(copiedVector[1], nullptr);

  std::vector<SimplePtr> existing;
  existing.push_back(makeSimpleCloneable(10, "old"));
  existing.push_back(makeSimpleCloneable(11, "old2"));
  existing.push_back(makeSimpleCloneable(12, "old3"));
  CloneableVector<SimplePtr> target(std::move(existing));
  target.copy(copied);

  const auto& targetVector = target.getVector();
  ASSERT_NE(targetVector[0], nullptr);
  EXPECT_EQ(
      static_cast<const SimpleMakeCloneable*>(targetVector[0].get())->value, 1);
  EXPECT_EQ(targetVector[1], nullptr);
  ASSERT_NE(targetVector[2], nullptr);
  EXPECT_EQ(
      static_cast<const SimpleMakeCloneable*>(targetVector[2].get())->value, 2);
}

//==============================================================================
TEST(CloneableVector, MoveConstructorTransfersOwnership)
{
  std::vector<SimplePtr> raw;
  raw.push_back(makeSimpleCloneable(7, "seven"));

  CloneableVector<SimplePtr> moved(std::move(raw));

  const auto& movedVector = moved.getVector();
  ASSERT_EQ(movedVector.size(), 1u);
  ASSERT_NE(movedVector[0], nullptr);
  EXPECT_EQ(
      static_cast<const SimpleMakeCloneable*>(movedVector[0].get())->value, 7);
}

//==============================================================================
TEST(CloneableVector, CopyConstructorClonesElements)
{
  std::vector<SimplePtr> raw;
  raw.push_back(makeSimpleCloneable(21, "twenty_one"));
  raw.push_back(nullptr);

  CloneableVector<SimplePtr> original(std::move(raw));
  CloneableVector<SimplePtr> copied(original);

  const auto& originalVector = original.getVector();
  const auto& copiedVector = copied.getVector();
  ASSERT_EQ(copiedVector.size(), 2u);
  ASSERT_NE(copiedVector[0], nullptr);
  EXPECT_NE(copiedVector[0].get(), originalVector[0].get());
  EXPECT_EQ(
      static_cast<const SimpleMakeCloneable*>(copiedVector[0].get())->value,
      21);
  EXPECT_EQ(copiedVector[1], nullptr);
}

//==============================================================================
TEST(CloneableVector, CopyUpdatesExistingEntriesInPlace)
{
  std::vector<SimplePtr> sourceRaw;
  sourceRaw.push_back(makeSimpleCloneable(5, "five"));
  CloneableVector<SimplePtr> source(std::move(sourceRaw));

  std::vector<SimplePtr> targetRaw;
  targetRaw.push_back(makeSimpleCloneable(9, "nine"));
  CloneableVector<SimplePtr> target(std::move(targetRaw));

  const auto* originalPtr = target.getVector()[0].get();
  target.copy(source);

  const auto& targetVector = target.getVector();
  ASSERT_NE(targetVector[0], nullptr);
  EXPECT_EQ(targetVector[0].get(), originalPtr);
  EXPECT_EQ(
      static_cast<const SimpleMakeCloneable*>(targetVector[0].get())->value, 5);
}

//==============================================================================
TEST(CloneableMap, MoveAndMerge)
{
  using SimpleMap = std::map<std::string, SimplePtr>;

  SimpleMap baseMap;
  baseMap["keep"] = makeSimpleCloneable(1, "keep");
  baseMap["replace"] = makeSimpleCloneable(2, "replace");

  CloneableMap<SimpleMap> holder(baseMap);
  CloneableMap<SimpleMap> movedHolder(std::move(holder));

  SimpleMap movedMap;
  movedMap["moved"] = makeSimpleCloneable(3, "moved");
  CloneableMap<SimpleMap> mapAssigned;
  mapAssigned = std::move(movedMap);
  EXPECT_NE(mapAssigned.getMap()["moved"], nullptr);

  SimpleMap incoming;
  incoming["replace"] = nullptr;
  incoming["added"] = makeSimpleCloneable(4, "added");

  movedHolder.merge(incoming);
  EXPECT_NE(movedHolder.getMap()["replace"], nullptr);
  EXPECT_NE(movedHolder.getMap()["added"], nullptr);

  movedHolder.copy(incoming, false);
  EXPECT_EQ(movedHolder.getMap()["replace"], nullptr);
}

//==============================================================================
TEST(CloneableMap, MergeMatchingKeysCopiesInPlace)
{
  using SimpleMap = std::map<std::string, SimplePtr>;

  SimpleMap baseMap;
  baseMap["shared"] = makeSimpleCloneable(1, "base");
  CloneableMap<SimpleMap> holder(baseMap);

  SimpleMap incoming;
  incoming["shared"] = makeSimpleCloneable(2, "incoming");

  const auto* originalPtr = holder.getMap().at("shared").get();
  holder.merge(incoming);

  const auto& map = holder.getMap();
  ASSERT_NE(map.at("shared"), nullptr);
  EXPECT_EQ(map.at("shared").get(), originalPtr);
  EXPECT_EQ(
      static_cast<const SimpleMakeCloneable*>(map.at("shared").get())->value,
      2);
}

//==============================================================================
TEST(CloneableMap, MergeFromCloneableMap)
{
  using SimpleMap = std::map<std::string, SimplePtr>;

  SimpleMap baseMap;
  baseMap["base"] = makeSimpleCloneable(1, "base");
  baseMap["override"] = makeSimpleCloneable(2, "old");

  CloneableMap<SimpleMap> holder(baseMap);

  SimpleMap incomingMap;
  incomingMap["override"] = makeSimpleCloneable(3, "new");
  incomingMap["added"] = makeSimpleCloneable(4, "added");

  CloneableMap<SimpleMap> incomingHolder(incomingMap);
  holder.merge(incomingHolder);

  const auto& map = holder.getMap();
  ASSERT_NE(map.at("override"), nullptr);
  EXPECT_EQ(
      static_cast<const SimpleMakeCloneable*>(map.at("override").get())->value,
      3);
  ASSERT_NE(map.at("added"), nullptr);
}

//==============================================================================
TEST(CloneableMap, CopyConstructorAndAssignment)
{
  using SimpleMap = std::map<std::string, SimplePtr>;

  SimpleMap baseMap;
  baseMap["first"] = makeSimpleCloneable(1, "first");
  baseMap["empty"] = nullptr;

  CloneableMap<SimpleMap> holder(baseMap);
  CloneableMap<SimpleMap> copied(holder);

  const auto& copiedMap = copied.getMap();
  ASSERT_NE(copiedMap.at("first"), nullptr);
  EXPECT_EQ(
      static_cast<const SimpleMakeCloneable*>(copiedMap.at("first").get())
          ->value,
      1);
  ASSERT_TRUE(copiedMap.contains("empty"));
  EXPECT_EQ(copiedMap.at("empty"), nullptr);

  CloneableMap<SimpleMap> assigned;
  assigned = holder;
  ASSERT_NE(assigned.getMap().at("first"), nullptr);
  EXPECT_EQ(
      static_cast<const SimpleMakeCloneable*>(
          assigned.getMap().at("first").get())
          ->value,
      1);
}

//==============================================================================
TEST(CloneableVector, CopyAssignment)
{
  std::vector<SimplePtr> sourceVec;
  sourceVec.push_back(makeSimpleCloneable(5, "five"));
  CloneableVector<SimplePtr> source(std::move(sourceVec));

  std::vector<SimplePtr> targetVec;
  targetVec.push_back(makeSimpleCloneable(9, "nine"));
  CloneableVector<SimplePtr> target(std::move(targetVec));

  target = source;

  const auto& targetVector = target.getVector();
  ASSERT_EQ(targetVector.size(), 1u);
  ASSERT_NE(targetVector[0], nullptr);
  EXPECT_EQ(
      static_cast<const SimpleMakeCloneable*>(targetVector[0].get())->value, 5);
}

//==============================================================================
TEST(ProxyCloneable, CopyAndMoveAssignment)
{
  ProxyOwner owner;
  owner.data = SimpleData{1, "owner"};

  SimpleProxy owned(&owner);
  owned = SimpleData{2, "updated"};
  EXPECT_EQ(owner.data.value, 2);
  EXPECT_EQ(owner.data.name, "updated");

  SimpleProxy standalone(SimpleData{3, "standalone"});
  SimpleProxy moved;
  moved = std::move(standalone);
  EXPECT_EQ(moved.get().value, 3);
  EXPECT_EQ(moved.get().name, "standalone");

  SimpleProxy copied;
  copied = owned;
  EXPECT_EQ(copied.get().value, 2);
  EXPECT_EQ(copied.get().name, "updated");
}

//==============================================================================
TEST(ProxyCloneable, CopyAndMoveConstructors)
{
  ProxyOwner owner;
  owner.data = SimpleData{7, "owner"};

  CopyableProxy owned(&owner);
  owned = SimpleData{8, "updated"};

  const CopyableProxy& constOwned = owned;
  CopyableProxy copied(constOwned);
  EXPECT_EQ(copied.get().value, 8);
  EXPECT_EQ(copied.get().name, "updated");
  EXPECT_EQ(copied.getOwner(), nullptr);

  CopyableProxy moved(std::move(owned));
  EXPECT_EQ(moved.get().value, 8);
  EXPECT_EQ(moved.get().name, "updated");
  EXPECT_EQ(moved.getOwner(), nullptr);

  EXPECT_EQ(owner.data.value, 8);
  EXPECT_EQ(owner.data.name, "updated");
}
