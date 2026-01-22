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

#include <dart/common/name_manager.hpp>

#include <gtest/gtest.h>

#include <memory>

using namespace dart;
using namespace dart::common;

class NameManagerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    mgr = std::make_unique<NameManager<int*>>("TestManager", "unnamed");
  }

  std::unique_ptr<NameManager<int*>> mgr;
  int obj1 = 1;
  int obj2 = 2;
  int obj3 = 3;
};

TEST_F(NameManagerTest, DefaultConstruction)
{
  EXPECT_EQ(mgr->getManagerName(), "TestManager");
  EXPECT_EQ(mgr->getDefaultName(), "unnamed");
  EXPECT_EQ(mgr->getCount(), 0u);
}

TEST_F(NameManagerTest, AddAndRetrieve)
{
  EXPECT_TRUE(mgr->addName("first", &obj1));
  EXPECT_EQ(mgr->getCount(), 1u);
  EXPECT_TRUE(mgr->hasName("first"));
  EXPECT_TRUE(mgr->hasObject(&obj1));
  EXPECT_EQ(mgr->getObject("first"), &obj1);
  EXPECT_EQ(mgr->getName(&obj1), "first");
}

TEST_F(NameManagerTest, DuplicateNameRejected)
{
  EXPECT_TRUE(mgr->addName("same", &obj1));
  EXPECT_FALSE(mgr->addName("same", &obj2));
  EXPECT_EQ(mgr->getCount(), 1u);
  EXPECT_EQ(mgr->getObject("same"), &obj1);
}

TEST_F(NameManagerTest, IssueNewName)
{
  EXPECT_TRUE(mgr->addName("item", &obj1));

  std::string newName = mgr->issueNewName("item");
  EXPECT_NE(newName, "item");
  EXPECT_FALSE(mgr->hasName(newName));
}

TEST_F(NameManagerTest, IssueNewNameAndAdd)
{
  EXPECT_TRUE(mgr->addName("item", &obj1));

  std::string newName = mgr->issueNewNameAndAdd("item", &obj2);
  EXPECT_NE(newName, "item");
  EXPECT_TRUE(mgr->hasName(newName));
  EXPECT_EQ(mgr->getObject(newName), &obj2);
  EXPECT_EQ(mgr->getCount(), 2u);
}

TEST_F(NameManagerTest, RemoveByName)
{
  mgr->addName("toRemove", &obj1);
  EXPECT_TRUE(mgr->hasName("toRemove"));

  EXPECT_TRUE(mgr->removeName("toRemove"));
  EXPECT_FALSE(mgr->hasName("toRemove"));
  EXPECT_FALSE(mgr->hasObject(&obj1));
  EXPECT_EQ(mgr->getCount(), 0u);
}

TEST_F(NameManagerTest, RemoveByObject)
{
  mgr->addName("objEntry", &obj1);
  EXPECT_TRUE(mgr->hasObject(&obj1));

  EXPECT_TRUE(mgr->removeObject(&obj1));
  EXPECT_FALSE(mgr->hasObject(&obj1));
  EXPECT_FALSE(mgr->hasName("objEntry"));
}

TEST_F(NameManagerTest, RemoveNonexistent)
{
  EXPECT_FALSE(mgr->removeName("doesNotExist"));
  EXPECT_FALSE(mgr->removeObject(&obj1));
}

TEST_F(NameManagerTest, Clear)
{
  mgr->addName("a", &obj1);
  mgr->addName("b", &obj2);
  EXPECT_EQ(mgr->getCount(), 2u);

  mgr->clear();
  EXPECT_EQ(mgr->getCount(), 0u);
  EXPECT_FALSE(mgr->hasName("a"));
  EXPECT_FALSE(mgr->hasName("b"));
}

TEST_F(NameManagerTest, ChangeObjectName)
{
  mgr->addName("oldName", &obj1);

  std::string result = mgr->changeObjectName(&obj1, "newName");
  EXPECT_EQ(result, "newName");
  EXPECT_FALSE(mgr->hasName("oldName"));
  EXPECT_TRUE(mgr->hasName("newName"));
  EXPECT_EQ(mgr->getName(&obj1), "newName");
}

TEST_F(NameManagerTest, ChangeObjectNameWithConflict)
{
  mgr->addName("name1", &obj1);
  mgr->addName("name2", &obj2);

  std::string result = mgr->changeObjectName(&obj1, "name2");
  EXPECT_NE(result, "name2");
  EXPECT_NE(result, "name1");
  EXPECT_TRUE(mgr->hasName(result));
  EXPECT_EQ(mgr->getName(&obj1), result);
}

TEST_F(NameManagerTest, ChangeNameOfUnregisteredObject)
{
  std::string result = mgr->changeObjectName(&obj1, "anyName");
  EXPECT_EQ(result, "anyName");
  EXPECT_FALSE(mgr->hasObject(&obj1));
}

TEST_F(NameManagerTest, SetPattern)
{
  EXPECT_TRUE(mgr->setPattern("%s_%d"));
  mgr->addName("base", &obj1);

  std::string newName = mgr->issueNewName("base");
  EXPECT_TRUE(newName.find("base_") != std::string::npos);
}

TEST_F(NameManagerTest, InvalidPatternRejected)
{
  EXPECT_FALSE(mgr->setPattern("noPlaceholders"));
  EXPECT_FALSE(mgr->setPattern("%s_only"));
  EXPECT_FALSE(mgr->setPattern("%d_only"));
}

TEST_F(NameManagerTest, SetAndGetDefaultName)
{
  mgr->setDefaultName("newDefault");
  EXPECT_EQ(mgr->getDefaultName(), "newDefault");
}

TEST_F(NameManagerTest, SetAndGetManagerName)
{
  mgr->setManagerName("RenamedManager");
  EXPECT_EQ(mgr->getManagerName(), "RenamedManager");
}

TEST_F(NameManagerTest, GetObjectNotFound)
{
  EXPECT_EQ(mgr->getObject("nonexistent"), nullptr);
}

TEST_F(NameManagerTest, GetNameNotFound)
{
  EXPECT_TRUE(mgr->getName(&obj1).empty());
}

TEST_F(NameManagerTest, RemoveEntries)
{
  mgr->addName("entry", &obj1);
  mgr->removeEntries("entry", &obj1);
  EXPECT_FALSE(mgr->hasName("entry"));
  EXPECT_FALSE(mgr->hasObject(&obj1));
}

TEST_F(NameManagerTest, MultipleObjects)
{
  mgr->addName("first", &obj1);
  mgr->addName("second", &obj2);
  mgr->addName("third", &obj3);

  EXPECT_EQ(mgr->getCount(), 3u);
  EXPECT_EQ(mgr->getObject("first"), &obj1);
  EXPECT_EQ(mgr->getObject("second"), &obj2);
  EXPECT_EQ(mgr->getObject("third"), &obj3);
}
