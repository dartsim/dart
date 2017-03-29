/*
 * Copyright (c) 2017, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2017, Personal Robotics Lab, Carnegie Mellon University
 * All rights reserved.
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
#include <dart/common/Factory.hpp>
#include "TestHelpers.hpp"

using namespace dart;

enum ObjectTypeEnum
{
  OT_None,
  OT_Apple,
  OT_Banana,
  OT_Orange
};

enum class ObjectTypeEnumClass
{
  None,
  Apple,
  Banana,
  Orange
};

class Fruit
{
public:
  virtual std::string getName() const = 0;
};

class Apple : public Fruit
{
public:
  std::string getName() const override
  {
    return "apple";
  }
};

class Banana : public Fruit
{
public:
  std::string getName() const override
  {
    return "banana";
  }
};

class Orange : public Fruit
{
public:
  std::string getName() const override
  {
    return "orange";
  }
};

DART_REGISTER_OBJECT_TO_FACTORY(ObjectTypeEnum, OT_Apple, Fruit, Apple)
DART_REGISTER_OBJECT_TO_FACTORY(ObjectTypeEnum, OT_Banana, Fruit, Banana)

DART_REGISTER_OBJECT_TO_FACTORY(
    ObjectTypeEnumClass, ObjectTypeEnumClass::Apple, Fruit, Apple)
DART_REGISTER_OBJECT_TO_FACTORY(
    ObjectTypeEnumClass, ObjectTypeEnumClass::Banana, Fruit, Banana)

DART_REGISTER_OBJECT_TO_FACTORY(std::string, "apple", Fruit, Apple)
DART_REGISTER_OBJECT_TO_FACTORY(std::string, "banana", Fruit, Banana)

//==============================================================================
TEST(Factory, Create)
{
  //----------------------------------------------------------------------------
  // enum class key type
  //----------------------------------------------------------------------------
  using FruitFactoryEnum = common::Factory<ObjectTypeEnum, Fruit>;

  EXPECT_TRUE(!FruitFactoryEnum::create(OT_None));

  EXPECT_TRUE(FruitFactoryEnum::canCreate(OT_Apple));
  EXPECT_EQ(FruitFactoryEnum::create(OT_Apple)->getName(), "apple");

  // Can't create an orange since it's not registered yet.
  EXPECT_TRUE(!FruitFactoryEnum::canCreate(OT_Orange));

  // Once it's registered we can create an orange.
  FruitFactoryEnum::registerCreator<Orange>(OT_Orange);
  EXPECT_TRUE(FruitFactoryEnum::canCreate(OT_Orange));
  EXPECT_EQ(FruitFactoryEnum::create(OT_Orange)->getName(), "orange");

  FruitFactoryEnum::unregisterCreator(OT_Orange);
  EXPECT_TRUE(!FruitFactoryEnum::canCreate(OT_Orange));

  //----------------------------------------------------------------------------
  // enum class key type
  //----------------------------------------------------------------------------
  using FruitFactoryEnumClass = common::Factory<ObjectTypeEnumClass, Fruit>;

  EXPECT_TRUE(!FruitFactoryEnumClass::create(ObjectTypeEnumClass::None));

  EXPECT_TRUE(FruitFactoryEnumClass::canCreate(ObjectTypeEnumClass::Apple));
  EXPECT_EQ(FruitFactoryEnumClass::create(
      ObjectTypeEnumClass::Apple)->getName(), "apple");

  // Can't create an orange since it's not registered yet.
  EXPECT_TRUE(!FruitFactoryEnumClass::canCreate(ObjectTypeEnumClass::Orange));

  // Once it's registered we can create an orange.
  FruitFactoryEnumClass::registerCreator<Orange>(ObjectTypeEnumClass::Orange);
  EXPECT_TRUE(FruitFactoryEnumClass::canCreate(ObjectTypeEnumClass::Orange));
  EXPECT_EQ(FruitFactoryEnumClass::create(
      ObjectTypeEnumClass::Orange)->getName(), "orange");

  FruitFactoryEnumClass::unregisterCreator(ObjectTypeEnumClass::Orange);
  EXPECT_TRUE(!FruitFactoryEnumClass::canCreate(ObjectTypeEnumClass::Orange));

  //----------------------------------------------------------------------------
  // std::string key type
  //----------------------------------------------------------------------------
  using FruitFactoryString = common::Factory<std::string, Fruit>;

  EXPECT_TRUE(!FruitFactoryString::create("none"));

  EXPECT_TRUE(FruitFactoryString::canCreate("apple"));
  EXPECT_EQ(FruitFactoryString::create("apple")->getName(), "apple");

  // Can't create an orange since it's not registered yet.
  EXPECT_TRUE(!FruitFactoryString::canCreate("orange"));

  // Once it's registered we can create an orange.
  FruitFactoryString::registerCreator<Orange>("orange");
  EXPECT_TRUE(FruitFactoryString::canCreate("orange"));
  EXPECT_EQ(FruitFactoryString::create("orange")->getName(), "orange");

  FruitFactoryString::unregisterCreator("orange");
  EXPECT_TRUE(!FruitFactoryString::canCreate("orange"));
}

//==============================================================================
static Orange* createOrange()
{
  return new Orange();
}

//==============================================================================
TEST(Factory, VariousCreatorFunctions)
{
  using FruitFactory = common::Factory<std::string, Fruit>;

  FruitFactory::unregisterAllCreators();

  EXPECT_TRUE(!FruitFactory::canCreate("apple"));
  EXPECT_TRUE(!FruitFactory::canCreate("banana"));
  EXPECT_TRUE(!FruitFactory::canCreate("orange"));

  // Default creator
  FruitFactory::registerCreator<Apple>("apple");
  FruitFactory::registerCreator(
      "banana", []() -> Banana* { return new Banana(); });
  FruitFactory::registerCreator("orange", createOrange);
}
