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
  using FactoryEnum = common::Factory<ObjectTypeEnum, Fruit>;
  using FactoryEnumClass = common::Factory<ObjectTypeEnumClass, Fruit>;
  using FactoryString = common::Factory<std::string, Fruit>;

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

class City
{
public:
  using Factory = common::Factory<std::string, City, std::shared_ptr, int>;

  City(int year) : mYear(year) {}

  virtual std::string getName() const = 0;

  int getYear() const { return mYear; }

protected:
  int mYear;
};

class Atlanta : public City
{
public:
  Atlanta(int year) : City(year) {}

  std::string getName() const override
  {
    return "Atlanta";
  }
};

class Pittsburgh : public City
{
public:
  Pittsburgh(int year) : City(year) {}

  std::string getName() const override
  {
    return "Pittsburgh";
  }
};

class Seattle : public City
{
public:
  Seattle(int year) : City(year) {}

  std::string getName() const override
  {
    return "Seattle";
  }
};

DART_REGISTER_DEFAULT_UNIQUE_PTR_CREATOR_TO_FACTORY(ObjectTypeEnum, OT_Apple, Fruit, Apple)
DART_REGISTER_DEFAULT_UNIQUE_PTR_CREATOR_TO_FACTORY(ObjectTypeEnum, OT_Banana, Fruit, Banana)

DART_REGISTER_DEFAULT_UNIQUE_PTR_CREATOR_TO_FACTORY(
    ObjectTypeEnumClass, ObjectTypeEnumClass::Apple, Fruit, Apple)
DART_REGISTER_DEFAULT_UNIQUE_PTR_CREATOR_TO_FACTORY(
    ObjectTypeEnumClass, ObjectTypeEnumClass::Banana, Fruit, Banana)

DART_REGISTER_DEFAULT_UNIQUE_PTR_CREATOR_TO_FACTORY(std::string, "apple", Fruit, Apple)
DART_REGISTER_DEFAULT_UNIQUE_PTR_CREATOR_TO_FACTORY(std::string, "banana", Fruit, Banana)

//==============================================================================
TEST(Factory, Create)
{
  //----------------------------------------------------------------------------
  // enum class key type
  //----------------------------------------------------------------------------
  EXPECT_TRUE(!Fruit::FactoryEnum::create(OT_None));

  EXPECT_TRUE(Fruit::FactoryEnum::canCreate(OT_Apple));
  EXPECT_EQ(Fruit::FactoryEnum::create(OT_Apple)->getName(), "apple");

  // Can't create an orange since it's not registered yet.
  EXPECT_TRUE(!Fruit::FactoryEnum::canCreate(OT_Orange));

  // Once it's registered we can create an orange.
  Fruit::FactoryEnum::registerCreator<Orange>(OT_Orange);
  EXPECT_TRUE(Fruit::FactoryEnum::canCreate(OT_Orange));
  EXPECT_EQ(Fruit::FactoryEnum::create(OT_Orange)->getName(), "orange");

  Fruit::FactoryEnum::unregisterCreator(OT_Orange);
  EXPECT_TRUE(!Fruit::FactoryEnum::canCreate(OT_Orange));

  //----------------------------------------------------------------------------
  // enum class key type
  //----------------------------------------------------------------------------
  EXPECT_TRUE(!Fruit::FactoryEnumClass::create(ObjectTypeEnumClass::None));

  EXPECT_TRUE(Fruit::FactoryEnumClass::canCreate(ObjectTypeEnumClass::Apple));
  EXPECT_EQ(Fruit::FactoryEnumClass::create(
      ObjectTypeEnumClass::Apple)->getName(), "apple");

  // Can't create an orange since it's not registered yet.
  EXPECT_TRUE(!Fruit::FactoryEnumClass::canCreate(ObjectTypeEnumClass::Orange));

  // Once it's registered we can create an orange.
  Fruit::FactoryEnumClass::registerCreator<Orange>(ObjectTypeEnumClass::Orange);
  EXPECT_TRUE(Fruit::FactoryEnumClass::canCreate(ObjectTypeEnumClass::Orange));
  EXPECT_EQ(Fruit::FactoryEnumClass::create(
      ObjectTypeEnumClass::Orange)->getName(), "orange");

  Fruit::FactoryEnumClass::unregisterCreator(ObjectTypeEnumClass::Orange);
  EXPECT_TRUE(!Fruit::FactoryEnumClass::canCreate(ObjectTypeEnumClass::Orange));

  //----------------------------------------------------------------------------
  // std::string key type
  //----------------------------------------------------------------------------
  EXPECT_TRUE(!Fruit::FactoryString::create("none"));

  EXPECT_TRUE(Fruit::FactoryString::canCreate("apple"));
  EXPECT_EQ(Fruit::FactoryString::create("apple")->getName(), "apple");

  // Can't create an orange since it's not registered yet.
  EXPECT_TRUE(!Fruit::FactoryString::canCreate("orange"));

  // Once it's registered we can create an orange.
  Fruit::FactoryString::registerCreator<Orange>("orange");
  EXPECT_TRUE(Fruit::FactoryString::canCreate("orange"));
  EXPECT_EQ(Fruit::FactoryString::create("orange")->getName(), "orange");

  Fruit::FactoryString::unregisterCreator("orange");
  EXPECT_TRUE(!Fruit::FactoryString::canCreate("orange"));
}

//==============================================================================
static std::shared_ptr<Orange> createOrange()
{
  return std::shared_ptr<Orange>(new Orange());
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

  // Lambda function
  FruitFactory::registerCreator(
      std::string("banana"), []() -> std::shared_ptr<Banana>
      { return std::shared_ptr<Banana>(new Banana()); });

  // Global static function
  FruitFactory::registerCreator(std::string("orange"), createOrange);
}

//==============================================================================
TEST(Factory, CreateWithArguments)
{
  City::Factory::registerCreator<Atlanta>("Atlanta");
  City::Factory::registerCreator<Pittsburgh>("Pittsburgh");
  City::Factory::registerCreator<Seattle>("Seattle");
  EXPECT_TRUE(City::Factory::canCreate("Atlanta"));
  EXPECT_TRUE(City::Factory::canCreate("Pittsburgh"));
  EXPECT_TRUE(City::Factory::canCreate("Seattle"));

  auto atlanta = City::Factory::create("Atlanta", 2013);
  auto pittsburgh = City::Factory::create("Pittsburgh", 2016);
  auto seatle = City::Factory::create("Seattle", 2017);
  EXPECT_TRUE(atlanta->getYear() == 2013);
  EXPECT_TRUE(pittsburgh->getYear() == 2016);
  EXPECT_TRUE(seatle->getYear() == 2017);
}
