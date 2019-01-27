/*
 * Copyright (c) 2011-2019, The DART development contributors
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

  using SingletonFactoryEnum = common::Singleton<FactoryEnum>;
  using SingletonFactoryEnumClass = common::Singleton<FactoryEnumClass>;
  using SingletonFactoryString = common::Singleton<FactoryString>;

  template <typename Derived>
  using RegistrarEnum = common::FactoryRegistrar<ObjectTypeEnum, Fruit, Derived, std::shared_ptr<Fruit>>;
  template <typename Derived>
  using RegistrarEnumClass = common::FactoryRegistrar<ObjectTypeEnumClass, Fruit, Derived, std::shared_ptr<Fruit>>;
  template <typename Derived>
  using RegistrarString = common::FactoryRegistrar<std::string, Fruit, Derived, std::shared_ptr<Fruit>>;

  static FactoryEnum* getFactoryEnum()
  {
    return SingletonFactoryEnum::getSingletonPtr();
  }

  static FactoryEnumClass* getFactoryEnumClass()
  {
    return SingletonFactoryEnumClass::getSingletonPtr();
  }

  static FactoryString* getFactoryString()
  {
    return SingletonFactoryString::getSingletonPtr();
  }

  virtual std::string getName() const = 0;
};

class Apple : public Fruit
{
public:
  std::string getName() const override
  {
    return "apple";
  }

  static RegistrarEnum<Apple> mRegistrarEnum;
  static RegistrarEnumClass<Apple> mRegistrarEnumClass;
  static RegistrarString<Apple> mRegistrarString;
};

Apple::RegistrarEnum<Apple> Apple::mRegistrarEnum{OT_Apple};
Apple::RegistrarEnumClass<Apple> Apple::mRegistrarEnumClass{ObjectTypeEnumClass::Apple};
Apple::RegistrarString<Apple> Apple::mRegistrarString{"apple"};

class Banana : public Fruit
{
public:
  std::string getName() const override
  {
    return "banana";
  }

  static RegistrarEnum<Banana> mRegistrarEnum;
  static RegistrarEnumClass<Banana> mRegistrarEnumClass;
  static RegistrarString<Banana> mRegistrarString;
};

Banana::RegistrarEnum<Banana> Banana::mRegistrarEnum{OT_Banana};
Banana::RegistrarEnumClass<Banana> Banana::mRegistrarEnumClass{ObjectTypeEnumClass::Banana};
Banana::RegistrarString<Banana> Banana::mRegistrarString{"banana"};

class Orange : public Fruit
{
public:
  std::string getName() const override
  {
    return "orange";
  }

  static RegistrarEnum<Orange> mRegistrarEnum;
  static RegistrarEnumClass<Orange> mRegistrarEnumClass;
  static RegistrarString<Orange> mRegistrarString;
};

Orange::RegistrarEnum<Orange> Orange::mRegistrarEnum{OT_Orange};
Orange::RegistrarEnumClass<Orange> Orange::mRegistrarEnumClass{ObjectTypeEnumClass::Orange};
Orange::RegistrarString<Orange> Orange::mRegistrarString{"orange"};

class City
{
public:
  using Factory = common::Factory<std::string, City, std::shared_ptr<City>, int>;
  using SingletonFactory = common::Singleton<Factory>;

  template <typename Derived>
  using Registrar = common::FactoryRegistrar<std::string, City, Derived, std::shared_ptr<City>, int>;

  static Factory* getFactory()
  {
    return SingletonFactory::getSingletonPtr();
  }

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

  static Registrar<Atlanta> mRegistrar;
};

City::Registrar<Atlanta> Atlanta::mRegistrar("Atlanta");

class Pittsburgh : public City
{
public:
  Pittsburgh(int year) : City(year) {}

  std::string getName() const override
  {
    return "Pittsburgh";
  }

  static Registrar<Pittsburgh> mRegistrar;
};

City::Registrar<Pittsburgh> Pittsburgh::mRegistrar("Pittsburgh");

class Seattle : public City
{
public:
  Seattle(int year) : City(year) {}

  std::string getName() const override
  {
    return "Seattle";
  }

  static Registrar<Seattle> mRegistrar;
};

City::Registrar<Seattle> Seattle::mRegistrar{"Seattle"};

//==============================================================================
TEST(Factory, Create)
{
  //----------------------------------------------------------------------------
  // enum class key type
  //----------------------------------------------------------------------------
  EXPECT_TRUE(!Fruit::getFactoryEnum()->create(OT_None));

  EXPECT_TRUE(Fruit::getFactoryEnum()->canCreate(OT_Apple));
  EXPECT_EQ(Fruit::getFactoryEnum()->create(OT_Apple)->getName(), "apple");

  // Can't create an orange since it's unregistered.
  Fruit::getFactoryEnum()->unregisterCreator(OT_Orange);
  EXPECT_TRUE(!Fruit::getFactoryEnum()->canCreate(OT_Orange));

  // Once it's registered back we can create an orange.
  Fruit::getFactoryEnum()->registerCreator<Orange>(OT_Orange);
  EXPECT_TRUE(Fruit::getFactoryEnum()->canCreate(OT_Orange));
  EXPECT_EQ(Fruit::getFactoryEnum()->create(OT_Orange)->getName(), "orange");

  Fruit::getFactoryEnum()->unregisterCreator(OT_Orange);
  EXPECT_TRUE(!Fruit::getFactoryEnum()->canCreate(OT_Orange));

  //----------------------------------------------------------------------------
  // enum class key type
  //----------------------------------------------------------------------------
  EXPECT_TRUE(!Fruit::getFactoryEnumClass()->create(ObjectTypeEnumClass::None));

  EXPECT_TRUE(Fruit::getFactoryEnumClass()->canCreate(ObjectTypeEnumClass::Apple));
  EXPECT_EQ(Fruit::getFactoryEnumClass()->create(
      ObjectTypeEnumClass::Apple)->getName(), "apple");

  // Can't create an orange since it's unregistered.
  Fruit::getFactoryEnumClass()->unregisterCreator(ObjectTypeEnumClass::Orange);
  EXPECT_TRUE(!Fruit::getFactoryEnumClass()->canCreate(ObjectTypeEnumClass::Orange));

  // Once it's registered back we can create an orange.
  Fruit::getFactoryEnumClass()->registerCreator<Orange>(ObjectTypeEnumClass::Orange);
  EXPECT_TRUE(Fruit::getFactoryEnumClass()->canCreate(ObjectTypeEnumClass::Orange));
  EXPECT_EQ(Fruit::getFactoryEnumClass()->create(
      ObjectTypeEnumClass::Orange)->getName(), "orange");

  Fruit::getFactoryEnumClass()->unregisterCreator(ObjectTypeEnumClass::Orange);
  EXPECT_TRUE(!Fruit::getFactoryEnumClass()->canCreate(ObjectTypeEnumClass::Orange));

  //----------------------------------------------------------------------------
  // std::string key type
  //----------------------------------------------------------------------------
  EXPECT_TRUE(!Fruit::getFactoryString()->create("none"));

  EXPECT_TRUE(Fruit::getFactoryString()->canCreate("apple"));
  EXPECT_EQ(Fruit::getFactoryString()->create("apple")->getName(), "apple");

  // Can't create an orange since it's unregistered.
  Fruit::getFactoryString()->unregisterCreator("orange");
  EXPECT_TRUE(!Fruit::getFactoryString()->canCreate("orange"));

  // Once it's registered back we can create an orange.
  Fruit::getFactoryString()->registerCreator<Orange>("orange");
  EXPECT_TRUE(Fruit::getFactoryString()->canCreate("orange"));
  EXPECT_EQ(Fruit::getFactoryString()->create("orange")->getName(), "orange");

  Fruit::getFactoryString()->unregisterCreator("orange");
  EXPECT_TRUE(!Fruit::getFactoryString()->canCreate("orange"));
}

//==============================================================================
static std::shared_ptr<Orange> createOrange()
{
  return std::shared_ptr<Orange>(new Orange());
}

//==============================================================================
TEST(Factory, VariousCreatorFunctions)
{
  using FruitFactory = common::Singleton<common::Factory<std::string, Fruit>>;

  FruitFactory::getSingleton().unregisterAllCreators();

  EXPECT_TRUE(!FruitFactory::getSingleton().canCreate("apple"));
  EXPECT_TRUE(!FruitFactory::getSingleton().canCreate("banana"));
  EXPECT_TRUE(!FruitFactory::getSingleton().canCreate("orange"));

  // Default creator
  FruitFactory::getSingleton().registerCreator<Apple>("apple");

  // Lambda function
  FruitFactory::getSingleton().registerCreator(
      std::string("banana"), []() -> std::shared_ptr<Banana>
      { return std::shared_ptr<Banana>(new Banana()); });

  // Global static function
  FruitFactory::getSingleton().registerCreator(std::string("orange"), createOrange);
}

//==============================================================================
TEST(Factory, CreateWithArguments)
{
  EXPECT_TRUE(City::getFactory()->canCreate("Atlanta"));
  EXPECT_TRUE(City::getFactory()->canCreate("Pittsburgh"));
  EXPECT_TRUE(City::getFactory()->canCreate("Seattle"));

  auto atlanta = City::getFactory()->create("Atlanta", 2013);
  auto pittsburgh = City::getFactory()->create("Pittsburgh", 2016);
  auto seatle = City::getFactory()->create("Seattle", 2017);
  EXPECT_TRUE(atlanta->getYear() == 2013);
  EXPECT_TRUE(pittsburgh->getYear() == 2016);
  EXPECT_TRUE(seatle->getYear() == 2017);
}
