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

#include <dart/common/Singleton.hpp>

#include <gtest/gtest.h>

using namespace dart;
using namespace dart::common;

// Test singleton class with default constructor
class SimpleSingleton : public Singleton<SimpleSingleton>
{
public:
  int getValue() const
  {
    return mValue;
  }
  void setValue(int val)
  {
    mValue = val;
  }

protected:
  friend class Singleton<SimpleSingleton>;
  SimpleSingleton() : mValue(42) {}

private:
  int mValue;
};

// Test singleton class with constructor arguments
class ConfiguredSingleton : public Singleton<ConfiguredSingleton>
{
public:
  int getInitialValue() const
  {
    return mInitialValue;
  }
  const std::string& getName() const
  {
    return mName;
  }

protected:
  friend class Singleton<ConfiguredSingleton>;
  ConfiguredSingleton(int value = 0, std::string name = "default")
    : mInitialValue(value), mName(std::move(name))
  {
  }

private:
  int mInitialValue;
  std::string mName;
};

// Test singleton class with counter to track instantiation
class CountedSingleton : public Singleton<CountedSingleton>
{
public:
  static int sConstructionCount;

  int getInstanceId() const
  {
    return mInstanceId;
  }

protected:
  friend class Singleton<CountedSingleton>;
  CountedSingleton() : mInstanceId(++sConstructionCount) {}

private:
  int mInstanceId;
};

int CountedSingleton::sConstructionCount = 0;

TEST(Singleton, GetSingletonReturnsReference)
{
  SimpleSingleton& singleton = SimpleSingleton::getSingleton();
  EXPECT_EQ(singleton.getValue(), 42);
}

TEST(Singleton, GetSingletonPtrReturnsPointer)
{
  SimpleSingleton* singletonPtr = SimpleSingleton::getSingletonPtr();
  EXPECT_NE(singletonPtr, nullptr);
  EXPECT_EQ(singletonPtr->getValue(), 42);
}

TEST(Singleton, SameInstanceReturned)
{
  SimpleSingleton& singleton1 = SimpleSingleton::getSingleton();
  SimpleSingleton& singleton2 = SimpleSingleton::getSingleton();
  EXPECT_EQ(&singleton1, &singleton2);
}

TEST(Singleton, SameInstanceFromRefAndPtr)
{
  SimpleSingleton& singleton = SimpleSingleton::getSingleton();
  SimpleSingleton* singletonPtr = SimpleSingleton::getSingletonPtr();
  EXPECT_EQ(&singleton, singletonPtr);
}

TEST(Singleton, StateIsPersistent)
{
  SimpleSingleton& singleton1 = SimpleSingleton::getSingleton();
  singleton1.setValue(100);

  SimpleSingleton& singleton2 = SimpleSingleton::getSingleton();
  EXPECT_EQ(singleton2.getValue(), 100);
}

TEST(Singleton, ConstructedOnlyOnce)
{
  int initialCount = CountedSingleton::sConstructionCount;

  CountedSingleton& singleton1 = CountedSingleton::getSingleton();
  int firstId = singleton1.getInstanceId();

  CountedSingleton& singleton2 = CountedSingleton::getSingleton();
  int secondId = singleton2.getInstanceId();

  // Both should have the same instance ID
  EXPECT_EQ(firstId, secondId);

  // Construction count should only increment by 1
  EXPECT_EQ(CountedSingleton::sConstructionCount, initialCount + 1);
}

TEST(Singleton, WithConstructorArguments)
{
  // First call initializes with arguments
  ConfiguredSingleton& singleton
      = ConfiguredSingleton::getSingleton(99, "custom");
  EXPECT_EQ(singleton.getInitialValue(), 99);
  EXPECT_EQ(singleton.getName(), "custom");
}

TEST(Singleton, SubsequentCallsIgnoreArguments)
{
  // Get the existing singleton (arguments should be ignored)
  ConfiguredSingleton& singleton
      = ConfiguredSingleton::getSingleton(999, "ignored");

  EXPECT_NE(singleton.getInitialValue(), 999);
  EXPECT_NE(singleton.getName(), "ignored");
}
