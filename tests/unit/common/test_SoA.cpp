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

#include "dart/common/SoA.hpp"

#include <gtest/gtest.h>

#include <list>
#include <vector>

using namespace dart;
using namespace common;

//==============================================================================
TEST(SoATest, EmptyTest)
{
  SoA<int, double, char> soa;
  EXPECT_TRUE(soa.isEmpty());
  EXPECT_EQ(soa.getSize(), 0);
  EXPECT_EQ(soa.getArray<0>().size(), 0);
  EXPECT_EQ(soa.getArray<1>().size(), 0);
  EXPECT_EQ(soa.getArray<2>().size(), 0);
}

//==============================================================================
TEST(SoATest, BasicTest)
{
  SoA<int, double, char> soa;
  soa.getArray<0>().push_back(1);
  soa.getArray<1>().push_back(2.0);
  soa.getArray<2>().push_back('a');
  EXPECT_FALSE(soa.isEmpty());
  EXPECT_EQ(soa.getSize(), 1);
  EXPECT_EQ(soa.get<0>(0), 1);
  EXPECT_EQ(soa.get<1>(0), 2.0);
  EXPECT_EQ(soa.get<2>(0), 'a');
}

//==============================================================================
TEST(SoATest, IncreaseSizeTest)
{
  SoA<int, double, char> soa;
  EXPECT_TRUE(soa.isEmpty());
  soa.increaseSizeAll(3);
  EXPECT_EQ(soa.getSize(), 3);
  EXPECT_EQ(soa.getArray<0>().size(), 3);
  EXPECT_EQ(soa.getArray<1>().size(), 3);
  EXPECT_EQ(soa.getArray<2>().size(), 3);
}

//==============================================================================
TEST(SoATest, SwapElementsTest)
{
  SoA<int, double, char> soa;
  soa.getArray<0>().push_back(1);
  soa.getArray<1>().push_back(2.0);
  soa.getArray<2>().push_back('a');
  soa.getArray<0>().push_back(3);
  soa.getArray<1>().push_back(4.0);
  soa.getArray<2>().push_back('b');
  soa.getArray<0>().push_back(5);
  soa.getArray<1>().push_back(6.0);
  soa.getArray<2>().push_back('c');
  EXPECT_EQ(soa.get<0>(0), 1);
  EXPECT_EQ(soa.get<1>(0), 2.0);
  EXPECT_EQ(soa.get<2>(0), 'a');
  EXPECT_EQ(soa.get<0>(1), 3);
  EXPECT_EQ(soa.get<1>(1), 4.0);
  EXPECT_EQ(soa.get<2>(1), 'b');
  EXPECT_EQ(soa.get<0>(2), 5);
  EXPECT_EQ(soa.get<1>(2), 6.0);
  EXPECT_EQ(soa.get<2>(2), 'c');
  soa.swapElementsAll(0, 2);
  EXPECT_EQ(soa.get<0>(0), 5);
  EXPECT_EQ(soa.get<1>(0), 6.0);
  EXPECT_EQ(soa.get<2>(0), 'c');
  EXPECT_EQ(soa.get<0>(1), 3);
  EXPECT_EQ(soa.get<1>(1), 4.0);
  EXPECT_EQ(soa.get<2>(1), 'b');
  EXPECT_EQ(soa.get<0>(2), 1);
  EXPECT_EQ(soa.get<1>(2), 2.0);
  EXPECT_EQ(soa.get<2>(2), 'a');
}

//==============================================================================
TEST(SoATest, isEmptyReturnsFalseWhenArraysAreNotEmpty)
{
  SoA<int, float, int> soa;
  std::get<0>(soa.data) = {1, 2, 3};
  std::get<1>(soa.data) = {1.1f, 2.2f, 3.3f};
  std::get<2>(soa.data) = {4, 5, 6};

  EXPECT_FALSE(soa.isEmpty());
}

//==============================================================================
TEST(SoATest, isEmptyReturnsTrueWhenArraysAreEmpty)
{
  SoA<int, float, int> emptySoa;
  EXPECT_TRUE(emptySoa.isEmpty());
}

//==============================================================================
TEST(SoATest, getSizeReturnsSizeOfArrays)
{
  SoA<int, float, int> soa;
  std::get<0>(soa.data) = {1, 2, 3};
  std::get<1>(soa.data) = {1.1f, 2.2f, 3.3f};
  std::get<2>(soa.data) = {4, 5, 6};

  EXPECT_EQ(3, soa.getSize());
}

//==============================================================================
TEST(SoATest, getArrayReturnsConstReferenceToCorrectArray)
{
  SoA<int, float, int> soa;
  std::get<0>(soa.data) = {1, 2, 3};
  std::get<1>(soa.data) = {1.1f, 2.2f, 3.3f};
  std::get<2>(soa.data) = {4, 5, 6};

  const auto& intArray = soa.getArray<0>();
  const auto& floatArray = soa.getArray<1>();
  // const auto& intArray2 = soa.getArray<int>(); // This should not compile

  EXPECT_EQ(std::get<0>(soa.data), intArray);
  EXPECT_EQ(std::get<1>(soa.data), floatArray);
  // EXPECT_EQ(std::get<0>(soa.data), intArray2); // This should not compile
}
