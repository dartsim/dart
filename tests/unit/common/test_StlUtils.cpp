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

#include "dart/common/StlUtils.hpp"

#include <gtest/gtest.h>

#include <list>
#include <vector>

using namespace dart;
using namespace common;

//==============================================================================
GTEST_TEST(StlUtilsTest, RangeEndOnly)
{
  const std::vector<int> expected = {0, 1, 2, 3, 4, 5};
  std::vector<int> result;
  for (const auto i : Range(6)) {
    result.push_back(i);
  }
  EXPECT_EQ(result, expected);
}

//==============================================================================
GTEST_TEST(StlUtilsTest, RangeStartEnd)
{
  const std::vector<int> expected = {0, 2, 4, 6, 8};
  std::vector<int> result;
  for (const auto i : Range(0, 10, 2)) {
    result.push_back(i);
  }
  EXPECT_EQ(result, expected);
}

//==============================================================================
GTEST_TEST(StlUtilsTest, NonZeroStart)
{
  // Test range(end)
  std::vector<int> expected1{1, 2, 3, 4};
  int i = 0;
  for (auto x : Range(1, 5)) {
    ASSERT_EQ(x, expected1[i]);
    i++;
  }

  // Test range(start, end)
  std::vector<int> expected2{2, 4, 6};
  i = 0;
  for (auto x : Range(2, 7, 2)) {
    ASSERT_EQ(x, expected2[i]);
    i++;
  }

  // Test range(start, end, step)
  std::vector<int> expected3{3, 6, 9, 12};
  i = 0;
  for (auto x : Range(3, 13, 3)) {
    ASSERT_EQ(x, expected3[i]);
    i++;
  }

  // Test negative step
  std::vector<int> expected4{10, 7, 4, 1};
  i = 0;
  for (auto x : Range(10, 0, -3)) {
    ASSERT_EQ(x, expected4[i]);
    i++;
  }

  // Test end < start with negative step
  std::vector<int> expected5{5, 4};
  i = 0;
  for (auto x : Range(5, 3, -1)) {
    ASSERT_EQ(x, expected5[i]);
    i++;
  }
}

//==============================================================================
GTEST_TEST(StlUtilsTest, RangeStartEndNegative)
{
  const std::vector<int> expected = {0, -2, -4, -6, -8};
  std::vector<int> result;
  for (const auto i : Range(0, -10, -2)) {
    result.push_back(i);
  }
  EXPECT_EQ(result, expected);
}

//==============================================================================
GTEST_TEST(StlUtilsTest, EnumerateVector)
{
  std::vector<int> v = {1, 2, 3};
  std::vector<std::pair<size_t, int>> expected = {{0, 1}, {1, 2}, {2, 3}};

  auto result = Enumerate(v);
  ASSERT_EQ(expected.size(), std::distance(result.begin(), result.end()));

  for (const auto& [index, value] : result) {
    ASSERT_EQ(expected[index], std::make_pair(index, value));
  }
}

//==============================================================================
GTEST_TEST(StlUtilsTest, EnumerateList)
{
  std::list<double> l = {1.1, 2.2, 3.3, 4.4};
  std::vector<std::pair<size_t, double>> expected
      = {{0, 1.1}, {1, 2.2}, {2, 3.3}, {3, 4.4}};

  auto result = Enumerate(l);
  ASSERT_EQ(expected.size(), std::distance(result.begin(), result.end()));

  for (const auto& [index, value] : result) {
    ASSERT_EQ(expected[index], std::make_pair(index, value));
  }
}

//==============================================================================
GTEST_TEST(StlUtilsTest, EnumerateEmptyContainer)
{
  std::vector<int> v;
  auto result = Enumerate(v);
  ASSERT_EQ(result.begin(), result.end());
}
