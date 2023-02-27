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

#include "dart/common/TemplateUtils.hpp"

#include <gtest/gtest.h>

#include <list>
#include <queue>

using namespace dart;
using namespace common;

GTEST_TEST(TemplateUtilsTest, Basics)
{
  std::vector<int> vec;
  const std::vector<int> cvec;

  EXPECT_TRUE(is_contiguous_iterator_v<decltype(vec.begin())>);
  EXPECT_TRUE(is_contiguous_iterator_v<decltype(cvec.begin())>);
  EXPECT_FALSE(is_const_contiguous_iterator_v<decltype(vec.begin())>);
  EXPECT_TRUE(is_const_contiguous_iterator_v<decltype(cvec.begin())>);
  EXPECT_TRUE(is_non_const_contiguous_iterator_v<decltype(vec.begin())>);
  EXPECT_FALSE(is_non_const_contiguous_iterator_v<decltype(cvec.begin())>);
}

// Test is_contiguous_iterator
GTEST_TEST(TemplateUtilsTest, IsContiguousIterator)
{
  EXPECT_TRUE((is_contiguous_iterator<int*>::value));
  EXPECT_TRUE((is_contiguous_iterator<std::vector<int>::iterator>::value));
  EXPECT_FALSE((is_contiguous_iterator<std::list<int>::iterator>::value));
  EXPECT_FALSE((is_contiguous_iterator<
                std::back_insert_iterator<std::vector<int>>>::value));
}

// Test is_const_contiguous_iterator
GTEST_TEST(TemplateUtilsTest, IsConstContiguousIterator)
{
  EXPECT_TRUE((is_const_contiguous_iterator<const int*>::value));
  EXPECT_TRUE(
      (is_const_contiguous_iterator<std::vector<int>::const_iterator>::value));
  EXPECT_FALSE((is_const_contiguous_iterator<int*>::value));
  EXPECT_FALSE(
      (is_const_contiguous_iterator<std::vector<int>::iterator>::value));
  EXPECT_FALSE((is_const_contiguous_iterator<std::list<int>::iterator>::value));
}

// Test is_non_const_contiguous_iterator
GTEST_TEST(TemplateUtilsTest, IsNonConstContiguousIterator)
{
  EXPECT_FALSE((is_non_const_contiguous_iterator<const int*>::value));
  EXPECT_FALSE((is_non_const_contiguous_iterator<
                std::vector<int>::const_iterator>::value));
  EXPECT_TRUE((is_non_const_contiguous_iterator<int*>::value));
  EXPECT_TRUE(
      (is_non_const_contiguous_iterator<std::vector<int>::iterator>::value));
  EXPECT_FALSE(
      (is_non_const_contiguous_iterator<std::list<int>::iterator>::value));
}

// Test is_contiguous_container
GTEST_TEST(TemplateUtilsTest, IsContiguousContainer)
{
  EXPECT_TRUE((is_contiguous_container<std::vector<int>>::value));
  EXPECT_TRUE((is_contiguous_container<std::array<int, 10>>::value));
  EXPECT_FALSE((is_contiguous_container<std::list<int>>::value));
  EXPECT_FALSE((is_contiguous_container<std::deque<int>>::value));
}

// Test FunctionTraits for a function pointer
GTEST_TEST(TemplateUtilsTest, FunctionTraits_FunctionPointer)
{
  auto func = [](int, bool, double) {
    return 42;
  };
  using traits = FunctionTraits<decltype(func)>;
  EXPECT_EQ(traits::num_args, 3u);
  EXPECT_EQ(
      (std::is_same_v<typename traits::args, std::tuple<int, bool, double>>),
      true);
  EXPECT_EQ((std::is_same_v<typename traits::arg<0>::type, int>), true);
  EXPECT_EQ((std::is_same_v<typename traits::arg<1>::type, bool>), true);
  EXPECT_EQ((std::is_same_v<typename traits::arg<2>::type, double>), true);
  EXPECT_EQ((std::is_same_v<typename traits::return_type, int>), true);
}

// Test FunctionTraits for a non-const member function pointer
GTEST_TEST(TemplateUtilsTest, FunctionTraits_MemberFunctionPointer)
{
  struct Foo
  {
    int bar(bool, double)
    {
      return 42;
    }
  };
  using traits = FunctionTraits<decltype(&Foo::bar)>;
  EXPECT_EQ(traits::num_args, 2u);
  EXPECT_EQ(
      (std::is_same_v<typename traits::args, std::tuple<bool, double>>), true);
  EXPECT_EQ((std::is_same_v<typename traits::arg<0>::type, bool>), true);
  EXPECT_EQ((std::is_same_v<typename traits::arg<1>::type, double>), true);
  EXPECT_EQ((std::is_same_v<typename traits::return_type, int>), true);
}

// Test FunctionTraits for a const member function pointer
GTEST_TEST(TemplateUtilsTest, FunctionTraits_ConstMemberFunctionPointer)
{
  struct Foo
  {
    int bar(bool, double) const
    {
      return 42;
    }
  };
  using traits = FunctionTraits<decltype(&Foo::bar)>;
  EXPECT_EQ(traits::num_args, 2u);
  EXPECT_EQ(
      (std::is_same_v<typename traits::args, std::tuple<bool, double>>), true);
  EXPECT_EQ((std::is_same_v<typename traits::arg<0>::type, bool>), true);
  EXPECT_EQ((std::is_same_v<typename traits::arg<1>::type, double>), true);
  EXPECT_EQ((std::is_same_v<typename traits::return_type, int>), true);
}
