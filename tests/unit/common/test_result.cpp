/*
 * Copyright (c) 2011-2025, The DART development contributors
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

#include <dart/common/result.hpp>

#include <gtest/gtest.h>

using namespace dart::common;

TEST(Result, OkResult)
{
  auto result = Result<int>::ok(42);
  EXPECT_TRUE(result.isOk());
  EXPECT_FALSE(result.isErr());
  EXPECT_TRUE(static_cast<bool>(result));
  EXPECT_EQ(result.value(), 42);
}

TEST(Result, ErrResult)
{
  auto result = Result<int>::err(Error("Something went wrong"));
  EXPECT_FALSE(result.isOk());
  EXPECT_TRUE(result.isErr());
  EXPECT_FALSE(static_cast<bool>(result));
  EXPECT_EQ(result.error().message, "Something went wrong");
}

TEST(Result, ValueOr)
{
  auto okResult = Result<int>::ok(42);
  auto errResult = Result<int>::err(Error("Error"));

  EXPECT_EQ(okResult.valueOr(0), 42);
  EXPECT_EQ(errResult.valueOr(0), 0);
}

TEST(Result, OperatorStar)
{
  auto result = Result<int>::ok(42);
  EXPECT_EQ(*result, 42);
}

TEST(Result, OperatorArrow)
{
  auto result = Result<std::string>::ok("hello");
  EXPECT_EQ(result->size(), 5);
}

TEST(Result, Map)
{
  auto result = Result<int>::ok(21);
  auto mapped = result.map([](int x) { return x * 2; });

  EXPECT_TRUE(mapped.isOk());
  EXPECT_EQ(mapped.value(), 42);
}

TEST(Result, MapOnError)
{
  auto result = Result<int>::err(Error("Error"));
  auto mapped = result.map([](int x) { return x * 2; });

  EXPECT_TRUE(mapped.isErr());
  EXPECT_EQ(mapped.error().message, "Error");
}

TEST(Result, AndThen)
{
  auto result = Result<int>::ok(21);
  auto chained = result.andThen([](int x) { return Result<int>::ok(x * 2); });

  EXPECT_TRUE(chained.isOk());
  EXPECT_EQ(chained.value(), 42);
}

TEST(Result, AndThenOnError)
{
  auto result = Result<int>::err(Error("Error"));
  auto chained = result.andThen([](int x) { return Result<int>::ok(x * 2); });

  EXPECT_TRUE(chained.isErr());
}

TEST(Result, VoidResultOk)
{
  auto result = Result<void>::ok();
  EXPECT_TRUE(result.isOk());
  EXPECT_FALSE(result.isErr());
}

TEST(Result, VoidResultErr)
{
  auto result = Result<void>::err(Error("Error"));
  EXPECT_FALSE(result.isOk());
  EXPECT_TRUE(result.isErr());
  EXPECT_EQ(result.error().message, "Error");
}

TEST(Result, AccessValueOnErrorThrows)
{
  auto result = Result<int>::err(Error("Error"));
  EXPECT_THROW((void)result.value(), InvalidOperationException);
}

TEST(Result, AccessErrorOnOkThrows)
{
  auto result = Result<int>::ok(42);
  EXPECT_THROW((void)result.error(), InvalidOperationException);
}

TEST(Result, MoveSemantics)
{
  auto result = Result<std::unique_ptr<int>>::ok(std::make_unique<int>(42));
  EXPECT_TRUE(result.isOk());

  auto ptr = std::move(result).value();
  EXPECT_EQ(*ptr, 42);
}

TEST(Result, StringResult)
{
  auto result = Result<std::string>::ok("hello world");
  EXPECT_TRUE(result.isOk());
  EXPECT_EQ(result.value(), "hello world");
}

TEST(Result, PointerResult)
{
  int value = 42;
  auto result = Result<int*>::ok(&value);
  EXPECT_TRUE(result.isOk());
  EXPECT_EQ(*result.value(), 42);
}

TEST(Result, NullPointerResult)
{
  auto result = Result<int*>::ok(nullptr);
  EXPECT_TRUE(result.isOk());
  EXPECT_EQ(result.value(), nullptr);
}
