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

#include <dart/common/exception.hpp>

#include <gtest/gtest.h>

using namespace dart::common;

TEST(Exception, BasicException)
{
  try {
    throw Exception("Test error message");
  } catch (const Exception& e) {
    EXPECT_EQ(e.message(), "Test error message");
    EXPECT_NE(
        std::string(e.what()).find("Test error message"), std::string::npos);
  }
}

TEST(Exception, ExceptionWithFormat)
{
  try {
    DART_THROW("Value {} is invalid", 42);
  } catch (const Exception& e) {
    EXPECT_EQ(e.message(), "Value 42 is invalid");
  }
}

TEST(Exception, InvalidArgumentException)
{
  EXPECT_THROW(
      { DART_THROW_T(InvalidArgumentException, "Bad argument"); },
      InvalidArgumentException);
}

TEST(Exception, OutOfRangeException)
{
  EXPECT_THROW(
      { DART_THROW_T(OutOfRangeException, "Index out of range"); },
      OutOfRangeException);
}

TEST(Exception, NullPointerException)
{
  EXPECT_THROW(
      { DART_THROW_T(NullPointerException, "Null pointer"); },
      NullPointerException);
}

TEST(Exception, NotImplementedException)
{
  EXPECT_THROW(
      { DART_THROW_T(NotImplementedException, "Not implemented"); },
      NotImplementedException);
}

TEST(Exception, InvalidOperationException)
{
  EXPECT_THROW(
      { DART_THROW_T(InvalidOperationException, "Invalid operation"); },
      InvalidOperationException);
}

TEST(Exception, FileNotFoundException)
{
  EXPECT_THROW(
      { DART_THROW_T(FileNotFoundException, "File not found"); },
      FileNotFoundException);
}

TEST(Exception, ParseException)
{
  EXPECT_THROW(
      { DART_THROW_T(ParseException, "Parse error"); }, ParseException);
}

TEST(Exception, ThrowIfConditionTrue)
{
  int x = -1;
  EXPECT_THROW({ DART_THROW_IF(x < 0, "x must be non-negative"); }, Exception);
}

TEST(Exception, NoThrowIfConditionFalse)
{
  int x = 1;
  EXPECT_NO_THROW({ DART_THROW_IF(x < 0, "x must be non-negative"); });
}

TEST(Exception, ThrowTypedIfConditionTrue)
{
  int index = 10;
  int size = 5;
  EXPECT_THROW(
      {
        DART_THROW_T_IF(
            index >= size, OutOfRangeException, "Index out of range");
      },
      OutOfRangeException);
}

TEST(Exception, NoThrowTypedIfConditionFalse)
{
  int index = 2;
  int size = 5;
  EXPECT_NO_THROW({
    DART_THROW_T_IF(index >= size, OutOfRangeException, "Index out of range");
  });
}

TEST(Exception, ExceptionInheritsFromRuntimeError)
{
  try {
    throw InvalidArgumentException("Test");
  } catch (const std::runtime_error& e) {
    SUCCEED();
    return;
  }
  FAIL() << "Exception should inherit from std::runtime_error";
}

TEST(Exception, SourceLocationIncluded)
{
  try {
    throw Exception("Test");
  } catch (const Exception& e) {
    EXPECT_GT(e.location().line(), 0);
    EXPECT_NE(
        std::string(e.location().file_name()).find("test_Exception.cpp"),
        std::string::npos);
  }
}
