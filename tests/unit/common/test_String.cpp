/*
 * Copyright (c) 2011-2022, The DART development contributors
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

#include <dart/common/String.hpp>
#include <gtest/gtest.h>

using namespace dart;

//==============================================================================
TEST(StringTest, CaseConversions)
{
  EXPECT_EQ(common::toUpper("to UppEr"), "TO UPPER");
  EXPECT_EQ(common::toLower("to LowEr"), "to lower");

  std::string str;

  str = "toUppEr";
  common::toUpperInPlace(str);
  EXPECT_EQ(str, "TOUPPER");

  str = "toLowEr";
  common::toLowerInPlace(str);
  EXPECT_EQ(str, "tolower");
}

//==============================================================================
TEST(StringTest, Trim)
{
  EXPECT_EQ(common::trimLeft(" trim ThIs "), "trim ThIs ");
  EXPECT_EQ(common::trimRight(" trim ThIs "), " trim ThIs");
  EXPECT_EQ(common::trim(" trim ThIs "), "trim ThIs");

  EXPECT_EQ(common::trimLeft("\n trim ThIs ", " "), "\n trim ThIs ");
  EXPECT_EQ(common::trimLeft("\n trim ThIs ", "\n"), " trim ThIs ");
  EXPECT_EQ(common::trimRight(" trim ThIs \n", " "), " trim ThIs \n");
  EXPECT_EQ(common::trimRight(" trim ThIs \n", "\n"), " trim ThIs ");
  EXPECT_EQ(common::trim("\n trim ThIs \n", " "), "\n trim ThIs \n");
  EXPECT_EQ(common::trim("\n trim ThIs \n", "\n"), " trim ThIs ");

  EXPECT_EQ(common::trimLeft("\n trim ThIs \n", " \n"), "trim ThIs \n");
  EXPECT_EQ(common::trimRight("\n trim ThIs \n", " \n"), "\n trim ThIs");
  EXPECT_EQ(common::trim("\n trim ThIs \n", " \n"), "trim ThIs");
}

//==============================================================================
TEST(StringTest, Split)
{
  ASSERT_EQ(common::split(" trim ThIs ").size(), 2);
  EXPECT_EQ(common::split(" trim ThIs ")[0], "trim");
  EXPECT_EQ(common::split(" trim ThIs ")[1], "ThIs");

  ASSERT_EQ(common::split("ThiSisaNApPle ", "a").size(), 2);
  ASSERT_EQ(common::split("ThiSisaNApPle ", "a")[0], "ThiSis");
  ASSERT_EQ(common::split("ThiSisaNApPle ", "a")[1], "NApPle ");

  ASSERT_EQ(common::split("ThiSisaNApPle ", "A").size(), 2);
  ASSERT_EQ(common::split("ThiSisaNApPle ", "A")[0], "ThiSisaN");
  ASSERT_EQ(common::split("ThiSisaNApPle ", "A")[1], "pPle ");
}
