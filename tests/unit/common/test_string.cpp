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

#include <dart/common/string.hpp>

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

//==============================================================================
TEST(StringTest, TrimEdgeCases)
{
  // Empty string
  EXPECT_EQ(common::trim(""), "");
  EXPECT_EQ(common::trimLeft(""), "");
  EXPECT_EQ(common::trimRight(""), "");

  // All whitespace
  EXPECT_EQ(common::trim("   "), "");
  EXPECT_EQ(common::trimLeft("   "), "");
  EXPECT_EQ(common::trimRight("   "), "");

  // No whitespace
  EXPECT_EQ(common::trim("hello"), "hello");
  EXPECT_EQ(common::trimLeft("hello"), "hello");
  EXPECT_EQ(common::trimRight("hello"), "hello");

  // Only left whitespace
  EXPECT_EQ(common::trim("  hello"), "hello");
  EXPECT_EQ(common::trimLeft("  hello"), "hello");

  // Only right whitespace
  EXPECT_EQ(common::trim("hello  "), "hello");
  EXPECT_EQ(common::trimRight("hello  "), "hello");

  // Mixed whitespace types
  EXPECT_EQ(common::trim("\t\n hello \n\t"), "hello");
}

//==============================================================================
TEST(StringTest, CaseConversionEdgeCases)
{
  // Empty string
  EXPECT_EQ(common::toUpper(""), "");
  EXPECT_EQ(common::toLower(""), "");

  // Numbers and special characters (should pass through unchanged)
  EXPECT_EQ(common::toUpper("123!@#"), "123!@#");
  EXPECT_EQ(common::toLower("123!@#"), "123!@#");

  // Already in target case
  EXPECT_EQ(common::toUpper("HELLO"), "HELLO");
  EXPECT_EQ(common::toLower("hello"), "hello");

  // In-place with empty string
  std::string empty;
  common::toUpperInPlace(empty);
  EXPECT_EQ(empty, "");
  common::toLowerInPlace(empty);
  EXPECT_EQ(empty, "");
}

//==============================================================================
TEST(StringTest, SplitEdgeCases)
{
  // Empty string
  auto emptyResult = common::split("");
  EXPECT_TRUE(emptyResult.empty());

  // No delimiter found
  auto noDelim = common::split("hello");
  ASSERT_EQ(noDelim.size(), 1u);
  EXPECT_EQ(noDelim[0], "hello");

  // Only delimiter
  auto onlyDelim = common::split("   ");
  EXPECT_TRUE(onlyDelim.empty());

  // Multiple consecutive delimiters
  auto multiDelim = common::split("a   b");
  ASSERT_EQ(multiDelim.size(), 2u);
  EXPECT_EQ(multiDelim[0], "a");
  EXPECT_EQ(multiDelim[1], "b");

  // Delimiter at start and end
  auto delimEnds = common::split(" hello world ");
  ASSERT_EQ(delimEnds.size(), 2u);
  EXPECT_EQ(delimEnds[0], "hello");
  EXPECT_EQ(delimEnds[1], "world");
}

//==============================================================================
TEST(StringTest, SplitWithCustomDelimiters)
{
  // Single character delimiter
  auto commaDelim = common::split("a,b,c", ",");
  ASSERT_EQ(commaDelim.size(), 3u);
  EXPECT_EQ(commaDelim[0], "a");
  EXPECT_EQ(commaDelim[1], "b");
  EXPECT_EQ(commaDelim[2], "c");

  // Multiple character delimiter set
  auto multiDelim = common::split("a,b;c:d", ",;:");
  ASSERT_EQ(multiDelim.size(), 4u);
  EXPECT_EQ(multiDelim[0], "a");
  EXPECT_EQ(multiDelim[1], "b");
  EXPECT_EQ(multiDelim[2], "c");
  EXPECT_EQ(multiDelim[3], "d");

  // Tab and newline delimiters
  auto tabNewline = common::split("hello\tworld\nfoo", "\t\n");
  ASSERT_EQ(tabNewline.size(), 3u);
  EXPECT_EQ(tabNewline[0], "hello");
  EXPECT_EQ(tabNewline[1], "world");
  EXPECT_EQ(tabNewline[2], "foo");

  // Single token (no delimiter in string)
  auto singleToken = common::split("hello", ",");
  ASSERT_EQ(singleToken.size(), 1u);
  EXPECT_EQ(singleToken[0], "hello");

  // Empty delimiter (should return whole string as single token)
  auto emptyDelim = common::split("hello", "");
  ASSERT_EQ(emptyDelim.size(), 1u);
  EXPECT_EQ(emptyDelim[0], "hello");
}

//==============================================================================
TEST(StringTest, TrimWithCustomWhitespaces)
{
  // Custom single character
  EXPECT_EQ(common::trim("###hello###", "#"), "hello");
  EXPECT_EQ(common::trimLeft("###hello###", "#"), "hello###");
  EXPECT_EQ(common::trimRight("###hello###", "#"), "###hello");

  // Custom multiple characters
  EXPECT_EQ(common::trim("xyzHELLOxyz", "xyz"), "HELLO");
  EXPECT_EQ(common::trimLeft("xyzHELLOxyz", "xyz"), "HELLOxyz");
  EXPECT_EQ(common::trimRight("xyzHELLOxyz", "xyz"), "xyzHELLO");

  // Whitespace characters only in custom set
  EXPECT_EQ(common::trim("\t\thello\t\t", "\t"), "hello");
  EXPECT_EQ(common::trim("\n\nhello\n\n", "\n"), "hello");
  EXPECT_EQ(common::trim("\r\rhello\r\r", "\r"), "hello");

  // Empty custom whitespace (should return original)
  EXPECT_EQ(common::trim("  hello  ", ""), "  hello  ");
}

//==============================================================================
TEST(StringTest, CaseConversionMixedContent)
{
  // Mixed alphanumeric
  EXPECT_EQ(common::toUpper("abc123def"), "ABC123DEF");
  EXPECT_EQ(common::toLower("ABC123DEF"), "abc123def");

  // With punctuation
  EXPECT_EQ(common::toUpper("hello, world!"), "HELLO, WORLD!");
  EXPECT_EQ(common::toLower("HELLO, WORLD!"), "hello, world!");

  // Single character
  EXPECT_EQ(common::toUpper("a"), "A");
  EXPECT_EQ(common::toLower("A"), "a");

  // Whitespace preservation
  EXPECT_EQ(common::toUpper("  hello  "), "  HELLO  ");
  EXPECT_EQ(common::toLower("  HELLO  "), "  hello  ");

  // In-place with mixed content
  std::string mixed = "HeLLo123WoRLd!";
  common::toUpperInPlace(mixed);
  EXPECT_EQ(mixed, "HELLO123WORLD!");

  mixed = "HeLLo123WoRLd!";
  common::toLowerInPlace(mixed);
  EXPECT_EQ(mixed, "hello123world!");
}

//==============================================================================
TEST(StringTest, SplitLongStrings)
{
  // Long string with many tokens
  auto manyTokens = common::split("a b c d e f g h i j");
  ASSERT_EQ(manyTokens.size(), 10u);
  EXPECT_EQ(manyTokens[0], "a");
  EXPECT_EQ(manyTokens[9], "j");

  // Long tokens
  std::string longToken = "verylongstringwithoutspaces";
  auto longResult = common::split(longToken);
  ASSERT_EQ(longResult.size(), 1u);
  EXPECT_EQ(longResult[0], longToken);

  // Mixed long and short tokens
  auto mixedLength = common::split("a verylongword b");
  ASSERT_EQ(mixedLength.size(), 3u);
  EXPECT_EQ(mixedLength[0], "a");
  EXPECT_EQ(mixedLength[1], "verylongword");
  EXPECT_EQ(mixedLength[2], "b");
}

//==============================================================================
TEST(StringTest, TrimSpecialCharacters)
{
  // Carriage return
  EXPECT_EQ(common::trim("\r\nhello\r\n"), "hello");
  EXPECT_EQ(common::trimLeft("\r\nhello"), "hello");
  EXPECT_EQ(common::trimRight("hello\r\n"), "hello");

  // Mixed whitespace types
  EXPECT_EQ(common::trim(" \t\n\rhello \t\n\r"), "hello");

  // Only carriage returns
  EXPECT_EQ(common::trim("\r\r\r"), "");

  // Tabs only
  EXPECT_EQ(common::trim("\t\t\t"), "");

  // Single whitespace character
  EXPECT_EQ(common::trim(" "), "");
  EXPECT_EQ(common::trim("\t"), "");
  EXPECT_EQ(common::trim("\n"), "");
  EXPECT_EQ(common::trim("\r"), "");
}
