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

#include <dart/simulation/common/diagnostics.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <string>
#include <string_view>

using namespace dart::simulation::common;

TEST(Diagnostics, GetCompilerInfo_ReturnsNonEmpty)
{
  std::string info = getCompilerInfo();
  EXPECT_FALSE(info.empty());
  constexpr auto compilerNames
      = std::to_array<std::string_view>({"GCC", "Clang", "MSVC", "Unknown"});
  EXPECT_TRUE(
      std::ranges::any_of(compilerNames, [&](std::string_view compilerName) {
        return info.find(compilerName) != std::string::npos;
      }));
}

TEST(Diagnostics, GetCxxABI_ReturnsNonEmpty)
{
  std::string abi = getCxxABI();
  EXPECT_FALSE(abi.empty());
  constexpr auto abiNames = std::to_array<std::string_view>(
      {"libstdc++", "libc++", "MSVC", "unknown"});
  EXPECT_TRUE(std::ranges::any_of(abiNames, [&](std::string_view abiName) {
    return abi.find(abiName) != std::string::npos;
  }));
}

TEST(Diagnostics, GetCxxStandard_ReturnsValidValue)
{
  long standard = getCxxStandard();
  EXPECT_GE(standard, 201703L);
}

TEST(Diagnostics, GetLibraryPath_ReturnsValidPath)
{
  void* funcPtr = reinterpret_cast<void*>(&getCompilerInfo);
  std::string path = getLibraryPath(funcPtr);
  EXPECT_FALSE(path.empty());
  EXPECT_NE(path, "unsupported platform");
}

TEST(Diagnostics, GetLibraryPath_WithNullptr)
{
  std::string path = getLibraryPath(nullptr);
  SUCCEED();
}

TEST(Diagnostics, PrintRuntimeInfo_DoesNotCrash)
{
  EXPECT_NO_THROW(printRuntimeInfo());
}

TEST(Diagnostics, PrintErrorHelp_DoesNotCrash)
{
  EXPECT_NO_THROW(printErrorHelp("TestSection", "TestError"));
}

TEST(Diagnostics, PrintErrorHelp_WithEmptyStrings)
{
  EXPECT_NO_THROW(printErrorHelp("", ""));
}

TEST(Diagnostics, PrintErrorHelp_WithLongStrings)
{
  std::string longSection(100, 'A');
  std::string longError(100, 'B');
  EXPECT_NO_THROW(printErrorHelp(longSection, longError));
}
