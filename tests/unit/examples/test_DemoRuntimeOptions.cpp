/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * This file is provided under the "BSD-style" License.
 */

#include "../../../examples/demos/RuntimeOptions.hpp"

#include <gtest/gtest.h>

#include <array>

#include <cstddef>

namespace {

using dart_demos::detail::parseSimulationThreadCount;

//==============================================================================
TEST(DemoRuntimeOptions, AcceptsDocumentedThreadCountRange)
{
  std::size_t threadCount = 99u;

  EXPECT_TRUE(parseSimulationThreadCount("0", threadCount));
  EXPECT_EQ(threadCount, 0u);
  EXPECT_TRUE(parseSimulationThreadCount("1", threadCount));
  EXPECT_EQ(threadCount, 1u);
  EXPECT_TRUE(parseSimulationThreadCount("256", threadCount));
  EXPECT_EQ(threadCount, dart_demos::kMaxSimulationThreads);
}

//==============================================================================
TEST(DemoRuntimeOptions, RejectsMalformedAndOutOfRangeThreadCounts)
{
  constexpr std::array<const char*, 9> invalidValues{
      nullptr,
      "",
      "-1",
      "+1",
      " 1",
      "1worker",
      "257",
      "18446744073709551615",
      "18446744073709551616"};

  for (const char* value : invalidValues) {
    std::size_t threadCount = 42u;
    EXPECT_FALSE(parseSimulationThreadCount(value, threadCount));
    EXPECT_EQ(threadCount, 42u);
  }
}

} // namespace
