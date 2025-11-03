/*
 * Copyright (c) 2011-2025, The DART development contributors
 * All rights reserved.
 */

#include <dart7/common/assert.hpp>

#include <gtest/gtest.h>

using namespace dart7::common;

// Test assertions
TEST(Assert, BasicAssertions)
{
  // These should not fail
  DART7_ASSERT(true);
  DART7_ASSERT_MSG(true, "This should pass");
  DART7_VERIFY(true);
  DART7_DEBUG_ASSERT(true);
}

// Test VERIFY returns bool
TEST(Assert, VerifyReturnsBool)
{
  // DART7_VERIFY should return the bool value
  bool result = DART7_VERIFY(true);
  EXPECT_TRUE(result);

  // Can be used in if statements
  if (DART7_VERIFY(true)) {
    // This should execute
    SUCCEED();
  } else {
    FAIL() << "DART7_VERIFY should return true";
  }
}
