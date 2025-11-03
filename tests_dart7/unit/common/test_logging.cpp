/*
 * Copyright (c) 2011-2025, The DART development contributors
 * All rights reserved.
 */

#include <dart7/common/logging.hpp>

#include <gtest/gtest.h>

using namespace dart7::common;

// Test logging initialization
TEST(Logging, Initialization)
{
  // Initialize logging (should be idempotent)
  initializeLogging();
  initializeLogging(); // Should not crash on second call

  // Basic logging test (just ensure no crashes)
  DART7_INFO("Info message");
  DART7_WARN("Warning message");
  DART7_DEBUG("Debug message");
}

// Test conditional logging
TEST(Logging, ConditionalLogging)
{
  initializeLogging();

  // Test conditional logging
  DART7_INFO_IF(true, "This should log");
  DART7_INFO_IF(false, "This should not log");
}

// Test log once functionality
TEST(Logging, LogOnce)
{
  initializeLogging();

  // Test log once - should only appear once despite being in a loop
  for (int i = 0; i < 5; ++i) {
    DART7_WARN_ONCE("This should only appear once");
  }
}

// Test log level setting and getting
TEST(Logging, LogLevel)
{
  initializeLogging();

  // Default level should be Info
  EXPECT_EQ(getLogLevel(), LogLevel::Info);

  // Set to Debug
  setLogLevel(LogLevel::Debug);
  EXPECT_EQ(getLogLevel(), LogLevel::Debug);

  // Set to Error
  setLogLevel(LogLevel::Error);
  EXPECT_EQ(getLogLevel(), LogLevel::Error);

  // Set to Off
  setLogLevel(LogLevel::Off);
  EXPECT_EQ(getLogLevel(), LogLevel::Off);

  // Restore to Info
  setLogLevel(LogLevel::Info);
  EXPECT_EQ(getLogLevel(), LogLevel::Info);
}

// Test all log levels
TEST(Logging, AllLevels)
{
  initializeLogging();

  // Test all log level macros (just ensure no crashes)
  DART7_TRACE("Trace message");
  DART7_DEBUG("Debug message");
  DART7_INFO("Info message");
  DART7_WARN("Warning message");
  DART7_ERROR("Error message");
  DART7_CRITICAL("Critical message");
}
