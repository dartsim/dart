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

#include <dart/simulation/experimental/common/logging.hpp>

#include <gtest/gtest.h>

using namespace dart::simulation::experimental::common;

// Test logging initialization
TEST(Logging, Initialization)
{
  // Initialize logging (should be idempotent)
  initializeLogging();
  initializeLogging(); // Should not crash on second call

  // Basic logging test (just ensure no crashes)
  DART_EXPERIMENTAL_INFO("Info message");
  DART_EXPERIMENTAL_WARN("Warning message");
  DART_EXPERIMENTAL_DEBUG("Debug message");
}

// Test conditional logging
TEST(Logging, ConditionalLogging)
{
  initializeLogging();

  // Test conditional logging
  DART_EXPERIMENTAL_INFO_IF(true, "This should log");
  DART_EXPERIMENTAL_INFO_IF(false, "This should not log");
}

// Test log once functionality
TEST(Logging, LogOnce)
{
  initializeLogging();

  // Test log once - should only appear once despite being in a loop
  for (int i = 0; i < 5; ++i) {
    DART_EXPERIMENTAL_WARN_ONCE("This should only appear once");
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
  DART_EXPERIMENTAL_TRACE("Trace message");
  DART_EXPERIMENTAL_DEBUG("Debug message");
  DART_EXPERIMENTAL_INFO("Info message");
  DART_EXPERIMENTAL_WARN("Warning message");
  DART_EXPERIMENTAL_ERROR("Error message");
  DART_EXPERIMENTAL_CRITICAL("Critical message");
}
