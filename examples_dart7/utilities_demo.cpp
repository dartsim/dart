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

#include <dart7/common/assert.hpp>
#include <dart7/common/exceptions.hpp>
#include <dart7/common/logging.hpp>
#include <dart7/common/profiling.hpp>

#include <iostream>

void demonstrateLogging()
{
  std::cout << "\n=== Logging Demo ===\n";

  DART7_INFO("This is an info message");
  DART7_WARN("This is a warning message");
  DART7_ERROR("This is an error message");

  // Conditional logging
  int value = 42;
  DART7_INFO_IF(value > 0, "Value {} is positive", value);

  // Log once in a loop
  for (int i = 0; i < 5; ++i) {
    DART7_WARN_ONCE("This warning only appears once, even in a loop");
  }
}

void demonstrateExceptions()
{
  std::cout << "\n=== Exception Demo ===\n";

  try {
    DART7_THROW(
        dart7::common::InvalidArgumentException, "Invalid value provided");
  } catch (const dart7::common::Exception& e) {
    std::cout << "Caught exception:\n" << e.what() << "\n";
  }

  // Conditional throw
  int age = -5;
  try {
    DART7_THROW_IF(
        age < 0,
        dart7::common::InvalidArgumentException,
        "Age cannot be negative");
  } catch (const dart7::common::Exception& e) {
    std::cout << "Caught conditional exception\n";
  }
}

double slowFunction()
{
  DART7_PROFILE_FUNCTION();

  double result = 0.0;
  for (int i = 0; i < 1000000; ++i) {
    result += std::sin(i * 0.001);
  }
  return result;
}

void demonstrateProfiling()
{
  std::cout << "\n=== Profiling Demo ===\n";

  dart7::common::ProfileStats::reset();

  {
    DART7_PROFILE_SCOPE("computation_block");
    double sum = slowFunction();
    std::cout << "Computed sum: " << sum << "\n";
  }

  {
    DART7_PROFILE_SCOPE("simple_loop");
    volatile int total = 0;
    for (int i = 0; i < 100000; ++i) {
      total += i;
    }
  }

#ifdef DART7_ENABLE_PROFILING
  std::cout << "\nProfile results:\n";
  dart7::common::ProfileStats::printSummary();
#else
  std::cout << "Profiling is disabled. Build with -DDART_ENABLE_PROFILING to "
               "enable.\n";
#endif
}

void demonstrateAssertions()
{
  std::cout << "\n=== Assertion Demo ===\n";

  int x = 10;
  DART7_ASSERT(x > 0);
  std::cout << "Assertion passed: x > 0\n";

  DART7_ASSERT_MSG(x == 10, "x should be 10");
  std::cout << "Assertion with message passed\n";

  // Debug assertions only run in debug builds
  DART7_DEBUG_ASSERT(x < 100);
  std::cout << "Debug assertion completed (only active in debug builds)\n";
}

int main()
{
  // Initialize logging
  dart7::common::initializeLogging();
  dart7::common::Logger::setLevel(spdlog::level::debug);

  std::cout << "DART7 Utilities Demo\n";
  std::cout << "====================\n";

  demonstrateLogging();
  demonstrateExceptions();
  demonstrateProfiling();
  demonstrateAssertions();

  std::cout << "\nDemo completed successfully!\n";

  return 0;
}
