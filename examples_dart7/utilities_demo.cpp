/*
 * Copyright (c) 2011-2025, The DART development contributors
 * All rights reserved.
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
