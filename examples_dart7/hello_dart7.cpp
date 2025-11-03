/*
 * Copyright (c) 2011-2025, The DART development contributors
 * All rights reserved.
 */

#include <dart7/common/logging.hpp>
#include <dart7/version.hpp>
#include <dart7/world.hpp>

#include <iostream>

int main()
{
  // Initialize logging
  dart7::common::initializeLogging();

  // Print version info
  std::cout << "DART 7.0 Example\n";
  std::cout << "Version: " << dart7::version() << "\n";
  std::cout << "Major: " << dart7::versionMajor() << "\n";
  std::cout << "Minor: " << dart7::versionMinor() << "\n";
  std::cout << "Patch: " << dart7::versionPatch() << "\n\n";

  // Use logging
  DART7_INFO("Creating world...");
  [[maybe_unused]] dart7::World world;
  DART7_INFO("World created successfully!");

  return 0;
}
