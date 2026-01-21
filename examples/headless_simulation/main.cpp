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

#include <dart/all.hpp>
#include <dart/io/read.hpp>

#include <chrono>
#include <iostream>
#include <limits>
#include <string>
#include <string_view>

#include <cstdlib>

namespace {

struct Options
{
  std::string worldUri = "dart://sample/sdf/double_pendulum.world";
  std::size_t steps = 1000;
  double timeStep = 0.001;
  bool setTimeStep = false;
  unsigned int seed = 42;
};

enum class ParseResult
{
  Ok,
  Help,
  Error
};

void printUsage(const char* argv0)
{
  std::cout << "Usage: " << argv0 << " [options]\n\n"
            << "Options:\n"
            << "  --world <uri>   World URI to load\n"
            << "  --steps <n>     Number of simulation steps\n"
            << "  --dt <seconds>  Override world time step\n"
            << "  --seed <n>      Random seed for deterministic runs\n"
            << "  -h, --help      Show this help\n";
}

bool parseSizeT(std::string_view value, std::size_t& output)
{
  if (value.empty() || value.front() == '-') {
    return false;
  }

  char* end = nullptr;
  const unsigned long long result
      = std::strtoull(std::string(value).c_str(), &end, 10);
  if (!end || *end != '\0') {
    return false;
  }

  if (result > std::numeric_limits<std::size_t>::max()) {
    return false;
  }

  output = static_cast<std::size_t>(result);
  return true;
}

bool parseUnsignedInt(std::string_view value, unsigned int& output)
{
  if (value.empty() || value.front() == '-') {
    return false;
  }

  char* end = nullptr;
  const unsigned long result
      = std::strtoul(std::string(value).c_str(), &end, 10);
  if (!end || *end != '\0') {
    return false;
  }

  if (result > std::numeric_limits<unsigned int>::max()) {
    return false;
  }

  output = static_cast<unsigned int>(result);
  return true;
}

bool parseDouble(std::string_view value, double& output)
{
  if (value.empty()) {
    return false;
  }

  char* end = nullptr;
  const double result = std::strtod(std::string(value).c_str(), &end);
  if (!end || *end != '\0') {
    return false;
  }

  output = result;
  return true;
}

ParseResult parseArgs(int argc, char* argv[], Options& options)
{
  for (int i = 1; i < argc; ++i) {
    const std::string_view arg(argv[i]);

    if (arg == "-h" || arg == "--help") {
      printUsage(argv[0]);
      return ParseResult::Help;
    }

    if (arg == "--world" && i + 1 < argc) {
      options.worldUri = argv[++i];
      continue;
    }

    if (arg == "--steps" && i + 1 < argc) {
      if (!parseSizeT(argv[++i], options.steps) || options.steps == 0) {
        std::cerr << "Invalid steps value: " << argv[i] << "\n";
        printUsage(argv[0]);
        return ParseResult::Error;
      }
      continue;
    }

    if (arg == "--dt" && i + 1 < argc) {
      if (!parseDouble(argv[++i], options.timeStep)
          || options.timeStep <= 0.0) {
        std::cerr << "Invalid time step: " << argv[i] << "\n";
        printUsage(argv[0]);
        return ParseResult::Error;
      }
      options.setTimeStep = true;
      continue;
    }

    if (arg == "--seed" && i + 1 < argc) {
      if (!parseUnsignedInt(argv[++i], options.seed)) {
        std::cerr << "Invalid seed: " << argv[i] << "\n";
        printUsage(argv[0]);
        return ParseResult::Error;
      }
      continue;
    }

    std::cerr << "Unknown argument: " << arg << "\n";
    printUsage(argv[0]);
    return ParseResult::Error;
  }

  return ParseResult::Ok;
}

} // namespace

int main(int argc, char* argv[])
{
  Options options;
  const ParseResult parseResult = parseArgs(argc, argv, options);
  if (parseResult == ParseResult::Help) {
    return 0;
  }
  if (parseResult == ParseResult::Error) {
    return 1;
  }

  dart::math::Random::setSeed(options.seed);

  auto world = dart::io::readWorld(dart::common::Uri(options.worldUri));
  if (!world) {
    std::cerr << "Failed to read world from '" << options.worldUri << "'.\n";
    return 1;
  }

  if (options.setTimeStep) {
    world->setTimeStep(options.timeStep);
  }

  std::cout << "World loaded with " << world->getNumSkeletons()
            << " skeleton(s).\n"
            << "Time step: " << world->getTimeStep() << "\n"
            << "Steps: " << options.steps << "\n";

  const auto start = std::chrono::steady_clock::now();
  for (std::size_t i = 0; i < options.steps; ++i) {
    world->step();
  }
  const auto end = std::chrono::steady_clock::now();

  const std::chrono::duration<double> elapsed = end - start;
  std::cout << "Simulated time: " << world->getTime() << "\n"
            << "Elapsed wall time: " << elapsed.count() << "\n";

  return 0;
}
