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

#include <Eigen/Geometry>

#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <string_view>

#include <cstdlib>

namespace {

struct Options
{
  std::string worldUri = "dart://sample/sdf/double_pendulum.world";
  std::string outputPath = "simulation_log.csv";
  std::size_t steps = 1000;
  double timeStep = 0.001;
  bool setTimeStep = false;
  std::string skeletonName;
  std::string bodyName;
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
            << "  --world <uri>      World URI to load\n"
            << "  --output <path>    Output CSV path\n"
            << "  --steps <n>        Number of simulation steps\n"
            << "  --dt <seconds>     Override world time step\n"
            << "  --skeleton <name>  Skeleton name to log (default: first)\n"
            << "  --body <name>      Body name to log (default: first)\n"
            << "  -h, --help         Show this help\n";
}

bool parseSizeT(std::string_view value, std::size_t& output)
{
  if (value.empty() || value.front() == '-')
    return false;

  char* end = nullptr;
  const unsigned long long result
      = std::strtoull(std::string(value).c_str(), &end, 10);
  if (!end || *end != '\0')
    return false;

  if (result > std::numeric_limits<std::size_t>::max())
    return false;

  output = static_cast<std::size_t>(result);
  return true;
}

bool parseDouble(std::string_view value, double& output)
{
  if (value.empty())
    return false;

  char* end = nullptr;
  const double result = std::strtod(std::string(value).c_str(), &end);
  if (!end || *end != '\0')
    return false;

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

    if (arg == "--output" && i + 1 < argc) {
      options.outputPath = argv[++i];
      continue;
    }

    if (arg == "--steps" && i + 1 < argc) {
      if (!parseSizeT(argv[++i], options.steps)) {
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

    if (arg == "--skeleton" && i + 1 < argc) {
      options.skeletonName = argv[++i];
      continue;
    }

    if (arg == "--body" && i + 1 < argc) {
      options.bodyName = argv[++i];
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
  if (parseResult == ParseResult::Help)
    return 0;
  if (parseResult == ParseResult::Error)
    return 1;

  auto world = dart::io::readWorld(dart::common::Uri(options.worldUri));
  if (!world) {
    std::cerr << "Failed to read world from '" << options.worldUri << "'.\n";
    return 1;
  }

  if (options.setTimeStep)
    world->setTimeStep(options.timeStep);

  dart::dynamics::SkeletonPtr skeleton;
  if (options.skeletonName.empty()) {
    if (world->getNumSkeletons() > 0)
      skeleton = world->getSkeleton(0);
  } else {
    skeleton = world->getSkeleton(options.skeletonName);
  }

  if (!skeleton) {
    std::cerr << "Skeleton not found.\n";
    return 1;
  }

  dart::dynamics::BodyNode* bodyNode = nullptr;
  if (options.bodyName.empty()) {
    if (skeleton->getNumBodyNodes() > 0)
      bodyNode = skeleton->getBodyNode(0);
  } else {
    bodyNode = skeleton->getBodyNode(options.bodyName);
  }

  if (!bodyNode) {
    std::cerr << "Body node not found.\n";
    return 1;
  }

  std::ofstream output(options.outputPath);
  if (!output) {
    std::cerr << "Failed to open output file: " << options.outputPath << "\n";
    return 1;
  }

  output << "time,com_x,com_y,com_z,"
         << "body_x,body_y,body_z,"
         << "body_qw,body_qx,body_qy,body_qz\n";
  output << std::fixed << std::setprecision(6);

  auto logRow = [&]() {
    const double time = world->getTime();
    const Eigen::Vector3d com = skeleton->getCOM();
    const Eigen::Isometry3d bodyTransform = bodyNode->getWorldTransform();
    const Eigen::Vector3d bodyPosition = bodyTransform.translation();
    const Eigen::Quaterniond bodyOrientation(bodyTransform.linear());

    output << time << ',' << com.x() << ',' << com.y() << ',' << com.z() << ','
           << bodyPosition.x() << ',' << bodyPosition.y() << ','
           << bodyPosition.z() << ',' << bodyOrientation.w() << ','
           << bodyOrientation.x() << ',' << bodyOrientation.y() << ','
           << bodyOrientation.z() << '\n';
  };

  logRow();
  for (std::size_t i = 0; i < options.steps; ++i) {
    world->step();
    logRow();
  }

  std::cout << "Wrote " << (options.steps + 1) << " row(s) to '"
            << options.outputPath << "'.\n";
  return 0;
}
