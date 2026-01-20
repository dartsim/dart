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

#include <dart/io/read.hpp>

#include <algorithm>
#include <iostream>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <cctype>

namespace {

struct Options
{
  std::string worldUri = "dart://sample/sdf/double_pendulum.world";
  std::string skeletonUri = "dart://sample/urdf/test/primitive_geometry.urdf";
  bool loadWorld = true;
  bool loadSkeleton = true;
  dart::io::ModelFormat format = dart::io::ModelFormat::Auto;
  dart::io::RootJointType sdfRootJointType = dart::io::RootJointType::Floating;
  std::vector<std::pair<std::string, std::string>> packageMappings;
};

enum class ParseResult
{
  Ok,
  Help,
  Error
};

std::string toLower(std::string_view value)
{
  std::string result;
  result.reserve(value.size());
  for (char character : value) {
    result.push_back(
        static_cast<char>(std::tolower(static_cast<unsigned char>(character))));
  }
  return result;
}

const char* formatToString(dart::io::ModelFormat format)
{
  switch (format) {
    case dart::io::ModelFormat::Auto:
      return "auto";
    case dart::io::ModelFormat::Skel:
      return "skel";
    case dart::io::ModelFormat::Sdf:
      return "sdf";
    case dart::io::ModelFormat::Urdf:
      return "urdf";
    case dart::io::ModelFormat::Mjcf:
      return "mjcf";
  }
  return "unknown";
}

const char* rootJointTypeToString(dart::io::RootJointType type)
{
  switch (type) {
    case dart::io::RootJointType::Floating:
      return "floating";
    case dart::io::RootJointType::Fixed:
      return "fixed";
  }
  return "unknown";
}

bool parseModelFormat(std::string_view value, dart::io::ModelFormat& format)
{
  const std::string lower = toLower(value);
  if (lower == "auto") {
    format = dart::io::ModelFormat::Auto;
    return true;
  }
  if (lower == "skel") {
    format = dart::io::ModelFormat::Skel;
    return true;
  }
  if (lower == "sdf") {
    format = dart::io::ModelFormat::Sdf;
    return true;
  }
  if (lower == "urdf") {
    format = dart::io::ModelFormat::Urdf;
    return true;
  }
  if (lower == "mjcf") {
    format = dart::io::ModelFormat::Mjcf;
    return true;
  }
  return false;
}

bool parseRootJointType(std::string_view value, dart::io::RootJointType& type)
{
  const std::string lower = toLower(value);
  if (lower == "floating") {
    type = dart::io::RootJointType::Floating;
    return true;
  }
  if (lower == "fixed") {
    type = dart::io::RootJointType::Fixed;
    return true;
  }
  return false;
}

bool parsePackageMapping(
    std::string_view value, std::string& packageName, std::string& packagePath)
{
  const std::size_t equalSign = value.find('=');
  if (equalSign == std::string_view::npos)
    return false;

  packageName = std::string(value.substr(0, equalSign));
  packagePath = std::string(value.substr(equalSign + 1));

  return !packageName.empty() && !packagePath.empty();
}

void printUsage(const char* argv0)
{
  std::cout
      << "Usage: " << argv0 << " [options]\n\n"
      << "Options:\n"
      << "  --world <uri>           World URI to load\n"
      << "  --skeleton <uri>        Skeleton URI to load\n"
      << "  --format <auto|skel|sdf|urdf|mjcf>\n"
      << "                          Force the model format for both loads\n"
      << "  --sdf-root-joint <floating|fixed>\n"
      << "                          Default root joint type for SDF models\n"
      << "  --package <name=path>   Register a package:// mapping "
         "(repeatable)\n"
      << "  --no-world              Skip readWorld\n"
      << "  --no-skeleton           Skip readSkeleton\n"
      << "  -h, --help              Show this help\n";
}

ParseResult parseArgs(int argc, char* argv[], Options& options)
{
  for (int i = 1; i < argc; ++i) {
    const std::string_view arg(argv[i]);

    if (arg == "-h" || arg == "--help") {
      printUsage(argv[0]);
      return ParseResult::Help;
    }

    if (arg == "--no-world") {
      options.loadWorld = false;
      continue;
    }

    if (arg == "--no-skeleton") {
      options.loadSkeleton = false;
      continue;
    }

    if (arg == "--world" && i + 1 < argc) {
      options.worldUri = argv[++i];
      options.loadWorld = true;
      continue;
    }

    if (arg == "--skeleton" && i + 1 < argc) {
      options.skeletonUri = argv[++i];
      options.loadSkeleton = true;
      continue;
    }

    if (arg == "--format" && i + 1 < argc) {
      if (!parseModelFormat(argv[++i], options.format)) {
        std::cerr << "Unknown format: " << argv[i] << "\n";
        printUsage(argv[0]);
        return ParseResult::Error;
      }
      continue;
    }

    if (arg == "--sdf-root-joint" && i + 1 < argc) {
      if (!parseRootJointType(argv[++i], options.sdfRootJointType)) {
        std::cerr << "Unknown root joint type: " << argv[i] << "\n";
        printUsage(argv[0]);
        return ParseResult::Error;
      }
      continue;
    }

    if (arg == "--package" && i + 1 < argc) {
      std::string packageName;
      std::string packagePath;
      if (!parsePackageMapping(argv[++i], packageName, packagePath)) {
        std::cerr << "Invalid package mapping: " << argv[i]
                  << " (expected name=path)\n";
        printUsage(argv[0]);
        return ParseResult::Error;
      }
      options.packageMappings.emplace_back(
          std::move(packageName), std::move(packagePath));
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

  if (!options.loadWorld && !options.loadSkeleton) {
    std::cerr << "Nothing to load. Enable --world or --skeleton.\n";
    return 1;
  }

  dart::io::ReadOptions readOptions;
  readOptions.format = options.format;
  readOptions.sdfDefaultRootJointType = options.sdfRootJointType;
  for (const auto& mapping : options.packageMappings)
    readOptions.addPackageDirectory(mapping.first, mapping.second);

  std::cout << "Read options: format=" << formatToString(readOptions.format)
            << ", sdf-root-joint="
            << rootJointTypeToString(readOptions.sdfDefaultRootJointType)
            << "\n";

  bool ok = true;

  if (options.loadWorld) {
    const dart::common::Uri worldUri(options.worldUri);
    auto world = dart::io::readWorld(worldUri, readOptions);
    if (!world) {
      std::cerr << "Failed to read world from '" << options.worldUri << "'.\n";
      ok = false;
    } else {
      std::cout << "World loaded with " << world->getNumSkeletons()
                << " skeleton(s).\n";
    }
  }

  if (options.loadSkeleton) {
    const dart::common::Uri skeletonUri(options.skeletonUri);
    auto skeleton = dart::io::readSkeleton(skeletonUri, readOptions);
    if (!skeleton) {
      std::cerr << "Failed to read skeleton from '" << options.skeletonUri
                << "'.\n";
      ok = false;
    } else {
      std::cout << "Skeleton '" << skeleton->getName() << "' has "
                << skeleton->getNumDofs() << " DOFs.\n";
    }
  }

  return ok ? 0 : 1;
}
