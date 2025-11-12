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

#include "dart/utils/scene/SceneParser.hpp"

#include <dart/utils/SkelParser.hpp>

#include <dart/common/Logging.hpp>

#if defined(DART_SCENEPARSER_HAS_URDF)
  #include <dart/utils/urdf/DartLoader.hpp>
#endif

#include <algorithm>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include <cctype>

namespace dart {
namespace utils {
namespace SceneParser {

namespace {

using ParseResult = std::optional<FileContents>;

std::string toLower(std::string text)
{
  std::transform(
      text.begin(), text.end(), text.begin(), [](unsigned char ch) -> char {
        return static_cast<char>(std::tolower(ch));
      });
  return text;
}

std::string getPathLikeString(const common::Uri& uri)
{
  if (uri.mPath)
    return *uri.mPath;

  return uri.toString();
}

std::string getExtension(const common::Uri& uri)
{
  const std::string path = getPathLikeString(uri);
  const auto dot = path.find_last_of('.');
  if (dot == std::string::npos)
    return "";

  return toLower(path.substr(dot));
}

Format guessFormat(const common::Uri& uri)
{
  const std::string ext = getExtension(uri);

  if (ext == ".skel")
    return Format::Skel;
  if (ext == ".sdf" || ext == ".world" || ext == ".model")
    return Format::Sdf;
  if (ext == ".urdf")
    return Format::Urdf;
  if (ext == ".mjcf")
    return Format::Mjcf;
  if (ext == ".vsk")
    return Format::Vsk;

  return Format::Auto;
}

void appendUnique(std::vector<Format>& formats, Format format)
{
  if (std::find(formats.begin(), formats.end(), format) == formats.end())
    formats.push_back(format);
}

std::vector<Format> computeCandidates(
    const common::Uri& uri, const Options& options)
{
  std::vector<Format> formats;

  if (options.mFormatHint != Format::Auto) {
    appendUnique(formats, options.mFormatHint);
    return formats;
  }

  const Format guessed = guessFormat(uri);
  if (guessed != Format::Auto)
    appendUnique(formats, guessed);

  appendUnique(formats, Format::Skel);
  appendUnique(formats, Format::Sdf);
  appendUnique(formats, Format::Urdf);
  appendUnique(formats, Format::Mjcf);
  appendUnique(formats, Format::Vsk);

  return formats;
}

void addSkeleton(FileContents& contents, const dynamics::SkeletonPtr& skeleton)
{
  if (!skeleton)
    return;

  const auto duplicate = std::find(
      contents.skeletons.begin(), contents.skeletons.end(), skeleton);

  if (duplicate == contents.skeletons.end())
    contents.skeletons.push_back(skeleton);
}

void addWorld(FileContents& contents, const simulation::WorldPtr& world)
{
  if (!world)
    return;

  contents.worlds.push_back(world);
  for (std::size_t i = 0; i < world->getNumSkeletons(); ++i)
    addSkeleton(contents, world->getSkeleton(i));
}

ParseResult parseSkel(const common::Uri& uri, const Options& options)
{
  const FileContents contents
      = SkelParser::readFile(uri, options.mResourceRetriever);

  if (!contents.empty())
    return contents;

  return std::nullopt;
}

ParseResult parseSdf(const common::Uri& uri, const Options& options)
{
  auto sdfOptions = options.mSdfOptions;
  if (!sdfOptions.mResourceRetriever)
    sdfOptions.mResourceRetriever = options.mResourceRetriever;

  FileContents contents;

  if (auto world = SdfParser::readWorld(uri, sdfOptions)) {
    addWorld(contents, world);
    return contents;
  }

  if (auto skeleton = SdfParser::readSkeleton(uri, sdfOptions)) {
    addSkeleton(contents, skeleton);
    return contents;
  }

  return std::nullopt;
}

#if defined(DART_SCENEPARSER_HAS_URDF)
ParseResult parseUrdf(const common::Uri& uri, const Options& options)
{
  auto urdfOptions = options.mUrdfOptions;
  if (!urdfOptions.mResourceRetriever)
    urdfOptions.mResourceRetriever = options.mResourceRetriever;

  FileContents contents;
  DartLoader loader(urdfOptions);

  if (auto world = loader.parseWorld(uri)) {
    addWorld(contents, world);
    return contents;
  }

  if (auto skeleton = loader.parseSkeleton(uri)) {
    addSkeleton(contents, skeleton);
    return contents;
  }

  return std::nullopt;
}
#else
ParseResult parseUrdf(const common::Uri&, const Options&)
{
  return std::nullopt;
}
#endif

ParseResult parseMjcf(const common::Uri& uri, const Options& options)
{
  auto mjcfOptions = options.mMjcfOptions;
  if (!mjcfOptions.mRetriever)
    mjcfOptions.mRetriever = options.mResourceRetriever;

  FileContents contents;
  if (auto world = MjcfParser::readWorld(uri, mjcfOptions)) {
    addWorld(contents, world);
    return contents;
  }
  return std::nullopt;
}

ParseResult parseVsk(const common::Uri& uri, const Options& options)
{
  auto vskOptions = options.mVskOptions;
  if (!vskOptions.retrieverOrNullptr)
    vskOptions.retrieverOrNullptr = options.mResourceRetriever;

  FileContents contents;
  if (auto skeleton = VskParser::readSkeleton(uri, vskOptions)) {
    addSkeleton(contents, skeleton);
    return contents;
  }

  return std::nullopt;
}

ParseResult parseWithFormat(
    Format format, const common::Uri& uri, const Options& options)
{
  switch (format) {
    case Format::Skel:
      return parseSkel(uri, options);
    case Format::Sdf:
      return parseSdf(uri, options);
    case Format::Urdf:
      return parseUrdf(uri, options);
    case Format::Mjcf:
      return parseMjcf(uri, options);
    case Format::Vsk:
      return parseVsk(uri, options);
    case Format::Auto:
      return std::nullopt;
  }

  return std::nullopt;
}

std::string formatName(Format format)
{
  switch (format) {
    case Format::Skel:
      return "SKEL";
    case Format::Sdf:
      return "SDF";
    case Format::Urdf:
      return "URDF";
    case Format::Mjcf:
      return "MJCF";
    case Format::Vsk:
      return "VSK";
    case Format::Auto:
      return "AUTO";
  }
  return "UNKNOWN";
}

std::string listTriedFormats(const std::vector<Format>& formats)
{
  std::ostringstream stream;
  for (std::size_t i = 0; i < formats.size(); ++i) {
    if (i != 0)
      stream << ", ";
    stream << formatName(formats[i]);
  }
  return stream.str();
}

} // namespace

//==============================================================================
FileContents readFile(const common::Uri& uri, const Options& options)
{
  const auto candidates = computeCandidates(uri, options);
  std::vector<Format> attempted;

  for (Format format : candidates) {
    if (auto contents = parseWithFormat(format, uri, options))
      return *contents;
    attempted.push_back(format);
  }

  DART_ERROR(
      "[SceneParser::readFile] Failed to parse [{}] with formats: {}",
      uri.toString(),
      listTriedFormats(attempted));

  return {};
}

} // namespace SceneParser
} // namespace utils
} // namespace dart
