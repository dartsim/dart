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

#include "dart/io/Read.hpp"

#include "dart/common/LocalResourceRetriever.hpp"
#include "dart/common/Logging.hpp"
#include "dart/common/String.hpp"
#include "dart/utils/CompositeResourceRetriever.hpp"
#include "dart/utils/DartResourceRetriever.hpp"
#include "dart/utils/SkelParser.hpp"
#include "dart/utils/VskParser.hpp"
#include "dart/utils/mjcf/MjcfParser.hpp"
#include "dart/utils/sdf/SdfParser.hpp"

#if DART_IO_HAS_URDF
  #include "dart/utils/urdf/UrdfParser.hpp"
#endif

#include <tinyxml2.h>

#include <optional>
#include <stdexcept>

namespace dart {
namespace io {

namespace {

common::ResourceRetrieverPtr getRetriever(
    const common::ResourceRetrieverPtr& retrieverOrNullptr)
{
  if (retrieverOrNullptr)
    return retrieverOrNullptr;

  auto composite = std::make_shared<utils::CompositeResourceRetriever>();
  composite->addSchemaRetriever(
      "file", std::make_shared<common::LocalResourceRetriever>());
  composite->addSchemaRetriever("dart", utils::DartResourceRetriever::create());
  return composite;
}

std::string getLowercaseExtension(const common::Uri& uri)
{
  std::string path;
  if (uri.mPath)
    path = uri.mPath.get();
  else
    path = uri.toString();

  const auto slash = path.find_last_of("/\\");
  const auto dot = path.find_last_of('.');
  if (dot == std::string::npos)
    return {};
  if (slash != std::string::npos && dot < slash)
    return {};
  return common::toLower(path.substr(dot));
}

std::optional<ModelFormat> inferFormatFromExtension(const common::Uri& uri)
{
  const auto ext = getLowercaseExtension(uri);
  if (ext == ".skel")
    return ModelFormat::Skel;
  if (ext == ".sdf" || ext == ".world")
    return ModelFormat::Sdf;
  if (ext == ".urdf")
    return ModelFormat::Urdf;
  if (ext == ".mjcf")
    return ModelFormat::Mjcf;
  if (ext == ".vsk")
    return ModelFormat::Vsk;

  // Extensions like ".xml" are ambiguous across multiple formats.
  return std::nullopt;
}

std::optional<ModelFormat> inferFormatFromXmlRoot(
    const common::Uri& uri, const common::ResourceRetrieverPtr& retriever)
{
  std::string content;
  try {
    content = retriever->readAll(uri);
  } catch (const std::exception& e) {
    DART_ERROR(
        "[dart::io::readWorld/readSkeleton] Failed reading [{}]: {}",
        uri.toString(),
        e.what());
    return std::nullopt;
  }

  if (content.empty()) {
    DART_ERROR(
        "[dart::io::readWorld/readSkeleton] Failed reading [{}]: empty content",
        uri.toString());
    return std::nullopt;
  }

  tinyxml2::XMLDocument doc;
  const auto result = doc.Parse(content.c_str(), content.size());
  if (result != tinyxml2::XML_SUCCESS) {
    DART_ERROR(
        "[dart::io::readWorld/readSkeleton] Failed parsing XML [{}]: {}",
        uri.toString(),
        doc.ErrorStr());
    return std::nullopt;
  }

  const auto* root = doc.RootElement();
  if (!root) {
    DART_ERROR(
        "[dart::io::readWorld/readSkeleton] Failed parsing XML [{}]: missing "
        "root element",
        uri.toString());
    return std::nullopt;
  }

  const std::string rootName = root->Name();
  if (rootName == "skel")
    return ModelFormat::Skel;
  if (rootName == "sdf")
    return ModelFormat::Sdf;
  if (rootName == "robot")
    return ModelFormat::Urdf;
  if (rootName == "mujoco")
    return ModelFormat::Mjcf;
  if (rootName == "KinematicModel")
    return ModelFormat::Vsk;

  return std::nullopt;
}

std::optional<ModelFormat> inferFormat(
    const common::Uri& uri, const common::ResourceRetrieverPtr& retriever)
{
  if (const auto ext = inferFormatFromExtension(uri))
    return ext;

  return inferFormatFromXmlRoot(uri, retriever);
}

ReadOptions resolveOptions(const ReadOptions& options)
{
  ReadOptions resolved = options;
  resolved.resourceRetriever = getRetriever(options.resourceRetriever);
  return resolved;
}

#if DART_IO_HAS_URDF
simulation::WorldPtr readUrdfWorld(
    const common::Uri& uri, const ReadOptions& options)
{
  utils::UrdfParser parser(
      utils::UrdfParser::Options(options.resourceRetriever));
  for (const auto& [packageName, packageDirectory] :
       options.urdfPackageDirectories) {
    parser.addPackageDirectory(packageName, packageDirectory);
  }
  return parser.parseWorld(uri);
}

dynamics::SkeletonPtr readUrdfSkeleton(
    const common::Uri& uri, const ReadOptions& options)
{
  utils::UrdfParser parser(
      utils::UrdfParser::Options(options.resourceRetriever));
  for (const auto& [packageName, packageDirectory] :
       options.urdfPackageDirectories) {
    parser.addPackageDirectory(packageName, packageDirectory);
  }
  return parser.parseSkeleton(uri);
}
#endif

} // namespace

//==============================================================================
simulation::WorldPtr readWorld(
    const common::Uri& uri, const ReadOptions& options)
{
  const auto resolved = resolveOptions(options);

  ModelFormat format = resolved.format;
  if (format == ModelFormat::Auto) {
    const auto inferred = inferFormat(uri, resolved.resourceRetriever);
    if (!inferred) {
      DART_ERROR(
          "[dart::io::readWorld] Failed inferring model format from URI [{}]",
          uri.toString());
      return nullptr;
    }
    format = *inferred;
  }

  switch (format) {
    case ModelFormat::Skel:
      return utils::SkelParser::readWorld(uri, resolved.resourceRetriever);
    case ModelFormat::Sdf:
      return utils::SdfParser::readWorld(
          uri, utils::SdfParser::Options(resolved.resourceRetriever));
    case ModelFormat::Mjcf:
      return utils::MjcfParser::readWorld(
          uri, utils::MjcfParser::Options(resolved.resourceRetriever));
    case ModelFormat::Urdf:
#if DART_IO_HAS_URDF
      return readUrdfWorld(uri, resolved);
#else
      DART_ERROR(
          "[dart::io::readWorld] URDF support is not available. Build and link "
          "against dart-utils-urdf to read URDF files. URI=[{}]",
          uri.toString());
      return nullptr;
#endif
    case ModelFormat::Vsk:
      DART_ERROR(
          "[dart::io::readWorld] VSK does not support world parsing. URI=[{}]",
          uri.toString());
      return nullptr;
    case ModelFormat::Auto:
      break;
  }

  DART_ERROR(
      "[dart::io::readWorld] Unsupported model format for URI [{}]",
      uri.toString());
  return nullptr;
}

//==============================================================================
dynamics::SkeletonPtr readSkeleton(
    const common::Uri& uri, const ReadOptions& options)
{
  const auto resolved = resolveOptions(options);

  ModelFormat format = resolved.format;
  if (format == ModelFormat::Auto) {
    const auto inferred = inferFormat(uri, resolved.resourceRetriever);
    if (!inferred) {
      DART_ERROR(
          "[dart::io::readSkeleton] Failed inferring model format from URI "
          "[{}]",
          uri.toString());
      return nullptr;
    }
    format = *inferred;
  }

  switch (format) {
    case ModelFormat::Skel: {
      const auto world
          = utils::SkelParser::readWorld(uri, resolved.resourceRetriever);
      if (world) {
        if (world->getNumSkeletons() == 0)
          return nullptr;

        if (world->getNumSkeletons() > 1) {
          DART_WARN(
              "[dart::io::readSkeleton] Loaded a Skel world containing "
              "multiple "
              "skeletons; returning the first skeleton. URI=[{}]",
              uri.toString());
        }

        return world->getSkeleton(0);
      }

      return utils::SkelParser::readSkeleton(uri, resolved.resourceRetriever);
    }
    case ModelFormat::Sdf:
      return utils::SdfParser::readSkeleton(
          uri, utils::SdfParser::Options(resolved.resourceRetriever));
    case ModelFormat::Vsk:
      return utils::VskParser::readSkeleton(
          uri, utils::VskParser::Options(resolved.resourceRetriever));
    case ModelFormat::Urdf:
#if DART_IO_HAS_URDF
      return readUrdfSkeleton(uri, resolved);
#else
      DART_ERROR(
          "[dart::io::readSkeleton] URDF support is not available. Build and "
          "link against dart-utils-urdf to read URDF files. URI=[{}]",
          uri.toString());
      return nullptr;
#endif
    case ModelFormat::Mjcf:
      DART_ERROR(
          "[dart::io::readSkeleton] MJCF does not support direct skeleton "
          "parsing. URI=[{}]",
          uri.toString());
      return nullptr;
    case ModelFormat::Auto:
      break;
  }

  DART_ERROR(
      "[dart::io::readSkeleton] Unsupported model format for URI [{}]",
      uri.toString());
  return nullptr;
}

} // namespace io
} // namespace dart
